
#include "stdafx.h"
#include "cameras/realistic.h"
#include "paramset.h"
#include "sampler.h"
#include <fstream>
#include <iostream>
#include <cmath>


RealisticCamera::RealisticCamera(const AnimatedTransform &cam2world,
				 float hither, float yon, 
				 float sopen, float sclose, 
				 float filmdistance, float aperture_diameter, string specfile, 
				 float filmdiag, Film *f)
	: Camera(cam2world, sopen, sclose, f) // pbrt-v2 doesnot specify hither and yon
{
    distance = 0;
    Hither = hither;
    Yon = yon;

    ParseSpec(specfile);

    // distance -> film position
    distance -= filmdistance;

    // compute the ratio between raster space and camera space
    float diag = sqrtf(f->xResolution * f->xResolution + f->yResolution * f->yResolution);
    float ratio = filmdiag / diag;
    // X and Y is in the Screen Space coordinate
    float X = ratio * 0.5 * float(f->xResolution);
    float Y = ratio * 0.5 * float(f->yResolution);
    RasterToCamera = Translate(Vector(0.f, 0.f, distance)) *
                     Translate(Vector(X, -Y, 0.f)) *
                     Scale(ratio, ratio, 1) *
                     Scale(-1.f, 1.f, 1.f);
}


float RealisticCamera::GenerateRay(const CameraSample &sample, Ray *ray) const {
    Point Pras(sample.imageX, sample.imageY, 0);
    Point Pcamera;
    RasterToCamera(Pras, &Pcamera);

    int final = lens.size() - 1;

    // Sample point using slide 56
    float lensU, lensV, lensZ;
    float r = lens[final].aperture * sqrtf(sample.lensU);
    float theta = 2 * M_PI * sample.lensV;
    lensU = r * cosf(theta);
    lensV = r * sinf(theta);

    // Calculate the z(depth) of sample point.
    float z = sqrtf(lens[final].radius * lens[final].radius - lensV * lensV - lensU * lensU);
    if (lens[final].radius < 0) {
        lensZ = lens[final].z - lens[final].radius - z;
    } else {
        lensZ = lens[final].z - lens[final].radius + z;
    }

    // Generate ray from film back to the last len.
    Point hit = Point(lensU, lensV, lensZ);
    Vector T = hit - Pcamera;

    // Tracing the ray
    for(int i = final; i >= 0; --i){
        if (lens[i].isStop) {
            float deltaZ = lens[i].z - hit.z;
            T = Normalize(T);
            float t = deltaZ / T.z;
            hit = hit + t * T;
            if (lens[i].aperture * lens[i].aperture < hit.x * hit.x + hit.y * hit.y) {
                return 0.f;
            }
        }
        else {
            // We want to make a triangle.
            // First, we need to make the first edge
            Vector c2hit = hit - lens[i].center;

            // Project vector c2hit to vector T to make the second edge.
            Vector normal_T = Normalize(T);
            float projection_distance = Dot(c2hit, normal_T);
            float normal_distance = sqrtf(c2hit.LengthSquared() - projection_distance * projection_distance);

            // The ray won't intersect with lens[i].
            if (normal_distance > abs(lens[i].radius)) {
                return 0.f;
            }

            // Find the intersection point.
            float temp = sqrtf(lens[i].radius * lens[i].radius - normal_distance * normal_distance);
            float forward_distance = 0.f;
            if (lens[i].radius > 0.f) {
                forward_distance = -projection_distance + temp;
            } else {
                forward_distance = -projection_distance - temp;
            }
            hit = hit + forward_distance * normal_T;

            // Even if we find the intersection point, we need to check it's on lens[i]
            if (lens[i].aperture * lens[i].aperture < hit.x * hit.x + hit.y * hit.y) {
                return 0.f;
            }

            // Prepare for Heckber's Method
            float n2 = (i == 0) ? 1 : lens[i - 1].n;
            Vector N;
            if (lens[i].radius > 0.f) {
                N = Normalize(lens[i].center - hit);
            } else {
                N = Normalize(hit - lens[i].center);
            }

            // Calculate cos(theta1), cos(theta2), and new T
            float n_ratio = lens[i].n / n2;
            float c1 = -Dot(normal_T, N);  // cos(theta1)
            float c2 = 1.f - n_ratio * n_ratio * (1.f - c1 * c1);   // cos(theta2)
            if (c2 <= 0.f) return 0.f;
            else c2 = sqrtf(c2);
            T = n_ratio * normal_T + (n_ratio * c1 - c2) * N;
        }
    }
    ray->o = hit;
    ray->d = Normalize(T);
    ray->mint = Hither;
    ray->maxt = (Yon - Hither) / ray->d.z;
    ray->time = Lerp(sample.time, shutterOpen, shutterClose);
    CameraToWorld(*ray, ray);

    // Set exposure weight by paper.
    float Lcos = Dot(Normalize(hit - Pcamera), Vector(0, 0, 1));
    float Zsqu = abs(distance) * abs(distance);
    float area = (lens[0].aperture * lens[0].aperture * M_PI);
    float weight = Lcos * Lcos * Lcos * Lcos * area / Zsqu;

    return weight;
}


void RealisticCamera::ParseSpec(const string &file){
    std::ifstream inputFile(file);
    std::string line;

    while(getline(inputFile, line)) {
        if (!line.length() || line[0] == '#')
            continue;
        float radius = 0, axpos = 0, N = 0, aperture = 0;
        sscanf(line.c_str(), "%f%f%f%f", &radius, &axpos, &N, &aperture);

        // Fill in the lens data.
        Len len;
        len.isStop = (N == 0);
        len.z = distance;
        len.radius = radius;
        len.n = (N == 0) ? 1 : N;
        len.aperture = aperture * 0.5;
        len.center = Point(0.f, 0.f, len.z - len.radius);

        // add len thick to distance for next len
        distance -= axpos;

        lens.push_back(len);
    }

    return ;
}


RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film) {
	// Extract common camera parameters from \use{ParamSet}
	float hither = params.FindOneFloat("hither", -1);
	float yon = params.FindOneFloat("yon", -1);
	float shutteropen = params.FindOneFloat("shutteropen", -1);
	float shutterclose = params.FindOneFloat("shutterclose", -1);

	// Realistic camera-specific parameters
	string specfile = params.FindOneString("specfile", "");
	float filmdistance = params.FindOneFloat("filmdistance", 70.0); // about 70 mm default to film
 	float fstop = params.FindOneFloat("aperture_diameter", 1.0);	
	float filmdiag = params.FindOneFloat("filmdiag", 35.0);

	Assert(hither != -1 && yon != -1 && shutteropen != -1 &&
		shutterclose != -1 && filmdistance!= -1);
	if (specfile == "") {
	    Severe( "No lens spec file supplied!\n" );
	}
	return new RealisticCamera(cam2world, hither, yon,
				   shutteropen, shutterclose, filmdistance, fstop, 
				   specfile, filmdiag, film);
}
