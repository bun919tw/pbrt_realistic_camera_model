
#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_CAMERAS_REALISTIC_H
#define PBRT_CAMERAS_REALISTIC_H

#include "camera.h"
#include "paramset.h"
#include "film.h"
using namespace std;

struct Len {
    // the z of len
    float z;
    // the radius of len
    float radius;
    // the n of len
    float n;
    // the aperture value of len
    float aperture;
    // true if it's a stop
    bool isStop;
    // the radius center
    Point center;
};

// RealisticCamera Declarations
class RealisticCamera : public Camera {
public:
	// RealisticCamera Public Methods
	RealisticCamera(const AnimatedTransform &cam2world,
						float hither, float yon, float sopen,
						float sclose, float filmdistance, float aperture_diameter, string specfile,
						float filmdiag, Film *film);
	float GenerateRay(const CameraSample &sample, Ray *) const;
  
private:
	// RealisticCamera Public Methods
    float Hither, Yon, distance;
    vector<Len> lens;
    Transform RasterToCamera;
    void ParseSpec(const string &file);
};


RealisticCamera *CreateRealisticCamera(const ParamSet &params,
        const AnimatedTransform &cam2world, Film *film);


#endif	// PBRT_CAMERAS_REALISTIC_H