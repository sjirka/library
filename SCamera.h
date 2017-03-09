#pragma once

#include <maya\MDagPath.h>
#include <maya\M3dView.h>
#include <maya\MFnCamera.h>
#include <maya\MDistance.h>
#include <maya\MGlobal.h>

class SCamera
{
public:
	SCamera() {};

	static double scaleFactor(M3dView& view, MPoint& point) {
		MDagPath dPath;
		view.getCamera(dPath);
		MFnCamera fnCamera(dPath);

		MPoint eye = fnCamera.eyePoint(MSpace::kWorld);
		MVector direction = fnCamera.viewDirection(MSpace::kWorld);

		MDistance distance = (fnCamera.isOrtho())?
			MDistance(fnCamera.orthoWidth(), MDistance::internalUnit()):
			MDistance(direction * (point - eye), MDistance::internalUnit());
		double scaleFactor = distance.asMillimeters() / fnCamera.focalLength();
		return scaleFactor;
	};
};