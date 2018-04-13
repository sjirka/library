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

	static double scaleFactor(M3dView& view, const MPoint& point) {
		MDagPath cameraPath;
		view.getCamera(cameraPath);
		
		return scaleFactor(cameraPath, point);
	};

	static double scaleFactor(MDagPath& camera, const MPoint& point) {
		MFnCamera fnCamera(camera);

		MPoint eye = fnCamera.eyePoint(MSpace::kWorld);
		MVector direction = fnCamera.viewDirection(MSpace::kWorld);

		double distance = (fnCamera.isOrtho()) ? fnCamera.orthoWidth() : direction * (point - eye);
		return distance / fnCamera.focalLength();
	};
};