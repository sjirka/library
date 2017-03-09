#pragma once

#include <maya/MObject.h>

class SData
{
public:
	SData() {};
	~SData() {};

	static bool isCurve(const MObject& object) {
		MFn::Type type = object.apiType();
		if (type == MFn::kNurbsCurve ||
			type == MFn::kNurbsCurveData ||
			type == MFn::kNurbsCurveGeom)
			return true;
		return false;
	};

	static bool isMesh(const MObject& object) {
		MFn::Type type = object.apiType();
		if (type == MFn::kMesh ||
			type == MFn::kMeshData ||
			type == MFn::kMeshGeom)
			return true;
		return false;
	}

	static bool isNurbs(const MObject& object) {
		MFn::Type type = object.apiType();
		if (type == MFn::kNurbsSurface ||
			type == MFn::kNurbsSurfaceData ||
			type == MFn::kNurbsSurfaceGeom)
			return true;
		return false;
	}

	static bool isSubd(const MObject& object) {
		MFn::Type type = object.apiType();
		if (type == MFn::kSubdiv ||
			type == MFn::kSubdivData ||
			type == MFn::kSubdivGeom)
			return true;
		return false;
	}

private:

};