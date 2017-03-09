#pragma once

#include <maya\MStatus.h>
#include <maya\MObject.h>
#include <maya\MDagPath.h>
#include <maya\MFnSet.h>

class SShadingGroup
{
public:
	SShadingGroup() {};
	~SShadingGroup() {};

	// Method by Bryan Ewert, maya@ewertb.com
	static MStatus SShadingGroup::AssignToShadingGroup(const MObject & shadingGroup, const MDagPath & dagPath, const MObject & component=MObject::kNullObj) {
		MStatus                     status;
		MFnSet                      fnSG(shadingGroup, &status);

		if (fnSG.restriction() != MFnSet::kRenderableOnly)
			return MS::kFailure;

		status = fnSG.addMember(dagPath, component);
		return status;
	}

private:

};