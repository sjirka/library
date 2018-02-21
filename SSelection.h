#pragma once

#include <maya\MGlobal.h>
#include <maya\MSelectionList.h>
#include <maya\MRichSelection.h>

struct SSelection{
	MSelectionList
		activeList,
		hiliteList;
	MRichSelection
		richList;
	MGlobal::MSelectionMode
		selectionMode;
	MSelectionMask
		objectMask,
		componentMask,
		animMask;
	bool
		hasRichSelection;

	MStatus storeCurrentSelection() {
		MStatus status;

		status = MGlobal::getActiveSelectionList(activeList);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = MGlobal::getHiliteList(hiliteList);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = MGlobal::getRichSelection(richList, false);
		hasRichSelection = (status == MS::kSuccess) ? true : false;
		selectionMode = MGlobal::selectionMode(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		componentMask = MGlobal::componentSelectionMask(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		objectMask = MGlobal::objectSelectionMask(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		animMask = MGlobal::animSelectionMask(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;
	};

	MStatus restoreSelection() {
		MStatus status;

		status = MGlobal::setSelectionMode(selectionMode);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = MGlobal::setComponentSelectionMask(componentMask);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = MGlobal::setAnimSelectionMask(animMask);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = MGlobal::setObjectSelectionMask(objectMask);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = MGlobal::setActiveSelectionList(activeList);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		status = MGlobal::setHiliteList(hiliteList);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		if (hasRichSelection) {
			status = MGlobal::setRichSelection(richList);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}

		return MS::kSuccess;
	};
};