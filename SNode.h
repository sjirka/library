#pragma once

#include <maya\MStatus.h>
#include <maya\MObject.h>
#include <maya\MFnDagNode.h>
#include <maya\MDagModifier.h>
#include <maya\MItDependencyNodes.h>

class SNode
{
public:
	SNode() {};
	~SNode() {};

	static MStatus SNode::createDagNode(const MString &type, const MString &name, MDagModifier &dagModifier, MObject& node, MObject& parent) {
		MStatus status;

		node = dagModifier.createNode(type, parent, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MFnDagNode fnNode(node, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		fnNode.setName(name);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;
	}

	static MStatus SNode::getPluginNode(const MString& name, const MTypeId &id, MObject& node) {
		MStatus status;

		MItDependencyNodes itNodes(MFn::kPluginDependNode);

		for (itNodes.reset(); !itNodes.isDone(); itNodes.next()) {
			MFnDependencyNode fnNode(itNodes.thisNode(), &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			if (id != fnNode.typeId())
				continue;

			if (fnNode.name() == name) {
				node = itNodes.thisNode();
				return MS::kSuccess;
			}
		}

		return MS::kFailure;
	}


	static MStatus SNode::createDagGroup(const MString& name, MDagModifier &dagModifier, MObject &node, MObject& transform) {
		MStatus status;

		status = createDagNode("transform", name, dagModifier, transform, MObject::kNullObj);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = createDagNode("mesh", name + "Shape", dagModifier, node, transform);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;
	}

private:

};