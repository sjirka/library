#pragma once

#include "SMesh.h"

class SSeamMesh : public SMesh
{
public:
	SSeamMesh();
	SSeamMesh(MObject& obj, MStatus *ref = NULL);
	SSeamMesh(SMesh &mesh);
	SSeamMesh(const SSeamMesh &mesh);
	SSeamMesh& operator=(const SSeamMesh& mesh);
	~SSeamMesh();

	MStatus transferEdges(const MObject& sourceMesh, const MIntArray &edges);
	MStatus offsetEdgeloops(float offsetDistance, bool createPolygons = true);

protected:

	MStatus offsetEdgeloop(SEdgeLoop &edgeLoop, float offsetDistance, bool createPolygons = true);
};