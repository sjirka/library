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
	MStatus offsetEdgeloop(SEdgeLoop &edgeLoop, float offsetDistance, bool createPolygons = true);
	MStatus offsetEdgeloops(float offsetDistance, bool createPolygons = true);
	MStatus setHardEdges(MIntArray& edges, double tresholdAngle);

	void	getEdgeMap(std::map <unsigned int, unsigned int> &edgeMap);

protected:
	std::map <unsigned int, unsigned int> m_edgeMap;
};