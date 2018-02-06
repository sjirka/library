#include "SSeamMesh.h"

SSeamMesh::SSeamMesh() {};

SSeamMesh::SSeamMesh(MObject &obj, MStatus *ref) : SMesh(obj) {
	if (obj.apiType() != MFn::kMeshData)
		*ref = MS::kInvalidParameter;
};

SSeamMesh::SSeamMesh(SMesh &mesh) :SMesh(mesh) {
};

SSeamMesh::SSeamMesh(const SSeamMesh &mesh) :SMesh(mesh) {
};

SSeamMesh& SSeamMesh::operator=(const SSeamMesh& mesh) {
	SMesh::operator=(mesh);
	return *this;
}

SSeamMesh::~SSeamMesh() {
}

MStatus SSeamMesh::transferEdges(const MObject& sourceMesh, const MIntArray &edges) {
	MStatus status;

	MFnMesh fnSrcMesh(sourceMesh);
	MFnMesh fnTrgMesh(m_mesh);

	std::map <unsigned int, std::set<unsigned int>> srcEdges, trgEdges;

	// Src edge vtx map
	for (unsigned int e = 0; e < edges.length(); e++) {
		int vertices[2];
		status = fnSrcMesh.getEdgeVertices(edges[e], vertices);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		srcEdges[e].insert(vertices[0]);
		srcEdges[e].insert(vertices[1]);
	}

	// Trg edge vtx map
	for (int e = 0; e < fnTrgMesh.numEdges(); e++) {
		int vertices[2];
		status = fnTrgMesh.getEdgeVertices(e, vertices);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		trgEdges[e].insert(m_vtxMap[vertices[0]]);
		trgEdges[e].insert(m_vtxMap[vertices[1]]);
	}

	MIntArray newEdges;
	for (auto &trgEdge : trgEdges)
		for (auto &srcEdge : srcEdges)
			if (trgEdge.second == srcEdge.second) {
				newEdges.append(trgEdge.first);
				break;
			}

	status = setActiveEdges(newEdges);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

MStatus SSeamMesh::offsetEdgeloops(float offsetDistance, bool createPolygons) {
	MStatus status;

	for (auto &loop : m_activeLoops) {
		status = offsetEdgeloop(loop, offsetDistance, createPolygons);
		CHECK_MSTATUS_AND_RETURN_IT(status);
	}

	return MS::kSuccess;
}

MStatus SSeamMesh::setHardEdges(MIntArray& edges, double tresholdAngle) {
	MStatus status;

	MFnMesh fnMesh(m_mesh);

	// Put edges in set for faster search
	std::set <unsigned int> hardEdges;
	for (unsigned int e = 0; e < edges.length(); e++)
		hardEdges.insert(edges[e]);

	// Now we're going to generate new ids for vertices on detached edges
	MItMeshVertex itVertex(m_mesh);
	MItMeshEdge itEdge(m_mesh);
	for (itVertex.reset(); !itVertex.isDone(); itVertex.next()) {
		MIntArray
			conEdges,
			conFaces,
			tmpFaces;

		itVertex.getConnectedEdges(conEdges);
		itVertex.getConnectedFaces(tmpFaces);

		int
			numEdges = conEdges.length(),
			numFaces = tmpFaces.length();

		for (int i = numFaces - 1; i >= 0; i--)
			conFaces.append(tmpFaces[i]);

		int
			startEdge = -1,
			numSplits = 0;

		for (int e = 0; e < numEdges; e++) {
			if (hardEdges.find(conEdges[e]) != hardEdges.end()) {
				int prevId;
				itEdge.setIndex(conEdges[e], prevId);
				
				MIntArray edgeConFaces;
				itEdge.getConnectedFaces(edgeConFaces);

				if (2 != edgeConFaces.length())
					continue;

				MVector vA, vB;
				fnMesh.getPolygonNormal(edgeConFaces[0], vA);
				fnMesh.getPolygonNormal(edgeConFaces[1], vB);

				double angle = M_PI - acos(vA*vB);

				if (tresholdAngle < angle)
					continue;

				if (startEdge < 0)
					startEdge = e;
				numSplits++;
			}
		}
		if (itVertex.onBoundary())
			startEdge = 0;

		if ((numSplits == 1 && !itVertex.onBoundary()) || numSplits == 0)
			continue;

		MVector average;
		MIntArray faces;
		
		for (int f = 0; f < numFaces; f++) {
			int relativeIdx = (startEdge + f) % numFaces;

			if (f!=0 && hardEdges.find(conEdges[relativeIdx]) != hardEdges.end()) {
				average.normal();
				for (unsigned int i = 0; i < faces.length(); i++)
					fnMesh.setFaceVertexNormal(average, faces[i], itVertex.index());

				average = MVector::zero;
				faces.clear();
			}

			MVector polyNormal;
			fnMesh.getPolygonNormal(conFaces[relativeIdx], polyNormal);
			average += polyNormal;
			faces.append(conFaces[relativeIdx]);
		}
	}

	return MS::kSuccess;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods ////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

MStatus SSeamMesh::offsetEdgeloop(SEdgeLoop &edgeLoop, float offsetDistance, bool createPolygons) {
	MStatus status;

	MFnMesh fnMesh(m_mesh);
	int numEdges = fnMesh.numEdges();
	int numVertices = fnMesh.numVertices();
	int numFaces = fnMesh.numPolygons();

	MPointArray
		meshPoints;
	MIntArray
		polyCounts,
		polyIndices;
	MVectorArray
		meshNormals;

	fnMesh.getPoints(meshPoints);
	fnMesh.getVertices(polyCounts, polyIndices);
	getNormals(meshNormals);

	// Load UV sets
	SUVSet currentSet(fnMesh.currentUVSetName());
	status = fnMesh.getUVs(currentSet.U, currentSet.V);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = fnMesh.getAssignedUVs(currentSet.uvCounts, currentSet.uvIndices);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MItMeshVertex itVertex(m_mesh);

	bool isClosedEnd = false;
	bool isCrossedEnd = false;
	int firstVtxId = -1;
	int lastEdge;
	unsigned int lastElement = edgeLoop.numEdges() - 1;
	MPoint tmpFirstPoint;

	for (unsigned int l = 0; l < edgeLoop.numEdges(); l++) {

		int vertices[2];
		fnMesh.getEdgeVertices(edgeLoop[l], vertices);

		bool isLast = false;

		// Duplicate and move original point	
		for (int v = (l == 0) ? 0 : 1; v < 2 ; v++) {
			bool isFirst = (l == 0 && v == 0);
			isLast = (l == lastElement && v==1);

			int vtxId = vertices[v], previousVtx;
			status = itVertex.setIndex(vtxId, previousVtx);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			if (!itVertex.onBoundary())
				return MS::kFailure;

			MIntArray conEdges;
			itVertex.getConnectedEdges(conEdges);

			if (isFirst && 1<edgeLoop.numEdges() && itVertex.connectedToEdge(edgeLoop[lastElement])) {
				if (conEdges.length() == 2)
					isCrossedEnd = true;
				else
					isClosedEnd = true;
			}

			if (isLast && isClosedEnd)
				break;

			// Set face vertex normals
			if (createPolygons) {
				MPoint newPoint = meshPoints[vtxId];

				if (isCrossedEnd && isFirst) {
					tmpFirstPoint = newPoint;
					firstVtxId = vtxId;
					MVector direction = getEdgeVector(conEdges[0], vtxId).normal();
					newPoint += direction*offsetDistance;
				}

				meshPoints.append(newPoint);
				meshNormals.append(meshNormals[vtxId]);
			}

			MVector direction;
			if (isFirst && !isClosedEnd)
				direction = getEdgeVector(conEdges[conEdges.length() - 1], vtxId).normal();
			else if (isLast && !isClosedEnd)
				direction = getEdgeVector(conEdges[0], vtxId).normal();
			else
				for (unsigned int e = 1; e < conEdges.length() - 1; e++)
					direction += getEdgeVector(conEdges[e], vtxId).normal();
			meshPoints[vtxId] += direction.normal()*offsetDistance;

			if (createPolygons && !isClosedEnd && !isCrossedEnd) {
				for (auto &conLoop : m_activeLoops) {
					if (isFirst) {
						if (conLoop.contains(conEdges[conEdges.length() - 1]))
							conLoop.pushBack(numEdges);
					}
					if (isLast) {
						if (conLoop.contains(conEdges[0]))
							conLoop.pushFront(numEdges + 2 * edgeLoop.numEdges());
					}
				}
			}

			
		}

		if (createPolygons) {
			// Define new polygon
			polyIndices.append(vertices[0]);
			polyIndices.append(numVertices + l);
			polyIndices.append((isLast && isClosedEnd) ? numVertices : numVertices + l + 1);
			polyIndices.append(vertices[1]);
			polyCounts.append(4);
			currentSet.addPolygon();

			// Udate edge loop id
			edgeLoop[l] = lastEdge = 2 * l + 1 + numEdges;
		}
	}


	if (isCrossedEnd && createPolygons) {
		polyIndices.append(meshPoints.length() - 1);
		polyIndices.append(meshPoints.length());
		polyIndices.append(numVertices);
		polyIndices.append(firstVtxId);

		polyCounts.append(4);
		currentSet.addPolygon();

		meshPoints.append(tmpFirstPoint);
		meshNormals.append(meshNormals[firstVtxId]);

		edgeLoop.pushFront(lastEdge + 3);
		edgeLoop.pushBack(lastEdge + 2);
	}

	if (createPolygons) {
		// Update mesh
		fnMesh.create(
			meshPoints.length(),
			polyCounts.length(),
			meshPoints,
			polyCounts,
			polyIndices,
			currentSet.U,
			currentSet.V,
			m_mesh,
			&status
		);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = fnMesh.assignUVs(currentSet.uvCounts, currentSet.uvIndices);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = setNormals(meshNormals);
		CHECK_MSTATUS_AND_RETURN_IT(status)
	}
	else
		fnMesh.setPoints(meshPoints);

	return MS::kSuccess;
}