#include "SMesh.h"

SMesh::SMesh(){};

SMesh::~SMesh(){}

SMesh::SMesh(MObject &obj, MStatus *ref){
	if (obj.apiType() != MFn::kMeshData && obj.apiType() != MFn::kMesh)
		*ref = MS::kInvalidParameter;
	
	m_mesh = obj;

	MFnMesh fnMesh(m_mesh);
	for (int v = 0; v < fnMesh.numVertices(); v++)
		m_vtxMap[v] = v;
};

SMesh::SMesh(const SMesh &mesh){
	MFnMeshData meshData;
	m_mesh = meshData.create();

	MFnMesh fnMesh(mesh.m_mesh);
	fnMesh.copy(mesh.m_mesh, m_mesh);

	m_normals = mesh.m_normals;
	m_vtxMap = mesh.m_vtxMap;
	m_vtxSplitValence = mesh.m_vtxSplitValence;
	m_activeLoops = mesh.m_activeLoops;

	updateMeshPointers();
};

SMesh& SMesh::operator=(const SMesh& mesh) {
	m_mesh = mesh.m_mesh;
	m_normals = mesh.m_normals;
	m_vtxMap = mesh.m_vtxMap;
	m_vtxSplitValence = mesh.m_vtxSplitValence;
	m_activeLoops = mesh.m_activeLoops;

	updateMeshPointers();

	return *this;
}

void SMesh::updateMeshPointers() {
	for (auto &loop : m_activeLoops)
		loop.setMeshPtr(&m_mesh);
}

MObject SMesh::getObject() const {
	return m_mesh;
}

bool SMesh::isEquivalent(const MObject& firstMesh, const MObject& secondMesh, MStatus *ref) {
	if (firstMesh.apiType() != MFn::kMeshData || secondMesh.apiType() != MFn::kMeshData) {
		*ref = MS::kInvalidParameter;
		return false;
	}

	MFnMesh
		fnFirst(firstMesh),
		fnSecond(secondMesh);

	if (fnFirst.numVertices() != fnSecond.numVertices() ||
		fnFirst.numEdges() != fnSecond.numEdges() ||
		fnFirst.numPolygons() != fnSecond.numPolygons())
		return false;

	MIntArray
		polyCounts,
		indicesFirst,
		indicesSecond;

	fnFirst.getVertices(polyCounts, indicesFirst);
	fnSecond.getVertices(polyCounts, indicesSecond);

	if (indicesFirst.length() != indicesSecond.length())
		return false;

	for (unsigned int i = 0; i < indicesFirst.length(); i++)
		if (indicesFirst[i] != indicesSecond[i])
			return false;
	
	return true;
}

MStatus SMesh::smoothMesh(MMeshSmoothOptions &smoothOptions) {
	MStatus status;
	
	unsigned int multiplier = (unsigned int)pow(2, smoothOptions.divisions());
	if (multiplier < 2)
		return MS::kSuccess;

	MFnMeshData meshData;
	MObject smoothMesh = meshData.create();

	MFnMesh fnMesh(m_mesh);
	fnMesh.generateSmoothMesh(smoothMesh, &smoothOptions, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	m_mesh = smoothMesh;

	for (auto &loop : m_activeLoops){
		SEdgeLoop smoothLoop(&m_mesh);
		for (unsigned int e = 0; e < loop.numEdges(); e++)
			for (unsigned int m = 0; m < multiplier; m++) {
				int newM = (loop.isFlipped(e)) ? (multiplier - 1 - m) : m;
				smoothLoop.pushBack(loop[e] * multiplier + newM, loop.isFlipped(e));
			}
		loop = smoothLoop;
	}

	return MS::kSuccess;
}

MStatus SMesh::combine(const MObjectArray& meshes, MObject& combinedMesh) {
	MStatus status;

	if (0 == meshes.length() || combinedMesh.apiType()!=MFn::kMeshData)
		return MS::kInvalidParameter;

	MPointArray
		combinedVertices;
	MIntArray
		combinedCounts,
		combinedIndices;

	for (unsigned int m = 0; m < meshes.length(); m++) {
		if (meshes[m].apiType() != MFn::kMeshData)
			return MS::kInvalidParameter;

		unsigned int vtxCount = combinedVertices.length();

		MPointArray
			meshPoints;
		MIntArray
			polyCounts,
			polyIndices;

		MFnMesh fnMesh(meshes[m], &status);
		fnMesh.getPoints(meshPoints);
		fnMesh.getVertices(polyCounts, polyIndices);

		for (unsigned int p = 0; p < meshPoints.length(); p++)
			combinedVertices.append(meshPoints[p]);

		for (unsigned int c = 0; c < polyCounts.length(); c++)
			combinedCounts.append(polyCounts[c]);

		for (unsigned int i = 0; i < polyIndices.length(); i++)
			combinedIndices.append(polyIndices[i] + vtxCount);
	}

	MFnMesh fnMesh;
	fnMesh.create(
		combinedVertices.length(),
		combinedCounts.length(),
		combinedVertices,
		combinedCounts,
		combinedIndices,
		combinedMesh,
		&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

MStatus SMesh::detachEdges(const MIntArray &edges) {
	MStatus status;

	MFnMesh fnMesh(m_mesh);

	MPointArray
		meshPoints;
	MFloatVectorArray
		meshNormals;
	MIntArray
		polyCounts,
		polyIndices;

	fnMesh.getPoints(meshPoints);
	fnMesh.getVertexNormals(true, meshNormals);
	fnMesh.getVertices(polyCounts, polyIndices);

	// Feed polygon info in more convenient form
	std::vector <SMeshPolygon> meshPolygons;
	unsigned int currentId = 0;
	for (unsigned int f = 0; f < polyCounts.length(); f++) {
		unsigned int currentMax = currentId + polyCounts[f];
		SMeshPolygon polygon;
		for (currentId; currentId < currentMax; currentId++)
			polygon.append(polyIndices[currentId]);
		meshPolygons.push_back(polygon);
	}

	// Put edges in set for faster search
	std::set <unsigned int> detachedEdges;
	for (unsigned int e = 0; e < edges.length(); e++)
		detachedEdges.insert(edges[e]);

	// Now we're going to generate new ids for vertices on detached edges
	MItMeshVertex itVertex(m_mesh);
	for (itVertex.reset(); !itVertex.isDone(); itVertex.next()) {
		MIntArray
			conEdges,
			conFaces,
			tmpFaces;
		int
			numEdges,
			numFaces;

		itVertex.getConnectedEdges(conEdges);
		itVertex.getConnectedFaces(tmpFaces);
		itVertex.numConnectedEdges(numEdges);
		itVertex.numConnectedFaces(numFaces);

		// Since faces are ordered CW and edge CCW we reverse the faces
		for (int i = numFaces - 1; i >= 0; i--)
			conFaces.append(tmpFaces[i]);
		
		// Find the first split edge connected to current vertex. And total splits
		int
			startEdge = -1,
			numSplits = 0;

		for (int e = 0; e < numEdges; e++) {
			if (detachedEdges.find(conEdges[e]) != detachedEdges.end()) {
				if (startEdge < 0)
					startEdge = e;
				numSplits++;
			}
		}
		if (itVertex.onBoundary())
			startEdge = 0;
		
		// Skip interior vertices with only one split and vertices without split
		if ((numSplits==1 && !itVertex.onBoundary()) || numSplits == 0)
			continue;

		// Iterating over faces, replacing old vertices with new. Every time we cross a split edge we create new vertex
		unsigned int newVtxId = itVertex.index();
		for (int f = 0; f < numFaces; f++) {
			int relativeIdx = (startEdge + f) % numFaces;
			if (f != 0 && detachedEdges.find(conEdges[relativeIdx]) != detachedEdges.end()) {
				newVtxId = meshPoints.length();
				meshPoints.append(meshPoints[itVertex.index()]);
				meshNormals.append(meshNormals[itVertex.index()]);
				m_vtxMap[newVtxId] = itVertex.index();
				m_vtxSplitValence[newVtxId] = numSplits;
			}
			if (newVtxId != itVertex.index())
				meshPolygons[conFaces[relativeIdx]].replace(itVertex.index(), newVtxId);
			m_vtxSplitValence[newVtxId] = numSplits;
		}
	}

	// Update polyindices
	polyIndices.clear();
	for (auto &polygon : meshPolygons)
		for (unsigned int p = 0; p < polygon.length(); p++)
			polyIndices.append(polygon[p]);
	
	// Load UV sets
	SUVSet
		currentSet(fnMesh.currentUVSetName());

	status = fnMesh.getUVs(currentSet.U, currentSet.V);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = fnMesh.getAssignedUVs(currentSet.uvCounts, currentSet.uvIndices);
	CHECK_MSTATUS_AND_RETURN_IT(status);

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

	// Get shared normals used by this class for parallel flanges etc.
	MVectorArray newNormals;
	for (unsigned int i = 0; i<meshNormals.length(); i++)
		newNormals.append(meshNormals[i]);

	setNormals(newNormals);

	return MS::kSuccess;
}


MVector SMesh::getEdgeVector(int edge, int fromVertex) {
	
	MFnMesh fnMesh(m_mesh);
	int vertices[2];
	fnMesh.getEdgeVertices(edge, vertices);

	MPoint A, B;
	fnMesh.getPoint(vertices[0], A);
	fnMesh.getPoint(vertices[1], B);
	
	return (fromVertex == vertices[0]) ? B - A : A - B;
}

MStatus SMesh::setNormals(const MVectorArray& normals) {
	MFnMesh fnMesh(m_mesh);
	
	if (normals.length() != fnMesh.numVertices())
		return MS::kInvalidParameter;

	m_normals = normals;
	return MS::kSuccess;
}

void SMesh::getNormals(MVectorArray& normals) {
	normals = m_normals;
}

MStatus SMesh::updateMesh(const MObject& sourceMesh) {
	MStatus status;

	if (sourceMesh.apiType() != MFn::kMeshData)
		return MS::kInvalidParameter;

	MFnMesh fnSrcMesh(sourceMesh);
	MFnMesh fnTrgMesh(m_mesh);

	MPointArray
		srcPoints,
		trgPoints;
	MFloatVectorArray
		srcNormals;
	MVectorArray
		trgNormals;

	fnSrcMesh.getPoints(srcPoints);
	fnSrcMesh.getVertexNormals(true, srcNormals);

	for (int v = 0; v < fnTrgMesh.numVertices(); v++) {
		if (srcPoints.length() <= m_vtxMap[v])
			return MS::kFailure;
		int srcVtxId = m_vtxMap[v];
		trgPoints.append(srcPoints[srcVtxId]);
		trgNormals.append(srcNormals[srcVtxId]);
	}

	fnTrgMesh.setPoints(trgPoints);
	setNormals(trgNormals);

	return MS::kSuccess;
}

MStatus SMesh::getEdgeVertices(const MIntArray& edges, MIntArray& vertices) {
	MStatus status;
	
	vertices.clear();

	std::set <unsigned int> vtxId;
	
	MFnMesh fnMesh(m_mesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	for (unsigned int i = 0; i < edges.length(); i++){
		int vertices[2];
		status = fnMesh.getEdgeVertices(edges[i], vertices);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		vtxId.insert(std::begin(vertices), std::end(vertices));
	}

	for (auto &vtx : vtxId)
		vertices.append(vtx);

	return MS::kSuccess;
}

MStatus SMesh::pullVertices(const MIntArray& vertices, const float distance) {
	MStatus status;

	MFnMesh fnMesh(m_mesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MPointArray
		meshPoints;
	MVectorArray
		meshNormals;

	std::set <unsigned int> done;

	fnMesh.getPoints(meshPoints);
	getNormals(meshNormals);

	for (unsigned int i = 0; i < vertices.length(); i++)
		if ((int)meshPoints.length() <= vertices[i])
			return MS::kInvalidParameter;
		else
			if (done.find(vertices[i]) == done.end()) {
				meshPoints[vertices[i]] += meshNormals[vertices[i]] * distance;
				done.insert(vertices[i]);
			}

	fnMesh.setPoints(meshPoints);

	return MS::kSuccess;
}

MStatus SMesh::getBoundaryEdges(MIntArray &edges) {
	MStatus status;

	edges.clear();

	MItMeshEdge itEdge(m_mesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	for (itEdge.reset(); !itEdge.isDone(); itEdge.next())
		if (itEdge.onBoundary())
			edges.append(itEdge.index());

	return MS::kSuccess;
}


MStatus SMesh::extrudeEdges(const MIntArray& edges, const float thickness, const unsigned int divisions) {
	MStatus status;

	unsigned int numSegments = divisions + 1;
	float divisionThickness = thickness / numSegments;

	MPointArray
		meshPoints;
	MIntArray
		polyCounts,
		polyIndices;
	MVectorArray
		meshNormals;

	MFnMesh fnMesh(m_mesh);
	fnMesh.getPoints(meshPoints);
	fnMesh.getVertices(polyCounts, polyIndices);
	getNormals(meshNormals);

	// Load UV sets
	SUVSet currentSet(fnMesh.currentUVSetName());
	status = fnMesh.getUVs(currentSet.U, currentSet.V);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = fnMesh.getAssignedUVs(currentSet.uvCounts, currentSet.uvIndices);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// <division level <original id, extruded id>
	std::map <unsigned int, std::map <unsigned int, unsigned int>> mapIds;

	for (unsigned int e = 0; e < edges.length(); e++) {
		int vertices[2];
		fnMesh.getEdgeVertices(edges[e], vertices);

		for (unsigned int d = 0; d < numSegments; d++) {
			// Start polygon with original vertices
			for (unsigned int i = 0; i < 2; i++) {
				int vtxId = vertices[(i == 0) ? 1 : 0];
				int divVtxId = (d == 0) ? vtxId : mapIds[d - 1][vtxId];
				polyIndices.append(divVtxId);
			}

			// Complete polygon with extruded vertices
			for (unsigned int i = 0; i < 2; i++) {
				int vtxId = vertices[i];

				// Define new vertex if necessary
				if (mapIds[d].find(vtxId) == mapIds[d].end()) {
					float extDistance = (d + 1)*divisionThickness*(-1);
					MPoint extrudedPoint = meshPoints[vtxId] + extDistance*meshNormals[vtxId];
					mapIds[d][vtxId] = meshPoints.length();
					meshPoints.append(extrudedPoint);
				}
				polyIndices.append(mapIds[d][vtxId]);
			}
			polyCounts.append(4);
			currentSet.addPolygon();
		}
	}

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

	return MS::kSuccess;
}

MStatus SMesh::setActiveEdges(const MIntArray& edges) {
	MStatus status;

	m_activeLoops.clear();
	
	std::set <unsigned int> remainingEdges;
	for (unsigned int e = 0; e < edges.length(); e++)
		remainingEdges.insert(edges[e]);

	// Check if mesh contains listed edges
	MFnMesh fnMesh(m_mesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	unsigned int numEdges = fnMesh.numEdges();
	if(*remainingEdges.rbegin() >= numEdges)
		return MS::kInvalidParameter;

	MItMeshVertex itVertex(m_mesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	for (itVertex.reset(); !itVertex.isDone(); itVertex.next()) {
		int
			prevId,
			vertId = itVertex.index();

		MIntArray conEdges;
		itVertex.getConnectedEdges(conEdges);
		unsigned int numEdges = conEdges.length();
		
		for (unsigned int e = 0; e < numEdges; e++) {
			int edgeId = conEdges[e];
			if (remainingEdges.find(edgeId) == remainingEdges.end())
				continue;

			SEdgeLoop edgeLoop(&m_mesh);
			itVertex.setIndex(vertId, prevId);
			status = contiguousEdges(itVertex, remainingEdges, edgeLoop, edgeId);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			// Explore edge in opposite direction
			if ((!itVertex.onBoundary() && numEdges == 4 && e<2) || (itVertex.onBoundary() && 2<numEdges && e == 0)) {
				int relOutIdx = (itVertex.onBoundary()) ? numEdges - 1 : e + 2;
				int outIdx = conEdges[relOutIdx];
				if (remainingEdges.find(outIdx) != remainingEdges.end()) {	
					itVertex.setIndex(vertId, prevId);
					status = contiguousEdges(itVertex, remainingEdges, edgeLoop, outIdx);
					CHECK_MSTATUS_AND_RETURN_IT(status);
				}
			}

			if (0 < edgeLoop.numEdges())
				m_activeLoops.push_back(edgeLoop);

			if (remainingEdges.size() == 0)
				break;
		}

		if (remainingEdges.size() == 0)
			break;

		itVertex.setIndex(vertId, prevId);
	}

	return MS::kSuccess;
}

MStatus SMesh::setActiveEdges(const MObject& edgeComponent) {
	MStatus status;

	if (edgeComponent.apiType() != MFn::kMeshEdgeComponent)
		return MS::kInvalidParameter;

	MFnSingleIndexedComponent fnComponent(edgeComponent, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	MIntArray edges;
	status = fnComponent.getElements(edges);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = setActiveEdges(edges);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

void SMesh::getActiveEdges(MIntArray& edges) {
	edges.clear();
	for (auto &loop : m_activeLoops)
		for (unsigned int i = 0; i < loop.numEdges(); i++)
			edges.append(loop[i]);
}

void SMesh::getActiveLoops(std::vector <SEdgeLoop> &activeLoops) {
	activeLoops = m_activeLoops;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Component grouping  ////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

MStatus	SMesh::groupConnectedComponents(const MObject &component, MObjectArray& componentGroups) {
	MStatus status;

	componentGroups.clear();

	// Get indices from the component
	MFnSingleIndexedComponent fnComponent(component, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MIntArray compIndices;
	fnComponent.getElements(compIndices);

	std::set <unsigned int> compSet;
	for (unsigned int i = 0; i < compIndices.length(); i++)
		compSet.insert(compIndices[i]);

	// Group faces
	if (component.apiType() == MFn::kMeshPolygonComponent) {
		MItMeshPolygon itPolygon(m_mesh);
		while (compSet.size() != 0) {
			int currentId = *compSet.begin(), previousId;
			compSet.erase(currentId);
			MIntArray groupIndices;
			groupIndices.append(currentId);

			status = itPolygon.setIndex(currentId, previousId);
			CHECK_MSTATUS_AND_RETURN_IT(status);
	
			status = groupConnectedFaces(itPolygon, compSet, groupIndices);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			MFnSingleIndexedComponent fnGroupComponent;
			MObject groupComponent = fnGroupComponent.create(MFn::kMeshPolygonComponent, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			fnGroupComponent.addElements(groupIndices);
			componentGroups.append(groupComponent);
		}
	}
	// Group vertices
	else if (component.apiType() == MFn::kMeshVertComponent) {
		MItMeshVertex itVertex(m_mesh);
		while (compSet.size() != 0) {
			int currentId = *compSet.begin(), previousId;
			compSet.erase(currentId);
			MIntArray groupIndices;
			groupIndices.append(currentId);

			status = itVertex.setIndex(currentId, previousId);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			status = groupConnectedVertices(itVertex, compSet, groupIndices);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			MFnSingleIndexedComponent fnGroupComponent;
			MObject groupComponent = fnGroupComponent.create(MFn::kMeshVertComponent, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			fnGroupComponent.addElements(groupIndices);
			componentGroups.append(groupComponent);
		}
	}
	// Group edges
	else if (component.apiType() == MFn::kMeshEdgeComponent)
	{
		status = setActiveEdges(compIndices);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		std::vector<SEdgeLoop> loops;
		getActiveLoops(loops);

		for (auto &loop : loops) {
			MIntArray groupIndices;
			loop.get(groupIndices);

			MFnSingleIndexedComponent fnGroupComponent;
			MObject groupComponent = fnGroupComponent.create(MFn::kMeshEdgeComponent, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			fnGroupComponent.addElements(groupIndices);
			componentGroups.append(groupComponent);
		}
	}
	else
		return MS::kInvalidParameter;

	return MS::kSuccess;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Protected methods //////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

MStatus SMesh::groupConnectedFaces(MItMeshPolygon &itPolygon, std::set <unsigned int> &compSet, MIntArray &indices) {
	MStatus status;

	MIntArray connectedFaces;
	itPolygon.getConnectedFaces(connectedFaces);

	for (unsigned int f = 0; f < connectedFaces.length(); f++) {
		int currentId = connectedFaces[f], previousId;

		if (compSet.find(currentId) != compSet.end()) {
			compSet.erase(currentId);
			indices.append(currentId);

			status = itPolygon.setIndex(currentId, previousId);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			status = groupConnectedFaces(itPolygon, compSet, indices);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
	}

	return MS::kSuccess;
}

MStatus SMesh::groupConnectedVertices(MItMeshVertex &itVertex, std::set <unsigned int> &compSet, MIntArray &indices) {
	MStatus status;

	MIntArray connectedVertices;
	itVertex.getConnectedVertices(connectedVertices);

	for (unsigned int v = 0; v < connectedVertices.length(); v++) {
		int currentId = connectedVertices[v], previousId;

		if (compSet.find(currentId) != compSet.end()) {
			compSet.erase(currentId);
			indices.append(currentId);

			status = itVertex.setIndex(currentId, previousId);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			status = groupConnectedVertices(itVertex, compSet, indices);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}
	}

	return MS::kSuccess;
}

MStatus SMesh::contiguousEdges(MItMeshVertex &itVertex, std::set <unsigned int> &remainingEdges, SEdgeLoop &loop, const int edge) {
	MStatus status;

	status = loop.add(edge);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	remainingEdges.erase(edge);

	int nextVtx, currentIdx;
	status = itVertex.getOppositeVertex(nextVtx, edge);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = itVertex.setIndex(nextVtx, currentIdx);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	status = extendEdgeLoop(itVertex, remainingEdges, loop);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}

MStatus SMesh::extendEdgeLoop(MItMeshVertex &itVertex, std::set <unsigned int> &remainingEdges, SEdgeLoop &activeLoop) {
	MStatus status;

	if (0 == activeLoop.numEdges())
		return MS::kInvalidParameter;

	MIntArray conEdges;
	itVertex.getConnectedEdges(conEdges);
	unsigned int numEdges = conEdges.length();

	// End loop once you hit star-shaped vertex
	if ((itVertex.onBoundary() && numEdges == 2) ||
		(!itVertex.onBoundary() && numEdges != 4))
		return MS::kSuccess;

	// Find vertex relative index of incoming edge
	int relInIdx = -1;
	for (unsigned int e=0; e < numEdges; e++)
		if (conEdges[e] == activeLoop[0] ||
			conEdges[e] == activeLoop[activeLoop.numEdges() - 1]) {
			relInIdx = e;
			break;
		}
	if (relInIdx < 0)
		return MS::kFailure;

	// End loop once boundary is reached
	if (itVertex.onBoundary() && (relInIdx != 0 && relInIdx != numEdges - 1))
		return MS::kSuccess;

	// Find index of outcoming edge
	int relOutIdx;
	if (itVertex.onBoundary())
		relOutIdx = (relInIdx==0) ? numEdges - 1 : 0;
	else
		relOutIdx = (relInIdx + 2) % numEdges;
	int outIdx = conEdges[relOutIdx];

	if (remainingEdges.find(outIdx) != remainingEdges.end())
		contiguousEdges(itVertex, remainingEdges, activeLoop, outIdx);

	return MS::kSuccess;
}