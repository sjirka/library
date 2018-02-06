#include "SEdgeLoop.h"

#include <set>

	SEdgeLoop::SEdgeLoop(MObject *meshPtr) {
		m_meshPtr = meshPtr;
	}

	SEdgeLoop::SEdgeLoop(const SEdgeLoop &edgeLoop) {
		m_meshPtr = edgeLoop.m_meshPtr;
		m_ordered = edgeLoop.m_ordered;
		m_flipped = edgeLoop.m_flipped;
	};

	SEdgeLoop& SEdgeLoop::operator=(const SEdgeLoop& edgeLoop) {
		m_meshPtr = edgeLoop.m_meshPtr;
		m_ordered = edgeLoop.m_ordered;
		m_flipped = edgeLoop.m_flipped;

		return *this;
	}

	SEdgeLoop::~SEdgeLoop() {
	}

	void SEdgeLoop::setMeshPtr(MObject *meshPtr) {
		m_meshPtr = meshPtr;
	}

	MStatus SEdgeLoop::add(const unsigned int edge) {
		MStatus status;

		if (m_meshPtr == NULL)
			return MS::kFailure;

		if (m_ordered.size() == 0) {
			pushBack(edge);
			return MS::kSuccess;
		}

		MFnMesh fnMesh(*m_meshPtr, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		int verticesNew[2];
		status = fnMesh.getEdgeVertices(edge, verticesNew);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		
		int verticesFirst[2];
		bool flipped;
		status = getEdgeVertices(0, verticesFirst, flipped);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		if (verticesFirst[0] == verticesNew[1])
			pushFront(edge);
		else if(verticesFirst[0] == verticesNew[0])
			pushFront(edge, true);
		else{
			int verticesLast[2];
			status = getEdgeVertices(numEdges() - 1, verticesLast, flipped);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			if (verticesLast[1] == verticesNew[0])
				pushBack(edge);
			else if (verticesLast[1] == verticesNew[1])
				pushBack(edge, true);
			else
				pushBack(edge);
		}

		return MS::kSuccess;
	}

	MStatus SEdgeLoop::getLength(double &loopLength) {
		MStatus status;
		
		MItMeshEdge itEdge(*m_meshPtr, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		loopLength = 0;
		for (auto &edge : m_ordered) {
			int previousVtx;
			status = itEdge.setIndex(edge, previousVtx);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			double edgeLength;
			status = itEdge.getLength(edgeLength);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			loopLength += edgeLength;
		}

		return MS::kSuccess;
	}

	bool SEdgeLoop::pushFront(const unsigned int edge, const bool flip) {
		if (contains(edge))
			return false;

		m_ordered.push_front(edge);
		m_flipped.push_front(flip);
		return true;
	}

	bool SEdgeLoop::pushBack(const unsigned int edge, const bool flip) {
		if (contains(edge))
			return false;

		m_ordered.push_back(edge);
		m_flipped.push_back(flip);
		return true;
	}

	bool SEdgeLoop::contains(const unsigned int edge) const {
		if (std::find(m_ordered.begin(), m_ordered.end(), edge) == m_ordered.end())
			return false;
		return true;
	}

	unsigned int SEdgeLoop::numEdges() const {
		return (int)m_ordered.size();
	}

	unsigned int &SEdgeLoop::operator[](const unsigned int index) {
		return m_ordered[index];
	}

	void SEdgeLoop::get(MIntArray &edges) {
		for (auto &edgeId : m_ordered)
			edges.append(edgeId);
	}

	void SEdgeLoop::print() {
		cout << "edge start ---------------" << endl;
		for (auto &edgeId : m_ordered)
			cout << edgeId << endl;
	}

	void SEdgeLoop::flip(int2 &vertices) {
		int tmp = vertices[0];
		vertices[0] = vertices[1];
		vertices[1] = tmp;
	}

	MStatus SEdgeLoop::getEdgeVertices(const unsigned int index, int2 &vertices, bool &flipped) {
		MStatus status;

		if (m_meshPtr == NULL)
			return MS::kFailure;

		MFnMesh fnMesh(*m_meshPtr, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		if (index > (numEdges() - 1))
			return MS::kInvalidParameter;
		if (m_ordered[index] > (unsigned int)(fnMesh.numEdges() - 1))
			return MS::kInvalidParameter;

		status = fnMesh.getEdgeVertices(m_ordered[index], vertices);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		flipped = m_flipped[index];
		if (flipped)
			flip(vertices);

		return MS::kSuccess;
	}

	MStatus SEdgeLoop::getVertices(MIntArray& vertices) {
		MStatus status;

		vertices.clear();

		for (unsigned int i = 0; i < numEdges(); i++) {
			int2 edgeVertices; bool flipped;
			status = getEdgeVertices(i, edgeVertices, flipped);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			if(i==0)
				vertices.append(edgeVertices[0]);
			vertices.append(edgeVertices[1]);
		}

		return MS::kSuccess;
	}

	MStatus SEdgeLoop::getPoints(MPointArray& points, MSpace::Space space) {
		MStatus status;

		points.clear();

		MIntArray vertices;
		status = getVertices(vertices);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MFnMesh fnMesh(*m_meshPtr, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MPointArray meshPoints;
		status = fnMesh.getPoints(meshPoints, space);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		for (unsigned int i = 0; i < vertices.length(); i++)
			points.append(meshPoints[vertices[i]]);

		return MS::kSuccess;
	}

	bool SEdgeLoop::isFlipped(const unsigned int index) {
		return m_flipped[index];
	}

	bool SEdgeLoop::isClosed() {
		int2 vertices;
		endVertices(vertices);

		if (vertices[0] == vertices[1])
			return true;
		return false;
	}

	void SEdgeLoop::endVertices(int2 &vertices) {
		int2 first, last;
		bool flFirst, flLast;
		getEdgeVertices(0, first, flFirst);
		getEdgeVertices(numEdges() - 1, last, flLast);

		vertices[0] = first[0];
		vertices[1] = last[1];
	}