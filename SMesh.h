#pragma once

#include <maya\MObject.h>
#include <maya\MStatus.h>
#include <maya\MFnMesh.h>
#include <maya\MPointArray.h>
#include <maya\MFloatVectorArray.h>
#include <maya\MFloatPointArray.h>
#include <maya\MVectorArray.h>
#include <maya\MIntArray.h>
#include <maya\MItMeshPolygon.h>
#include <maya\MItMeshEdge.h>
#include <maya\MItMeshVertex.h>
#include <maya\MFnMeshData.h>
#include <maya\MGlobal.h>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include "SEdgeLoop.h"

class SMeshArray;

class SMesh
{
public:
	SMesh();
	SMesh(MObject &obj, MStatus *ref=NULL);
	SMesh(const SMesh &mesh);
	SMesh& operator=(const SMesh& mesh);
	~SMesh();

	MObject			getObject() const;
	static bool		isEquivalent(const MObject& firstMesh, const MObject& secondMesh, MStatus *ref = NULL);
	MStatus			updateMesh(const MObject& sourceMesh);
	MStatus			smoothMesh(MMeshSmoothOptions &smoothOptions);

	MStatus			detachEdges(const MIntArray &edges);
	static MStatus	combine(const MObjectArray& meshes, MObject& combinedMesh);

	MStatus			setNormals(const MVectorArray& normals);
	void			getNormals(MVectorArray& normals);

	MStatus			extrudeEdges(const MIntArray& edges, const float thickness, const unsigned int divisions);
	MStatus			pullVertices(const MIntArray& vertices, const float distance);

	virtual MStatus setActiveEdges(const MIntArray& edges);
	void			getActiveEdges(MIntArray& edges);
	void			getActiveLoops(std::vector <SEdgeLoop> &activeLoops);
	MStatus			getBoundaryEdges(MIntArray &edges);
	void			updateMeshPointers();
	
	MStatus			getEdgeVertices(const MIntArray& edges, MIntArray& vertices);
	MVector			getEdgeVector(int edge, int fromVertex = -1);

protected:
	MObject m_mesh;
	MVectorArray m_normals;

	std::map <unsigned int, unsigned int> m_vtxMap;
	std::map <unsigned int, unsigned int> m_vtxSplitValence;
	std::vector <SEdgeLoop> m_activeLoops;

	virtual MStatus extendEdgeLoop(MItMeshVertex &itVertex, std::set <unsigned int> &remainingEdges, SEdgeLoop &activeLoop);
	virtual MStatus contiguousEdges(MItMeshVertex &itVertex, std::set <unsigned int> &remainingEdges, SEdgeLoop &loop, const int edge);
};

class SMeshPolygon
{
public:
	SMeshPolygon() {};
	~SMeshPolygon() {};

	void append(unsigned int vertex) {
		m_vtxId.push_back(vertex);
	}

	bool contains(unsigned int vertex) {
		if (std::find(m_vtxId.begin(), m_vtxId.end(), vertex) == m_vtxId.end())
			return false;
		return true;
	}

	bool replace(unsigned int vertex, unsigned int newVertex) {
		auto it = std::find(m_vtxId.begin(), m_vtxId.end(), vertex);
		if (it == m_vtxId.end())
			return false;
		*it = newVertex;
		return true;
	}

	unsigned int length() const {
		return (int)m_vtxId.size();
	}

	unsigned int &operator[](const unsigned int index) {
		return m_vtxId[index];
	}

private:
	std::vector <unsigned int> m_vtxId;
};

class SUVSet {
public:
	SUVSet(){};

	SUVSet(MString &name) {
		this->name = name;
	};

	SUVSet(const SUVSet &uvSet) {
		name = uvSet.name;
		U = uvSet.U;
		V = uvSet.U;
		uvCounts = uvSet.uvCounts;
		uvIndices = uvSet.uvIndices;
	};

	SUVSet& operator=(const SUVSet& uvSet) {
		name = uvSet.name;
		U = uvSet.U;
		V = uvSet.U;
		uvCounts = uvSet.uvCounts;
		uvIndices = uvSet.uvIndices;

		return *this;
	}

	~SUVSet() {}

	virtual void addPolygon() {
		unsigned int currentLength = U.length();
		
		U.append(0);
		U.append(0);
		U.append(1);
		U.append(1);

		V.append(0);
		V.append(1);
		V.append(1);
		V.append(0);

		for (unsigned int i = currentLength; i < currentLength + 4; i++)
			uvIndices.append(i);

		uvCounts.append(4);
	}

	MString
		name;
	MFloatArray
		U,
		V;
	MIntArray
		uvCounts,
		uvIndices;
};