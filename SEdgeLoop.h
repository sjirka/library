#pragma once

#include <maya\MItMeshEdge.h>
#include <maya\MObject.h>
#include <maya\MGlobal.h>
#include <maya\MFnMesh.h>
#include <maya\MDistance.h>
#include <maya\MPointArray.h>

#include <deque>

class SEdgeLoop {
public:
	SEdgeLoop(MObject *meshPtr=NULL);
	SEdgeLoop(const SEdgeLoop &edgeLoop);
	SEdgeLoop& operator=(const SEdgeLoop& edgeLoop);
	~SEdgeLoop();

	void setMeshPtr(MObject *meshPtr);

	MStatus add(const unsigned int edge);
	bool pushFront(const unsigned int edge, const bool flip = false);
	bool pushBack(const unsigned int edge, const bool flip = false);
	bool contains(const unsigned int edge) const;

	MStatus getLength(double &loopLength);
	unsigned int numEdges() const;
	unsigned int &operator[](const unsigned int index);
	void get(MIntArray &edges);
	MStatus getVertices(MIntArray &vertices);
	MStatus getPoints(MPointArray &points, MSpace::Space space = MSpace::kObject);

	MStatus getEdgeVertices(const unsigned int index, int2 &vertices, bool &flipped);

	static void flip(int2 &vertices);
	bool isFlipped(const unsigned int index);

	bool isClosed();
	void endVertices(int2 &vertices);

	void print();

	void reverse();
	bool isReversed();
	void setReversed(bool reversed);

protected:
	MObject *m_meshPtr = NULL;
	std::deque <unsigned int> m_ordered;
	std::deque <bool> m_flipped;
	bool m_isReversed = false;
};