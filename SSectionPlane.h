#pragma once
#include "SPlane.h"

#include <set>
#include <map>
#include <maya\MItMeshPolygon.h>
#include <maya\MItMeshEdge.h>
#include <maya\MItMeshVertex.h>
#include <maya\MFnNurbsCurveData.h>
#include <maya\MFnNurbsCurve.h>

class SSectionPlane : public SPlane{
public:
	SSectionPlane(const MPoint& point, const MVector& normal) {
		m_point = point;
		m_normal = normal;
	}

	~SSectionPlane(){}

	MStatus getIntersections(MObject &mesh, MObjectArray& curves, MMatrix& transform = MMatrix()) {
		MStatus status;
		
		curves.clear();

		if (mesh.apiType() != MFn::kMesh &&
			mesh.apiType() != MFn::kMeshData &&
			mesh.apiType() != MFn::kMeshGeom)
			return MS::kInvalidParameter;

		clear();

		status = generatePlaneSections(mesh, transform, curves);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;
	}

	static MStatus generateNurbsCurve(MPointArray& editPoints, MFnNurbsCurve::Form form, MObject& curveData) {
		MStatus status;

		if (2 > editPoints.length() || (MFnNurbsCurve::kClosed==form && 3>editPoints.length()))
			return MS::kInvalidParameter;

		MFnNurbsCurveData fnCurveData;
		curveData = fnCurveData.create(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MFnNurbsCurve fnCurve;
		fnCurve.createWithEditPoints(editPoints, 1, form, false, false, true, curveData, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;
	}

protected:
	std::set<unsigned int> m_intersectingEdges;
	std::map<unsigned int, bool> m_isEdgeOnBoundary;
	std::map<unsigned int, double> m_intersectionParameters;
	std::map<unsigned int, MPoint> m_intersectionPoints;

	void clear() {
		m_intersectingEdges.clear();
		m_intersectionParameters.clear();
		m_intersectionPoints.clear();
		m_isEdgeOnBoundary.clear();
	}

	MStatus generatePlaneSections(MObject &mesh, MMatrix &transform, MObjectArray &sectionCurves) {
		MStatus status;

		// Iterators
		MItMeshPolygon itPolygon(mesh, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MItMeshEdge itEdge(mesh, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		MItMeshVertex itVertex(mesh, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		//Find intersections
		status = findIntersections(mesh, transform);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Sort intersections and create curves
		while (0<m_intersectingEdges.size()) {
			auto first = *m_intersectingEdges.begin();

			for (auto iter : m_intersectingEdges)
				if (m_isEdgeOnBoundary[iter]) {
					first = iter;
					break;
				}

			MIntArray sortedEdges;
			bool isClosed;
			status = sortIntersections(first, itPolygon, itEdge, itVertex, sortedEdges, isClosed);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			if (2 > sortedEdges.length())
				continue;

			MPointArray sectionPoints;
			for (unsigned int i = 0; i < sortedEdges.length(); i++)
				if(0 == sectionPoints.length() || !sectionPoints[sectionPoints.length()-1].isEquivalent(m_intersectionPoints[sortedEdges[i]], 0.01))
					sectionPoints.append(m_intersectionPoints[sortedEdges[i]]);
			if (isClosed && sectionPoints[0] != sectionPoints[sectionPoints.length() - 1])
				sectionPoints.append(sectionPoints[0]);

			if (2 > sectionPoints.length() || (isClosed && 3 > sectionPoints.length()))
				continue;

			MObject sectionCurve;
			status = generateNurbsCurve(sectionPoints, (isClosed) ? MFnNurbsCurve::kClosed : MFnNurbsCurve::kOpen, sectionCurve);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			if(!sectionCurve.isNull())
				sectionCurves.append(sectionCurve);
		}

		return MS::kSuccess;
	}

	MStatus findIntersections(MObject& mesh, MMatrix& transform){
		MStatus status;

		MItMeshEdge itEdge(mesh, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		for (itEdge.reset(); !itEdge.isDone(); itEdge.next()) {
			double parameter;
			MPoint intersection;

			MPoint pointA = itEdge.point(0)*transform;
			MPoint pointB = itEdge.point(1)*transform;
			MVector direction = pointB - pointA;

			if (!intersect(pointA, direction, intersection, parameter))
				continue;

			unsigned int index = itEdge.index();
			m_intersectingEdges.insert(index);
			m_isEdgeOnBoundary[index] = itEdge.onBoundary();
			m_intersectionParameters[index] = parameter;
			m_intersectionPoints[index] = intersection;
		}

		return MS::kSuccess;
	}

	MStatus sortIntersections(unsigned int currentEdge, MItMeshPolygon &itPolygon, MItMeshEdge &itEdge, MItMeshVertex &itVertex, MIntArray& sortedEdges, bool &isClosed) {
		MStatus status;

		sortedEdges.append(currentEdge);
		m_intersectingEdges.erase(currentEdge);

		MIntArray connectedFaces;
		bool onBoundary;
		int dummyIndex;

		status = itEdge.setIndex(currentEdge, dummyIndex);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		onBoundary = itEdge.onBoundary(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// If inersection on vertex, get rid of faces sharing this vertex
		double parameter = m_intersectionParameters[currentEdge];
		if (1==parameter || 0==parameter) {
			status = itVertex.setIndex(itEdge.index(int(parameter)), dummyIndex);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			status = itVertex.getConnectedFaces(connectedFaces);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			clearConnected(itVertex);
		}
		else {
			itEdge.getConnectedFaces(connectedFaces, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}

		// Iterate over connected faces, looking for next intersection
		for (unsigned int f = 0; f<connectedFaces.length(); f++) {
			status = itPolygon.setIndex(connectedFaces[f], dummyIndex);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			MIntArray connectedEdges;
			status = itPolygon.getEdges(connectedEdges);
			CHECK_MSTATUS_AND_RETURN_IT(status);
			for (unsigned int e = 0; e < connectedEdges.length(); e++)
				if (m_intersectingEdges.end() != m_intersectingEdges.find(connectedEdges[e]))
					return sortIntersections(connectedEdges[e], itPolygon, itEdge, itVertex, sortedEdges, isClosed);
		}

		isClosed = (onBoundary) ? false : true;

		return MS::kSuccess;
	}

	MStatus clearConnected(MItMeshVertex &itVertex) {
		MStatus status;

		MIntArray connectedEdges;
		status = itVertex.getConnectedEdges(connectedEdges);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		for (unsigned int e = 0; e < connectedEdges.length(); e++)
			m_intersectingEdges.erase(connectedEdges[e]);

		return MS::kSuccess;
	}
};