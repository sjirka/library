#pragma once

#include <maya\MPoint.h>
#include <maya\MVector.h>
#include <maya\MStatus.h>

class SPlane{
public:
	SPlane() {
		m_point = MPoint::origin;
		m_normal = MVector::xAxis;
	}

	SPlane(const MPoint& point, const MVector& normal) {
		m_point = point;
		m_normal = normal;
	}

	~SPlane() {}

	// Set values /////////////////////////////////////////////////////////////////////////////////////

	MStatus setPoint(const MPoint& point) {
		m_point = point;
		return MS::kSuccess;
	}

	MStatus setNormal(const MVector& normal) {
		m_normal = normal;
		return MS::kSuccess;
	}

	// Get Values /////////////////////////////////////////////////////////////////////////////////////

	MStatus getPoint(MPoint& point) {
		point = m_point;
		return MS::kSuccess;
	}

	MStatus getNormal(MVector& normal) {
		normal = m_normal;
		return MS::kSuccess;
	}

	MVector normal() {
		return m_normal;
	}


	// Helper functions ///////////////////////////////////////////////////////////////////////////////

	bool intersect(const MPoint& point, const MVector& direction, MPoint& intersection, double& parameter) {
		double dot = m_normal*direction;
		if (0 == dot)
			return false;

		double d = m_normal*MVector(m_point);
		parameter = (d - m_normal*MVector(point)) / dot;
		intersection = point + direction*parameter;

		if (parameter<0 || parameter>1)
			return false;
		return true;
	}

	SPlane& operator*=(const MMatrix& matrix){
		m_point *= matrix;
		m_normal *= matrix;
		return *this;
	}

	friend SPlane operator*(SPlane plane, const MMatrix& matrix)
	{
		plane *= matrix;
		return plane;
	}

	void get(double values[4]) {
		MPoint intersection;
		double parameter;
		intersect(MPoint::origin, m_normal, intersection, parameter);
		
		values[0] = m_normal.x;
		values[1] = m_normal.y;
		values[2] = m_normal.z;
		values[3] = (intersection-MPoint::origin).length();
	}

	MVector project(const MVector &vector) {
		MVector projection = (m_normal^(m_normal^vector)).normal();
		return projection*(vector*projection);
	}

protected:
	MPoint m_point;
	MVector m_normal;
};