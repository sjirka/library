#pragma once

#include <maya\MPoint.h>
#include <maya\MVector.h>
#include <maya\MStatus.h>
#include <maya\MPointArray.h>

class SPlane{
public:
	SPlane();
	SPlane(const MPoint& point, const MVector& normal, const MVector& tangent = MVector::zero);
	SPlane(const MPointArray& pointCloud, MStatus *status = NULL);
	SPlane& operator=(const SPlane& plane);

	~SPlane();

	void setOrigin(const MPoint& origin);
	void setNormal(const MVector& normal, bool tangentReset = true);
	void setTangent(const MVector& tangent);
	void resetTangent();
	
	MVector normal() const;
	MVector tangent() const;
	MPoint origin() const;

	MMatrix matrix();

	MPoint project(const MPoint &point);
	MPointArray project(const MPointArray &points);
	MVector project(const MVector &vector);

	bool intersect(const MPoint& point, const MVector& direction, MPoint& intersection, double& parameter);

	MStatus fit(const MPointArray &pointCloud);
	
	SPlane& operator*=(const MMatrix& matrix);
	
	friend SPlane operator*(SPlane plane, const MMatrix& matrix)
	{
		plane *= matrix;
		return plane;
	}

	bool SPlane::operator==(const SPlane& other)
	{
		return this->origin() == other.origin() && this->normal() == other.normal();
	}

	bool SPlane::operator!=(const SPlane& other)
	{
		return !(*this == other);
	}

	void get(double values[4]);

	// Predefined planes
	static SPlane ZERO;
	static SPlane YZ;
	static SPlane XZ;
	static SPlane XY;

protected:
	MPoint m_origin;
	MVector m_normal;
	MVector m_tangent;
};