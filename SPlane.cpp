#include "SPlane.h"
#include "SMath.h"

#include <maya\MMatrix.h>

// Predefined planes
SPlane SPlane::ZERO = SPlane(MPoint::origin, MVector::zero);
SPlane SPlane::YZ = SPlane(MPoint::origin, MVector::xAxis);
SPlane SPlane::XZ = SPlane(MPoint::origin, MVector::yAxis);
SPlane SPlane::XY = SPlane(MPoint::origin, MVector::zAxis);

SPlane::SPlane() {
	m_origin = MPoint::origin;
	m_normal = MVector::xAxis;

	resetTangent();
}

SPlane::SPlane(const MPoint& point, const MVector& normal, const MVector& tangent) {
	m_origin = point;
	m_normal = normal;

	(tangent*normal != 0)?setTangent(tangent):resetTangent();
}

SPlane::SPlane(const MPointArray& pointCloud, MStatus *status) {
	*status = fit(pointCloud);
}

SPlane& SPlane::operator=(const SPlane& plane) {
	m_origin = plane.m_origin;
	m_normal = plane.m_normal;
	m_tangent = plane.m_tangent;

	return *this;
}


SPlane::~SPlane() {}

// Set values /////////////////////////////////////////////////////////////////////////////////////

void SPlane::setOrigin(const MPoint& origin) {
	m_origin = origin;
}

void SPlane::setNormal(const MVector& normal, bool tangentReset) {
	m_normal = normal;
	if(tangentReset)
		resetTangent();
}

void SPlane::setTangent(const MVector& tangent) {
	m_tangent = tangent;
}

void SPlane::resetTangent() {
	m_tangent = (m_normal * MVector::xAxis != 0 ) ? m_normal ^ MVector::xAxis : m_normal ^ MVector::yAxis;
}


// Get Values /////////////////////////////////////////////////////////////////////////////////////

MVector SPlane::normal() const {
	return m_normal;
}

MVector SPlane::tangent() const {
	return m_tangent;
}

MPoint SPlane::origin() const {
	return m_origin;
}

void SPlane::get(double values[4]) {
	MPoint intersection;
	double parameter;
	intersect(MPoint::origin, m_normal, intersection, parameter);

	values[0] = m_normal.x;
	values[1] = m_normal.y;
	values[2] = m_normal.z;
	values[3] = (intersection - MPoint::origin).length();
}

MMatrix SPlane::matrix() {
	MVector normal = m_normal.normal();
	MVector cross = (m_tangent ^ normal).normal();
	MVector tangent = (normal ^ cross).normal();

	double mat[4][4] = {
		{ tangent.x, tangent.y, tangent.z, origin().x},
		{ cross.x, cross.y, cross.z, origin().y },
		{ normal.x, normal.y, normal.z, origin().z },
		{0,0,0,1}
	};

	
	MMatrix oMatrix(mat);
	MTransformationMatrix tMatrix(oMatrix);
	tMatrix.setTranslation(m_origin, MSpace::kWorld);
	
	return tMatrix.asMatrix();
}

// Helper functions ///////////////////////////////////////////////////////////////////////////////

MVector SPlane::project(const MVector &vector) {
	MVector projection = (m_normal ^ (m_normal^vector)).normal();
	return projection*(vector*projection);
}

MPoint SPlane::project(const MPoint &point) {
	MVector v = point - m_origin;
	double dist = v*m_normal;
	return point - dist*m_normal.normal();
}

MPointArray SPlane::project(const MPointArray &points) {
	MPointArray projectedPoints;
	for (unsigned int i = 0; i < points.length(); i++)
		projectedPoints.append(project(points[i]));
	return projectedPoints;
}

bool SPlane::intersect(const MPoint& point, const MVector& direction, MPoint& intersection, double& parameter) {
	double dot = m_normal*direction;
	if (0 == dot)
		return false;

	double d = m_normal*MVector(m_origin);
	parameter = (d - m_normal*MVector(point)) / dot;
	intersection = point + direction*parameter;

	if (parameter<0 || parameter>1)
		return false;
	return true;
}

SPlane& SPlane::operator*=(const MMatrix& matrix) {
	MTransformationMatrix tMatrix(matrix);
	double scale[3] = { 1,1,1 };
	double shear[3] = { 0,0,0 };
	tMatrix.setScale(scale, MSpace::kWorld);
	tMatrix.setShear(shear, MSpace::kWorld);
	MMatrix oMatrix = tMatrix.asMatrix();

	m_origin *= oMatrix;
	m_normal *= oMatrix;
	return *this;
}

MStatus SPlane::fit(const MPointArray &points) {
	MStatus status;
	
	// At least 3 points
	unsigned int n = points.length();
	if (n < 3)
		return MS::kInvalidParameter;

	MPoint sum;
	for (unsigned int i = 0; i < points.length(); i++)
		sum += points[i];
	MPoint centroid = sum / n;

	double
		xx, xy, xz,
		yy, yz, zz;

	xx = xy = xz = yy = yz = zz = 0;

	for (unsigned int i = 0; i < points.length(); i++) {
		MVector r = points[i] - centroid;
		xx += r.x*r.x;
		xy += r.x*r.y;
		xz += r.x*r.z;
		yy += r.y*r.y;
		yz += r.y*r.z;
		zz += r.z*r.z;
	}

	double det_x = yy*zz - yz*yz;
	double det_y = xx*zz - xz*xz;
	double det_z = xx*yy - xy*xy;

	MDoubleArray det_values;
	det_values.append(det_x);
	det_values.append(det_y);
	det_values.append(det_z);

	double det_max = SMath::max(det_values);
	if (det_max <= 0)
		return MS::kFailure;

	MVector dir;

	if (det_max == det_x){
		double a = (xz*yz - xy*zz) / det_x;
		double b = (xy*yz - xz*yy) / det_x;
		dir = MVector(1.0, a, b);
	}
	else if (det_max == det_y){
		double a = (yz*xz - xy*zz) / det_y;
		double b = (xy*xz - yz*xx) / det_y;
		dir = MVector(a, 1.0, b);
	}
	else {
		double a = (yz*xy - xz*yy) / det_z;
		double b = (xz*xy - yz*xx) / det_z;
		dir = MVector(a, b, 1.0);
	};

	setOrigin(centroid);
	setNormal(dir, true);

	return MS::kSuccess;
}