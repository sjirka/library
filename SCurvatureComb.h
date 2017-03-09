#pragma once

#include <maya\MObject.h>
#include <maya\MPointArray.h>
#include <maya\MFnNurbsCurve.h>
#include <maya\MVector.h>
#include <maya\MDoubleArray.h>
#include <maya\MGlobal.h>

#include "../_library/SData.h"
#include "../_library/SMath.h"

class SCurvatureComb
{
public:
	SCurvatureComb() {};
	SCurvatureComb(const MObject& curve, unsigned int samples = 100, double scale = 100, MStatus *status=NULL) {
		*status = setCurve(curve);
		setSamples(samples);
		setScale(scale);
	};
	~SCurvatureComb() {};

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Set variables //////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////

	MStatus setCurve(const MObject& curve){
		MStatus status;

		// Check data
		if (!SData::isCurve(curve))
			return MS::kInvalidParameter;
		
		// Store curve
		m_curve = curve;

		// Store curve degree
		MFnNurbsCurve fnCurve(m_curve, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		m_degree = fnCurve.degree(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		m_isClosed = (MFnNurbsCurve::kOpen != fnCurve.form(&status)) ? true : false;
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;
	};

	void setScale(double scale) {
		m_scale = scale;
	};

	void setSamples(unsigned int samples) {
		m_samples = (4 < samples) ? samples : 5;
	};

	void setFilter(unsigned int radius) {
		m_filterNoise = true;
		m_filterRadius = radius;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Get variables //////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////

	MStatus getPoints(MPointArray& samplePoints, MPointArray& crvPoints) {
		MStatus status;

		status = generateValues();
		CHECK_MSTATUS_AND_RETURN_IT(status);

		samplePoints = m_samplePoints;
		crvPoints = m_crvPoints;
		
		return MS::kSuccess;
	};

protected:
	MObject
		m_curve;
	double
		m_scale = 1.0;
	unsigned int
		m_samples = 100,
		m_degree = 0,
		m_filterRadius = 5;
	bool
		m_isClosed = false,
		m_filterNoise = false;

	MPointArray
		m_samplePoints,
		m_crvPoints;

	MVectorArray
		m_normals;

	MDoubleArray
		m_curvature;

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Main functions /////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////

	MStatus generateValues() {
		MStatus status;

		resetValues();

		switch (m_degree){
		case 1: {
			status = sampleCurve();
			CHECK_MSTATUS_AND_RETURN_IT(status);

			for (unsigned int i = ((m_isClosed) ? 0 : 1); i < ((m_isClosed) ? m_samples : m_samples - 1); i++) {
				MPointArray triad;
				if (0 == i || m_samples - 1 == i) {
					triad.append(m_samplePoints[1]);
					triad.append(m_samplePoints[0]);
					triad.append(m_samplePoints[m_samples - 2]);
				}
				else {
					triad.append(m_samplePoints[i - 1]);
					triad.append(m_samplePoints[i]);
					triad.append(m_samplePoints[i + 1]);
				}

				MPoint center; double radius = 0;
				status = SMath::threePointCircle(triad, center, radius);
				if ((MS::kSuccess != status))
					continue;

				MVector normal = ((triad[1] - triad[0]).normal() + (triad[1] - triad[2]).normal()).normal();

				m_curvature.set(1 / radius, i);
				m_normals.set(normal, i);
			}

			// Since first and last point don't contain any information, remove them
			if (!m_isClosed) {
				m_samplePoints.remove(m_samples - 1);
				m_samplePoints.remove(0);
				m_crvPoints.remove(m_samples - 1);
				m_crvPoints.remove(0);
				m_curvature.remove(m_samples - 1);
				m_curvature.remove(0);
				m_normals.remove(m_samples - 1);
				m_normals.remove(0);
			}

			// Smooth values
			if (m_filterNoise) {
				m_curvature = SMath::lowess(m_curvature, m_isClosed, m_filterRadius);
				m_normals = SMath::lowess(m_normals, m_isClosed, m_filterRadius, true);
			}

			break;
		}
		default:
			break;
		}

		// Calculate curvature comb points
		for (unsigned int i = 0; i < m_samples; i++)
			m_crvPoints.set( m_samplePoints[i] + m_normals[i] * m_curvature[i] * m_scale, i);

		return MS::kSuccess;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Helper functions ///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////

	void resetValues() {
		m_samplePoints = MPointArray(m_samples);
		m_crvPoints = MPointArray(m_samples);
		m_normals = MVectorArray(m_samples);
		m_curvature = MDoubleArray(m_samples);
	};

	MStatus sampleCurve() {
		MStatus status;

		MFnNurbsCurve fnCurve(m_curve, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		double crvLen = fnCurve.length(.001, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);
		double lenStep = crvLen / (m_samples - 1);

		for (unsigned int i = 0; i < m_samples; i++) {
			double sampleParam = fnCurve.findParamFromLength(lenStep*i);
			status = fnCurve.getPointAtParam(sampleParam, m_samplePoints[i], MSpace::kObject);
			CHECK_MSTATUS_AND_RETURN_IT(status);
		}

		return MS::kSuccess;
	};
};