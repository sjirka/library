#pragma once

#include <maya/MPointArray.h>
#include <maya\MDoubleArray.h>
#include <maya\MVectorArray.h>

class SMath{
public:
	SMath(){};
	virtual ~SMath(){};
	// Get squared length /////////////////////////////////////////////////////////////////////////
	static double getSquaredLength(const MVector& vector) {
		return (vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
	};

	// Get radius and center of a circle defined by 3 points //////////////////////////////////////
	static MStatus threePointCircle(MPointArray& points, MPoint& center, double &radius) {
		if (points.length() != 3)
			return MS::kInvalidParameter;

		MVector
			t(points[1] - points[0]),
			u(points[2] - points[0]),
			v(points[2] - points[1]);

		if (t.isParallel(u))
			return MS::kFailure;

		MVector w = t^u;
		double wsl = getSquaredLength(w);

		double iwsl2 = 1.0 / (2.0*wsl);
		double tt = t*t;
		double uu = u*u;

		center = points[0] + (u*tt*(u*v) - t*uu*(t*v)) * iwsl2;
		radius = (points[0] - center).length();

		return MS::kSuccess;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////
	// Lowess data smaoothing /////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////

	// Double
	static MDoubleArray lowess(MDoubleArray& data, bool isLoop = false, unsigned int radius = 5, bool zeroOnly = false) {

		int len = data.length();
		int lastID = len - 1;

		MDoubleArray smooth(len);
		for (int i = 0; i < len; i++) {
			
			// Only affects zero values
			if (zeroOnly && 0 != data[i]) {
				smooth[i] = data[i];
				continue;
			}

			double wSum = 0;
			for (int c = -(int)radius; c < (int)radius; c++) {
				int nID = i + c;

				if (isLoop && (nID < 0 || nID>lastID))
					nID += -(nID / abs(nID))*lastID;

				if (0 > nID || len - 1 < nID)
					continue;

				double w = pow(1 - pow(abs((double)c) / radius, 3), 3);
				wSum += w;
				smooth[i] += data[nID] * w;
			}
			if (wSum>0)
				smooth[i] /= wSum;
		}

		return smooth;
	};

	// Vector
	static MVectorArray lowess(MVectorArray& data, bool isLoop = false, unsigned int radius = 5, bool zeroOnly = false) {

		int len = data.length();
		int lastID = len - 1;

		MVectorArray smooth(len);
		for (int i = 0; i < len; i++) {
			
			// Only affects zero values
			if (zeroOnly && MVector::zero != data[i]){
				smooth[i] = data[i];
				continue;
			}

			double wSum = 0;
			for (int c = -(int)radius; c < (int)radius; c++) {
				int nID = i + c;

				if (isLoop && (nID < 0 || nID>lastID))
					nID += -(nID / abs(nID))*lastID;

				if (0 > nID || len - 1 < nID)
					continue;

				double w = pow(1 - pow(abs((double)c) / radius, 3), 3);
				wSum += w;
				smooth[i] += data[nID] * w;
			}
			if (wSum>0)
				smooth[i] /= wSum;
		}

		return smooth;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////

	static double max(MDoubleArray &values) {
		double maxValue = 0;
		for (unsigned int i = 0; i < values.length(); i++)
			if (i == 0 || maxValue < values[i])
				maxValue = values[i];
		return maxValue;
	}

	static double min(MDoubleArray &values) {
		double minValue = 0;
		for (unsigned int i = 0; i < values.length(); i++)
			if (i == 0 || values[i] < minValue)
				minValue = values[i];
		return minValue;
	}

	static MDoubleArray remap(MDoubleArray &values, double min, double max, double newMin, double newMax) {
		MDoubleArray newValues(values);
		double oldDomain = max - min;
		double newDomain = newMax - newMin;
		for (unsigned int i = 0; i < values.length(); i++)
			newValues[i] = newMin + (values[i] - min) * newDomain / oldDomain;
		return newValues;
	}

	static double polyRadius(double unsmoothedRadius, double radialSegments, double subdivs) {
		double newRadius = unsmoothedRadius;
		for (unsigned int i = 0; i < subdivs; i++) {
			double angle = 2 * M_PI / (radialSegments * pow(2, subdivs - i));
			newRadius /= cos(angle);
		}
		return newRadius;
	}

};