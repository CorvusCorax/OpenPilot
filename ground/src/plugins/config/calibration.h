#ifndef AHRS_CALIBRATION_HPP
#define AHRS_CALIBRATION_HPP

#include <Eigen/Core>
#include <cstdlib>
using std::size_t;
using namespace Eigen;

Vector3f
twostep_bias_only(const Vector3f samples[], 
		size_t n_samples,
		const Vector3f& referenceField,
		const float noise);

void 
twostep_bias_scale(Vector3f& bias, 
		Vector3f& scale, 
		const Vector3f samples[], 
		const size_t n_samples,
		const Vector3f& referenceField,
		const float noise);

void 
twostep_bias_scale(Vector3f& bias, 
		Matrix3f& scale, 
		const Vector3f samples[], 
		const size_t n_samples,
		const Vector3f& referenceField,
		const float noise);

void
openpilot_bias_scale(Vector3f& bias,
		Vector3f& scale, 
		const Vector3f samples[], 
		const size_t n_samples,
		const Vector3f& referenceField);

#endif // !defined AHRS_CALIBRATION_HPP

