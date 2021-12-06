#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>
#include "rgbd_image.h"


template <typename T>
void Convert7ParameterQuaternionRepresentationIntoMatrix(const T* X, T* XM){
	T RX[9];

	ceres::QuaternionToRotation(X, RX);

	XM[0] = RX[0];
	XM[1] = RX[1];
	XM[2] = RX[2];
	XM[3] = X[4];

	XM[4] = RX[3];
	XM[5] = RX[4];
	XM[6] = RX[5];
	XM[7] = X[5];

	XM[8] = RX[6];
	XM[9] = RX[7];
	XM[10] = RX[8];
	XM[11] = X[6];

	XM[12] = T(0);
	XM[13] = T(0);
	XM[14] = T(0);
	XM[15] = T(1);

}

template <typename T>
Eigen::Matrix<T, 3, 3> construct_intrinsic_matrix(const dvo::Intrinsic& intrinsic_dvo){
	Eigen::Matrix<T, 3, 3> intrinsic_matrix;
	intrinsic_matrix(0, 0) = T(intrinsic_dvo.fx());
	intrinsic_matrix(1, 1) = T(intrinsic_dvo.fy());
	intrinsic_matrix(0, 2) = T(intrinsic_dvo.ox());
	intrinsic_matrix(1, 2) = T(intrinsic_dvo.oy());	
	intrinsic_matrix(2, 2) = T(1);
	return intrinsic_matrx;
}

template <typename T>
typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> MatrixXT;
typedef Eigen::Matrix<T, 3, 3> Matrix3T;
float bilinearWithDepth(const MatrixXT& intensity, const MatrixXT& depth, const float& x, const float& y, const float& z) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if(x1 >= intensity.cols || y1 >= intensity.rows) return Invalid;
    
    const float x1_weight = x - x0;
    const float x0_weight = 1.0f - x1_weight;
    const float y1_weight = y - y0;
    const float y0_weight = 1.0f - y1_weight;
    const float z_eps = z - 0.05f;

    float val = 0.0f;
    float sum = 0.0f;

    if(std::isfinite(depth.at<float>(y0, x0)) && depth.at<float>(y0, x0) > z_eps) {
        val += x0_weight * y0_weight * intensity.at<float>(y0, x0);
        sum += x0_weight * y0_weight;
    }

    if(std::isfinite(depth.at<float>(y0, x1)) && depth.at<float>(y0, x1) > z_eps) {
        val += x1_weight * y0_weight * intensity.at<float>(y0, x1);
        sum += x1_weight * y0_weight;
    }

    if(std::isfinite(depth.at<float>(y1, x0)) && depth.at<float>(y1, x0) > z_eps) {
        val += x0_weight * y1_weight * intensity.at<float>(y1, x0);
        sum += x0_weight * y1_weight;
    }

    if(std::isfinite(depth.at<float>(y1, x1)) && depth.at<float>(y1, x1) > z_eps) {
        val += x1_weight * y1_weight * intensity.at<float>(y1, x1);
        sum += x1_weight * y1_weight;
    }

    if(sum > 0.0f) {
        val /= sum;
    } else {
        val = Invalid;
    }
}

float bilinearWithoutDepth(const MatrixXT& intensity, const T& x, const T& y) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if(x1 >= intensity.cols() || y1 >= intensity.rows()) return Invalid;
    
    const T x1_weight = x - x0;
    const T x0_weight = T(1.0) - x1_weight;
    const T y1_weight = y - y0;
    const T y0_weight = T(1.0) - y1_weight;
    const T z_eps = z - T(0.05);

    float val = T(0.0);
    float sum = T(0.0);

        val += x0_weight * y0_weight * intensity.at<float>(y0, x0);
        sum += x0_weight * y0_weight;

        val += x1_weight * y0_weight * intensity.at<float>(y0, x1);
        sum += x1_weight * y0_weight;

        val += x0_weight * y1_weight * intensity.at<float>(y1, x0);
        sum += x0_weight * y1_weight;
    }

    if(std::isfinite(depth.at<float>(y1, x1)) && depth.at<float>(y1, x1) > z_eps) {
        val += x1_weight * y1_weight * intensity.at<float>(y1, x1);
        sum += x1_weight * y1_weight;
    }

    if(sum > 0.0f) {
        val /= sum;
    } else {
        val = Invalid;
    }
}