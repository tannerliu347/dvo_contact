#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>
#include "rgbd_image.h"

/*
template <typename T>
using MatrixXT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
template <typename T>
using Matrix3T = Eigen::Matrix<T, 3, 3>;
*/
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

/*
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
*/

template <typename T>
T bilinearWithoutDepth(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& intensity, const T& x, const T& y) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if(x1 >= intensity.cols() || y1 >= intensity.rows()){
        std::cout << "intensity query out of range\n";
        return 0;
    }
    
    const T x1_weight = x - x0;
    const T x0_weight = T(1.0) - x1_weight;
    const T y1_weight = y - y0;
    const T y0_weight = T(1.0) - y1_weight;

    T val = 0.0;
    T sum = 0.0;

    val += x0_weight * y0_weight * intensity(y0, x0);
    sum += x0_weight * y0_weight;

    val += x1_weight * y0_weight * intensity(y0, x1);
    sum += x1_weight * y0_weight;

    val += x0_weight * y1_weight * intensity(y1, x0);
    sum += x0_weight * y1_weight;

    val += x1_weight * y1_weight * intensity(y1, x1);
    sum += x1_weight * y1_weight;
    
    return val /= sum;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> comp_all_intensities(const Eigen::Matrix<T, 2, Eigen::Dynamic> & pixels_img2 , const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& img2_intensity ){
    int N = pixels_img2.col();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> intensities(N,1);
    for(int col=0; col<N; col++){
        intensities(col, 0) = bilinearWithoutDepth(img2_intensity, pixels_img2(0, col), pixels_img2(1, col));
    }
    return intensities;
}