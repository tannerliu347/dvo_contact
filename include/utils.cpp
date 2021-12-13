#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Core>
#include "rgbd_image.h"

// X: 7 parameters of  [qw, qx, qy, qz, tx, ty, tz]
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
T bilinearWithoutDepth(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& intensity, const T& x, const T& y) {
    //const int x0 = static_cast<int>(x);
    //const int y0 = static_cast<int>(y);
    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if(x1 >= intensity.cols() || y1 >= intensity.rows()){
        std::cout << "intensity query out of range\n";
        return 0;
    }
    
    const T x1_weight = x - T(x0);
    const T x0_weight = T(1.0) - x1_weight;
    const T y1_weight = y - T(y0);
    const T y0_weight = T(1.0) - y1_weight;

    T val = 0.0;
    T sum = 0.0;

    std::cout << intensity(x0, y0) << std::endl;
    val += x0_weight * y0_weight * intensity(x0, y0);
    sum += x0_weight * y0_weight;

    std::cout << intensity(x1, y0) << std::endl;
    val += x1_weight * y0_weight * intensity(x1, y0);
    sum += x1_weight * y0_weight;

    std::cout << intensity(x0, y1) << std::endl;
    val += x0_weight * y1_weight * intensity(x0, y1);
    sum += x0_weight * y1_weight;

    std::cout << intensity(x1, y1) << std::endl;
    val += x1_weight * y1_weight * intensity(x1, y1);
    sum += x1_weight * y1_weight;
    
    return val /= sum;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> comp_all_intensities(const Eigen::Matrix<T, 2, Eigen::Dynamic> & pixels_img2 , const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& img2_intensity ){
    int N = pixels_img2.cols();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> intensities(N,1);

    std::cout << "DEBUG before comp_all_intensities loop" << std::endl;
    std::cout << "DEBUG shape of pixels rows: " << pixels_img2.rows() << " cols: "<<  pixels_img2.cols() << std::endl;
    std::cout << "DEBUG value of N = " << N << std::endl;
    std::cout << "DEBUG intensities.cols() = " << intensities.rows() << std::endl;

    // ++++++++++++++ !! To Haoran !! +++++++++++++++++++++++++
    /* This appears to be the problem:
        intensities only has 2 cols,
        but N = 86;
        Thanks;
    */

    for(int col=0; col<N; col++){
        intensities(col, 0) = bilinearWithoutDepth(img2_intensity, pixels_img2(0, col), pixels_img2(1, col));
        std::cout << intensities(col, 0) << std::endl; 
        std::cout << "DEBUG inside comp_all_intesnity loop" << std::endl;
        // TODO seg fault appeared after 2nd loop 
        
    }
    std::cout << "DEBUG after comp_all_intensities loop" << std::endl;
    return intensities;
}

template <typename T>
bool check_pixels_in_img(const Eigen::Matrix<T, 2, Eigen::Dynamic>& pixels_img2, int width, int height){
    for(int i = 0; i < pixels_img2.cols(); i++){
        if(pixels_img2(0, i) >= width || pixels_img2(1, i) >= height){
            std::cout << "intensity query out of range\n";
            return false;
        }
    }
    return true;
}