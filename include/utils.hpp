#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Core>
#include "rgbd_image.h"
#include <ceres/jet.h>
#include <experimental/filesystem>
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

//q_t1, q_t2, q_t21 of the form [qw, qx, qy, qz, tx, ty, tz]
// calculates the 7 parameter quaternion-translation representation of 
// the relative transform from {2} to {1} (frame transform from {1} to {2})
template <typename T>
void calc_transform_from_quaternion(const T* q_t1, const T* q_t2, T* q_t21, bool verbose=false){
    double T1_arr[16];
    double T2_arr[16];
    Convert7ParameterQuaternionRepresentationIntoMatrix(q_t1, T1_arr);
    Convert7ParameterQuaternionRepresentationIntoMatrix(q_t2, T2_arr);
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T1 (T1_arr);
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T2 (T2_arr);
    //Transform that represents frame 1 in frame 2
    auto T21 = T2.inverse() * T1;
    if(verbose){
        std::cout << "T21: \n";
        std::cout << T21 << std::endl;
    }
    Eigen::Quaterniond q(T21.block<3,3>(0, 0));
    q_t21[0] = q.w();
    q_t21[1] = q.x();
    q_t21[2] = q.y();
    q_t21[3] = q.z();
    q_t21[4] = T21(0, 3);
    q_t21[5] = T21(1, 3);
    q_t21[6] = T21(2, 3);
    if(verbose){
        std::cout << "relative quaternion\n";
        for(int i = 0; i < 7; i++){
            std::cout << q_t21[i] << ", ";
        }
        std::cout << std::endl; 
    }
}

template <typename T>
void normalize_quaternion(const T* q){
    T normalizer = ceres::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= normalizer;
    q[1] /= normalizer;
    q[2] /= normalizer;
    q[3] /= normalizer;
}


// define color space
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar MAGENTA(255, 0, 255);
const cv::Scalar CYAN(255, 255, 0);

using namespace cv;
using namespace std;
template<typename T>
void keypoint_plotter(Mat& img, vector<T>& points,  
                                char color, 
                                string name="keypt_plot.png",
                                bool write_to_file=false,
                                int radius=5,
                                int thickness=2
                                )
{
    
    cv::Scalar plot_color;
    switch (color)
    {
    case 'g': plot_color = GREEN;   break;
    case 'b': plot_color = BLUE;    break;
    case 'r': plot_color = RED;     break;
    case 'c': plot_color = CYAN;    break;
    case 'y': plot_color = YELLOW;  break;
    case 'm': plot_color = MAGENTA; break;
    default:  plot_color = GREEN;   break;
    }
    
    for (int i = 0; i < points.size(); i++) 
    {
    cv::circle(img, points[i], radius, plot_color, thickness);
    }
    std::string cwd = get_current_dir_name();
    if(write_to_file){
        imwrite(cwd+"/runtime_img/"+name, img);
    }
    
    // imwrite(cwd + relative_path, img);
}