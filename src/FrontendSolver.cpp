#include "FrontendSolver.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include  <ceres/local_parameterization.h>
#include <chrono>
#include <vector>
#include "utils.hpp"
#include <math.h>
#include <string>
using namespace std;

/*
class ReprojectionError {
 public:
  ReprojectionError(
      const Eigen::Matrix<double, 3, 4>& projection_matrix,
      const Eigen::Vector2d& feature)
      : projection_matrix_(projection_matrix), feature_(feature) {}

  template <typename T>
  bool operator()(const T* input_point, T* reprojection_error) const {
    Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);

    // Multiply the point with the projection matrix, then perform homogeneous
    // normalization to obtain the 2D pixel location of the reprojection.
    const Eigen::Matrix<T, 2, 1> reprojected_pixel =
        (projection_matrix_.cast<T>() * input_point).hnormalized();

    // Reprojection error is the distance from the reprojection to the observed
    // feature location.
    reprojection_error[0] = feature_[0] - reprojected_pixel[0];
    reprojection_error[1] = feature_[1] - reprojected_pixel[1];
    return true;
  }

 private:
  const Eigen::Matrix<double, 3, 4>& projection_matrix_;
  const Eigen::Vector2d& feature_;
};
*/

class PhotometricError {

 public:
  //
  //pc_1: point cloud of selected feature points, eigen matrix of (4 x n) each column: [x, y, z, 1]
  PhotometricError(const dvo::PointCloud& pc1, const Eigen::VectorXd& img1_intensity, const dvo::RgbdImage& img2) : pc1_ (pc1), 
                                                                           img1_intensity_ (img1_intensity),
                                                                           img2_(img2),
                                                                           img_width_(img2.width),
                                                                           img_height_(img2.height),
                                                                           intrinsics_cam2_(img2.camera().intrinsics().get_intrinsic_matrix().cast<double>()),
                                                                           img2_intensity_(Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (img2.intensity.ptr<double>(), img2.intensity.rows, img2.intensity.cols)),
                                                                           img2_depth_(Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (img2.depth.ptr<double>(), img2.depth.rows, img2.depth.cols)){std::cout << "Photometric Error initialization finish\n";} 

  //transform is a 7 parameter representation of transformation (quaternion, translation) T [7]
  template <typename L>
  bool operator()(const L* quat_trans, L* residual) const {
    int N = pc1_.cols(); //number of selected points
    L transform[16];
    //transform quaternion-trans parametrization to matrix 
    Convert7ParameterQuaternionRepresentationIntoMatrix<L>(quat_trans, transform);
    //TODO, verify T[16] to Eigen matrix conversion
    //Extrinsics: 4 x 4 matrix
    Eigen::Map<const Eigen::Matrix<L, 4, 4, Eigen::RowMajor>> extrinsics (transform);
    // std::cout << "After mapping extrinsics\n";
    //transform point cloud to camera 2 image plane through extrinsics and intrinsics
    const Eigen::Matrix<L, 2, Eigen::Dynamic> pixels_img2  = (intrinsics_cam2_.cast<L>() * (extrinsics * pc1_.cast<L>()).colwise().hnormalized()).colwise().hnormalized();
    // std::cout << "After pixels_img2\n";
    //TODO: check pixels_img2 are in img2
    //check_pixels_in_img(pixels_img2, img_width_, img_height_);
    //Check pixels in image and Get intensity in image 2
    auto intensity_in_img2 = comp_all_intensities<L>(pixels_img2 , img2_intensity_.cast<L>());
    // std::cout << "After compting intensity_in_img2 \n";
    residual[0] = (intensity_in_img2 - img1_intensity_.cast<L>()).norm();
    return true;
  }

 private:
    const dvo::PointCloud& pc1_;
    const Eigen::VectorXd& img1_intensity_;
    const dvo::RgbdImage& img2_; 
    int img_width_;
    int img_height_;
    Eigen::Matrix3d intrinsics_cam2_;
    const Eigen::MatrixXd img2_intensity_;
    const Eigen::MatrixXd  img2_depth_;
};


//using purely external modified warp_intensity function
int counter_ = 0;
class PhotometricError_ext {

 public:
  //
  //pc_1: point cloud of selected feature points, eigen matrix of (4 x n) each column: [x, y, z, 1]
  PhotometricError_ext(const dvo::PointCloud& pc1, const Eigen::VectorXd& img1_intensity, const dvo::RgbdImage& img2, bool vis) : pc1_ (pc1), 
                                                                           img1_intensity_ (img1_intensity),
                                                                           img2_(img2),
                                                                           vis_(vis)
                                                                            {std::cout << "Photometric Error initialization finish\n";} 

  //transform is a 7 parameter representation of transformation (quaternion, translation) T [7]
  bool operator()(const double* const quat, const double* const trans, double* residual) const {
    counter_ ++;
    int N = pc1_.cols(); //number of selected points
    double quat_trans[7] = {quat[0], quat[1], quat[2], quat[3], trans[0], trans[1], trans[2]};
    double transform[16];
    //transform quaternion-trans parametrization to matrix 
    Convert7ParameterQuaternionRepresentationIntoMatrix(quat_trans, transform);
    //TODO, verify T[16] to Eigen matrix conversion
    //Extrinsics: 4 x 4 matrix
    
    // std::cout << "After quaternion conversion\n";
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> extrinsics (transform);
    bool cur_vis = (counter_ % freq_ == 0) && vis_;
    string filename = std::to_string(counter_) + ".png";
    Eigen::VectorXd warped_intensity_in_img2 = img2_.warpIntensity2(extrinsics.cast<float>(), pc1_, img2_.camera().intrinsics(), cur_vis, filename);
    //std::cout << "DEBUG warped intensity" << intensity_in_img2;
    //std::cout << "DEBUG img1_intensity_" <<img1_intensity_;
    auto intensity_diff = warped_intensity_in_img2 - img1_intensity_;

    //residual[0] = intensity_diff.norm();
    residual[0] = cal_cost_discard_invalid_depth(warped_intensity_in_img2, img1_intensity_);
    std::cout << "Residual  " << residual[0] << std::endl;  
    std::cout << "Residual (discard invalid depth) " << cal_cost_discard_invalid_depth(warped_intensity_in_img2, img1_intensity_) << std::endl; 
    return true;
  }

 private:
    const dvo::PointCloud& pc1_;
    const Eigen::VectorXd& img1_intensity_;
    const dvo::RgbdImage& img2_; 
    bool vis_;
    bool cur_vis_;
    int freq_= 10;
};

double cal_cost_discard_invalid_depth(const Eigen::VectorXd& warped_img2_intensity, const Eigen::VectorXd& img1_intensity){
    int num_invalid = 0;
    double cost_squared = 0;
    double l1_cost = 0;
    int N =  warped_img2_intensity.rows();
    for(int i = 0; i < N; i++){
        //invalid intensities are set to negative
        if (warped_img2_intensity(i) < 0){
            num_invalid++;
            continue;
        } else{
            cost_squared += std::pow(warped_img2_intensity(i) - img1_intensity(i), 2);
            l1_cost += std::abs(warped_img2_intensity(i) - img1_intensity(i));
        }
    }
    std::cout << "intensity RMSE: " << std::sqrt(cost_squared/(N-num_invalid)) <<std::endl;
    std::cout << "intensity mean err: " << l1_cost/(N-num_invalid) <<std::endl;
    return std::sqrt(cost_squared);
}

//Initial guess is a 7 parameter representation of transformation (quaternion, translation) double [7]
void FrontendSolver::solve(const dvo::PointCloud& pc1, const Eigen::VectorXd& img1_intensity, const dvo::RgbdImage& img2, const double* initial_guess)
{   
    ceres::Problem problem;
    // initial guess of transform: 7 parameters of  [qw, qx, qy, qz, tx, ty, tz]
    double transform[7];
    //std::copy(std::begin(initial_guess), std::end(initial_guess), std::begin(transform));
    std::cout << "initial guess:\n" << std::endl;
    for(int i = 0; i < 7; i++){
        transform[i] = initial_guess[i];
        std::cout <<transform[i] << " ";
    }
    std::cout << std::endl;
    double quat[4] = {transform[0], transform[1], transform[2], transform[3]};
    double trans[3] = {transform[4], transform[5], transform[6]};

    problem.AddResidualBlock (
        new ceres::NumericDiffCostFunction<PhotometricError_ext, ceres::CENTRAL, 1, 4, 3>(
            new PhotometricError_ext ( pc1, img1_intensity, img2, vis_)
        ),
        nullptr,
        quat,
        trans
    );

    //Add quaternion constraint
    ceres::QuaternionParameterization*  quaternion_param = new ceres::QuaternionParameterization;
    problem.SetParameterization(quat, quaternion_param);

    // configure solver
    ceres::Solver::Options options;    
    options.linear_solver_type = ceres::DENSE_QR;  
    options.minimizer_progress_to_stdout = true;   

    ceres::Solver::Summary summary;               
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary ); 
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // solver output
    cout<<summary.FullReport() <<endl;
    cout<<"estimated transform quat:\n";
    for ( auto t:quat ) cout<< t <<" ";
    cout<<"\nestimated transform trans:\n";
    for ( auto t:trans ) cout<< t <<" ";
    cout<<endl; 
}

/*
//Initial guess is a 7 parameter representation of transformation (quaternion, translation) double [7]
Eigen::Matrix4d FrontendSolver::solve(const dvo::RgbdImage& img1, const dvo::RgbdImage& img2)
{   
    
    // img1.buildPointCloud();
    dvo::PointCloud img1_pc;
    img1.camera().buildPointCloud(img1.depth, img1_pc);

    //img1.PointSelection(); // (place holder) build feature points function
    
    // template<typename T>
    // Eigen::Matrix<T, 1, Eigen::Dynamic> feature_points_idx = img1.getFeaturePointsIdx(); // place holder methods
    
    // receive feature pointers
    //auto feature_points_starting_pointer = img1.getFeaturePointsStart(); // place holder method
    //auto feature_points_ending_pointer = img1.getFeaturePointEnd(); //place holder method 
    
    // initialize space for getting out all points
    std::vector<dvo::PtIntensityDepth> all_points;
    auto current_iter = feature_point_starting_pointer;

    // index counter 
    size_t tmp_idx = 0;
    while (current_iter != feature_point_starting_pointer)
    {
        all_points.push_back(*(current_iter));
        tmp_idx++ ;
    }
    
    // construct intermediate results for solver class
    dvo::PointCloud feature_pc;
    Eigen::VectorXd feature_intensity(all_points.size());
    
    for (size_t i = 0; all_points.size(); i++) 
    {
        feature_pc.col(i) = all_points[i].getPointVec();
        feature_intensity(i) = all_points[i].getIntensityDepty()(0);
    }
    
    
    template<typename T>
    Eigen::Matrix<T, 4, Eigen::Dynamic> feature_points;
    
    for (size_t i = 0; i < feature_points_idx.cols(); i++)
    {
        idx = feature_points_idx.cols(i);
        feature_points.col(i) = img1.point_cloud.col(idx);
    }

    Eigen::Matrix<T, 2, Eigen::Dynamic> feature_points_uv;
    for (size_t i = 0; i < feature_points_idx.cols(); i++)
    {
        feature_points_uv(i, 0) = T (static_cast<int>(feature_points_idx(0, i) / img1.width)); 
        feature_points_uv(i, 1) = T (feature_points_idx(0, i)-1 % img1.width);
    }

    Eigen::VectorXd img1_selected_intensity;
    for (size_t i = 0; i < feature_points_idx.cols(); i++)
    {
        u = feature_points_uv(i, 0);
        v = feature_points_uv(i, 1);
        img1_selected_intensity(i) = img1.intensity.at<double>(u, v);
    }
    

    // 构建最小二乘问题
    ceres::Problem problem;
    // transform: 7 parameters of  [qw, qx, qy, qz, tx, ty, tz]
    double transform[7] = {1, 0, 0, 0, 0, 0, 0};
    std::copy(std::begin(initial_guess), std::end(initial_guess), std::start(transform)); 
    problem.AddResidualBlock (     // 向问题中添加误差项
    // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
        new ceres::AutoDiffCostFunction<PhotometricError, 1, 7> ( 
            new PhotometricError ( feature_pc, feature_intensity, img2)
        ),
        nullptr,            // 核函数，这里不使用，为空
        transform                 // 待估计参数
    );


    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果
    cout<<summary.BriefReport() <<endl;
    cout<<"estimated transform";
    for ( auto t:transform ) cout<< t <<" ";
    cout<<endl; 
}
*/