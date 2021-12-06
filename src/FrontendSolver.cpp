#include "FrontendSolver.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include "utils.cpp"

/*
class FrontendSolver {
private:
    dvo::Intrinsic intrinsic_;
public:
    FrontendSolver(const dvo::Intrinsic & intrinsic);
    Eigen::Matrix4d solve (const RGBDImage& img1, const RGBDImage& img2, Eigen::Matrix initial_guess=identity());
};
*/

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
template <typename T>
typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> MatrixXT;
typedef Eigen::Matrix<T, 3, 3> Matrix3T;
 public:
  //
  //pc_1: point cloud of selected feature points, eigen matrix of (4 x n) each column: [x, y, z, 1]
  
  PhotometricError(const dvo::PointCloud& pc1, const Eigen::VectorXd& img1_intensity, const dvo::RgbdImage& img2) : pc1_ (pc1), 
                                                                           img1_intensity_ (img1_intensity),
                                                                           img2_(img2),
                                                                           img_width_(img2.),
                                                                           img_height_(img2.height),
                                                                           intrinsics_cam2_(construct_intrinsic_matrix(img2.camera().intrinsics())),
                                                                           img2_intensity_(Eigen::Map<MatrixXT> (img2.intensity.ptr<T>(), img2.intensity.rows, img2.intensity.cols)),
                                                                           img2_depth_(Eigen::Map<MatrixXT> (img2.depth.ptr<T>(), img2.depth.rows, img2.depth.cols)){} 

  //transform is a 7 parameter representation of transformation (quaternion, translation) T [7]
  bool operator()(const T* quat_trans, T* residual) const {
    N = pc1.cols(); //number of selected points
    T transform[16];
    intrinsics
    //transform quaternion-trans parametrization to matrix 
    Convert7ParameterQuaternionRepresentationIntoMatrix(quat_trans, transform);
    //TODO, verify T[16] to Eigen matrix conversion
    Eigen::Map<Eigen::Matrix<T, 4, 4, Eigen::RowMajor>> extrinsics(transform);
    //transform point cloud to camera 2 image plane through extrinsics and intrinsics
    const Eigen::Matrix<T, 2, N> pixels_img2  = (intrinsics_cam2_ * extrinsics * pc1).hnormalized();
    //TODO: check pixels_img2 are in img2
    //Get intensity in image 2
    const Eigen::Matrix<T, 2, img_height_ * img_width_> img2.intensity;
        (projection_matrix_.cast<T>() * input_point).hnormalized();

    Eigen::Map<const Eigen::Matrix<T, 4, 1>> 
    // Reprojection error is the distance from the reprojection to the observed
    // feature location.
    reprojection_error[0] = feature_[0] - reprojected_pixel[0];
    reprojection_error[1] = feature_[1] - reprojected_pixel[1];
    return true;
  }

 private:
    const dvo::PointCloud& pc1_;
    Eigen::VectorXd& img1_intensity_;
    const RGBDImage& img2_; 
    int img_width_;
    int img_height_;
    Matrix3T intrinsics_cam2_;
    MatrixXT img2_intensity_;
    MatrixXT  img2_depth_;
};


// 代价函数的计算模型
/* struct PHOTOMETRIC_COST_MatrixMultiply
{
    PHOTOMETRIC_COST ( const RGBDImage& x, const RGBDImage& y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const transform,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - x.warpIntensity(); // y-exp(ax^2+bx+c)
        return true;
    }
    const RGBDImage& _x, _y;    // x,y数据
};

struct PHOTOMETRIC_COST_Eigen
{
    PHOTOMETRIC_COST ( const RGBDImage& x, const RGBDImage& y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const transform,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - x.warpIntensity(); // y-exp(ax^2+bx+c)
        return true;
    }
    const RGBDImage& _x, _y;    // x,y数据
}; */

Eigen::Matrix4d FrontendSolver::solve(const dvo::RgbdImage& img1, const dvo::RgbdImage& img2, Eigen::Matrix initial_guess)
{   
    img1.buildPointCloud();
    img1.selectFeaturePoints(); // (place holder) build feature points function
    
    template<typename T>
    Eigen::Matrix<T, 1, Eigen::Dynamic> feature_points_idx = img1.getFeaturePointsIdx(); // place holder methods
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
    
    problem.AddResidualBlock (     // 向问题中添加误差项
    // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
        new ceres::AutoDiffCostFunction<PHOTOMETRIC_COST, 1, 12> ( 
            new PHOTOMETRIC_COST ( x_data[i], y_data[i] )
        ),
        nullptr,            // 核函数，这里不使用，为空
        abc                 // 待估计参数
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
    cout<<"estimated a,b,c = ";
    for ( auto a:abc ) cout<<a<<" ";
    cout<<endl; 
}
