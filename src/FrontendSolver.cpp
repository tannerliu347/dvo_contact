#include "FrontendSolver.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <chrono>

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
 public:
  PhotometricError(const dvo::PointCloud& pc_1, const RGBDImage& img2) : pc_1_ (pc_1), 
                                                                           img2_ (img2),
                                                                           img_width_(img2.width),
                                                                           img_height_(img2.height) {} 

  template <typename T>
  bool operator()(const T* transform, T* residual) const {
    Eigen::Map<const Eigen::Matrix<T, 4, img_height_ * img_width_>> pc_1_eigen(pc_1);
    
    const Eigen::Matrix<T, 2, img_height_ * img_width_> img2.intensity;
        (projection_matrix_.cast<T>() * input_point).hnormalized();


    // Reprojection error is the distance from the reprojection to the observed
    // feature location.
    reprojection_error[0] = feature_[0] - reprojected_pixel[0];
    reprojection_error[1] = feature_[1] - reprojected_pixel[1];
    return true;
  }

 private:
    const dvo::PointCloud& pc_1_;
    const RGBDImage& img2_; 
    int img_width_;
    int img_height_;
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

Eigen::Matrix4d FrontendSolver::solve(const RGBDImage& img1, const RGBDImage& img2, Eigen::Matrix initial_guess)
{
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
