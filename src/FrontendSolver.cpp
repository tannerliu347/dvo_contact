#include "FrontendSolver.h"
#include <iostream>
#include <opencv2/core/core.hpp>
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
// 代价函数的计算模型
struct PHOTOMETRIC_COST_MATMUL
{
    PHOTOMETRIC_COST_MATMUL ( const dvo::PointCloud& pc_ref, const RGBDImage& img2 ) : pc_ref_ (pc_ref), img2_ (img2) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const transform,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - x.warpIntensity(); // y-exp(ax^2+bx+c)
        return true;
    }
    const dvo::PointCloud& pc_ref_;
    const RGBDImage& img2_;    // x,y数据
};

Eigen::Matrix4d FrontendSolver::solve(const RGBDImage& img1, const RGBDImage& img2, Eigen::Matrix initial_guess)
{
    // 构建最小二乘问题
    ceres::Problem problem;
    for ( int i=0; i<N; i++ )
    {
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<PHOTOMETRIC_COST, 1, 2> ( 
                new PHOTOMETRIC_COST ( x_data[i], y_data[i] )
            ),
            nullptr,            // 核函数，这里不使用，为空
            abc                 // 待估计参数
        );
    }

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
