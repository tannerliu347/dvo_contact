#pragma once

#include "rgbd_image.h"
#include "point_selection.h"

#include <opencv2/opencv.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace dvo {

// Define the edge/residual in VO problem; Photometric error in our case
class EdgeSE3ProjectDirect: public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect(Eigen::Vector3f ref_point, dvo::Intrinsic intrins, const cv::Mat& cur_image)
                                                : point_world_(ref_point),
                                                  intrinsics_(intrins),
                                                  cur_image_(cur_image) {}
                                            
    void computeError() {
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // transform the point to next frame's local frame?
        Eigen::Vector3d x_local = v->estimate().map(point_world_.cast<double>());
        // project to next frame's image plane
        float x = x_local(0) * intrinsics_.fx() / x_local(2) + intrinsics_.ox();
        float y = x_local(1) * intrinsics_.fy() / x_local(2) + intrinsics_.oy();
        // check x,y is in the image
        if (x - 4 < 0 || (x + 4) > cur_image_.cols || (y - 4) < 0 || (y + 4) > cur_image_.rows) {
            _error(0, 0) = 0.0;
            this->setLevel(1);
        } else {
            _error (0, 0) = getPixelValue(x, y) - _measurement;
        }
    }

    void linearizeOplus() {
        if (level() == 1) {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }
        g2o::VertexSE3Expmap* vtx = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vtx->estimate().map (point_world_.cast<double>());

        double x = xyz_trans(0);
        double y = xyz_trans(1);
        double invz = 1.0 / xyz_trans(2);
        double invz_2 = invz * invz;

        float u = x * intrinsics_.fx() * invz + intrinsics_.ox();
        float v = y*intrinsics_.fy() * invz + intrinsics_.oy();

        // jacobian from se3 to u,v
        // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai(0, 0) = -x * y * invz_2 * intrinsics_.fx();
        jacobian_uv_ksai(0, 1) = (1 + (x * x * invz_2)) * intrinsics_.fx();
        jacobian_uv_ksai(0, 2) = -y * invz * intrinsics_.fx();
        jacobian_uv_ksai(0, 3) = invz * intrinsics_.fx();
        jacobian_uv_ksai(0, 4) = 0;
        jacobian_uv_ksai(0, 5) = -x * invz_2 * intrinsics_.fx();

        jacobian_uv_ksai(1, 0) = -(1 + y * y * invz_2) * intrinsics_.fy() ;
        jacobian_uv_ksai(1, 1) = x * y * invz_2 * intrinsics_.fy() ;
        jacobian_uv_ksai(1, 2) = x * invz * intrinsics_.fy() ;
        jacobian_uv_ksai(1, 3) = 0;
        jacobian_uv_ksai(1, 4) = invz * intrinsics_.fy() ;
        jacobian_uv_ksai(1, 5) = -y * invz_2 * intrinsics_.fy() ;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv (0, 0) = (getPixelValue(u + 1, v) - getPixelValue(u - 1, v)) / 2;
        jacobian_pixel_uv (0, 1) = (getPixelValue(u, v + 1) - getPixelValue(u, v - 1)) / 2; 

        _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;
    }

    bool read(std::istream& in) {}

    bool write(std::ostream& out) const {}

private:
    inline float getPixelValue (float x, float y) {
        uchar* data = & cur_image_.data[ int ( y ) * cur_image_.step + int ( x ) ];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float (
                (1 - xx) * (1 - yy) * data[0] +
                xx * (1 - yy) * data[1] +
                (1 - xx) * yy * data[cur_image_.step] +
                xx * yy * data[cur_image_.step+1]
            );
    }


protected:
    Eigen::Vector3f point_world_;
    dvo::Intrinsic intrinsics_;
    cv::Mat cur_image_;
};

class Solver {
public:
    Solver() {}
    
    bool solve(const PtSelection::PtIterator start_it, const PtSelection::PtIterator end_it, const RgbdImage& img2, Eigen::Isometry3d& Tcw) {
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
        DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
        DirectBlock* solver_ptr = new DirectBlock(linearSolver);
        // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm (solver);
        optimizer.setVerbose(false);

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setEstimate(g2o::SE3Quat(Tcw.rotation().cast<double>(), Tcw.translation().cast<double>()));
        pose->setId(0);
        optimizer.addVertex(pose);

        int id = 1;
        const Intrinsic& intrins = img2.camera().intrinsics();
        for (auto it = start_it; it != end_it; it++) {
            Eigen::Vector3f ref_point;
            ref_point << it->x, it->y, it->z;
            const cv::Mat& cur_image = img2.intensity;
            EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(ref_point, intrins, cur_image);
            edge->setVertex(0, pose);
            edge->setMeasurement(it->i);
            edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
            edge->setId(id++);
            optimizer.addEdge(edge);
        }
        optimizer.initializeOptimization();
        optimizer.optimize(30);
        Tcw = pose->estimate();
        // std::cout << "solved\n";
        return true;
    }

};

} //namespace dvo
