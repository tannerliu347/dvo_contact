#include "rgbd_image.h"
#include "TUM_loader.h"
#include "yaml-cpp/yaml.h"

#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <iostream>
#include <algorithm>

void TUM_loader_test() {
    dvo::TUMLoader tum_loader("/home/tannerliu/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/");
    auto cur = tum_loader.getNext();
    while (tum_loader.hasNext()) {
        std::cout << cur.second << std::endl;
        cv::waitKey(10);
    }
}


// test buildPointcloud
void rgbd_camera_test(std::string file_path) {
    dvo::TUMLoader tum_loader(file_path);
    auto curImgs = tum_loader.getNext().first;
    for (int i = 0; i < 700; i++)
        curImgs = tum_loader.getNext().first;
    cv::imshow("gray", curImgs[0]);
    cv::imshow("meh", curImgs[1]);
    cv::waitKey(0);
    // float maxInten = 0.0;
    // int maxI = 0, maxJ = 0;
    // for (int i = 0; i < curImgs[1].rows; i++)
    //     for (int j = 0; j < curImgs[1].cols; j++) {
    //         // std::cout << curImgs[1].at<float>(i, j) << std::endl;
    //         if (curImgs[1].at<float>(i, j) > maxInten) {
    //             maxI = i;
    //             maxJ = j;
    //             maxInten = curImgs[1].at<float>(i, j);
    //         }
    //         // maxInten = std::max(maxInten, curImgs[1].at<float>(i, j));
    //     }
    // std::cout << maxInten << "i: " << maxI << " j: " << maxJ << std::endl;
    dvo::Intrinsic intrins = tum_loader.getIntrinsic();
    // std::cout << "Camera fx:\n";
    // std::cout << intrins.fx() << std::endl;
    // build pc
    dvo::RgbdCamera rCam(640, 480, intrins);
    // dvo::PointCloud pc;
    // rCam.buildPointCloud(curImgs[1], pc);
    // for (int i = 153600; i < 153620; i++) {
    //     Eigen::Matrix<float, 4, 1> point = pc.col(i);
    //     std::cout << point << std::endl;
    //     std::cout << "======\n";
    // } 
    dvo::RgbdImagePtr imgPtr = rCam.create(curImgs[0], curImgs[1]);
    imgPtr->buildPointCloud();
    dvo::PointCloud pc = imgPtr->point_cloud;
    // visualize this pointcloud
    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,320,0.2,100),
            pangolin::ModelViewLookAt(2,0,2, 0,0,0, pangolin::AxisY)
    );

    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glBegin( GL_POINTS );
        glColor3f(1.0,1.0,1.0);
        std::cout << pc.cols() << std::endl;
        for (int i = 0; i < 640 * 480; i++) {
            Eigen::Vector4f point = pc.col(i);
            glVertex3f(point(0), point(1), point(2));
        } 
        glEnd();
        pangolin::FinishFrame();
    }
}

void pyramid_test(std::string file_path) {
    dvo::TUMLoader tum_loader("/home/tannerliu/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/");
    auto curImgs = tum_loader.getNext();
    dvo::Intrinsic intrins = tum_loader.getIntrinsic();
    dvo::RgbdCamera rCam(640, 480, intrins);
    dvo::CameraPyramid camPyr(rCam);
    camPyr.build(5);
    dvo::ImagePyramidPtr img_pyramid = camPyr.create(curImgs.first[0], curImgs.first[1]);
    img_pyramid->build(5);
    for (int i = 0; i < 5; i++) {
        dvo::RgbdImage pyrimg = img_pyramid->level(i);
        cv::Mat inten = pyrimg.intensity;
        cv::Mat depth = pyrimg.depth;
        cv::imshow("gray", inten);
        cv::imshow("dept", depth);
        cv::waitKey(0);
    }
}

void create_test() {
    cv::Mat img(cv::Size(640, 480),5);
    
    // cv::Mat a,b,c,d;
    typedef std::shared_ptr<cv::Mat> mat_ptr;
    std::vector<mat_ptr> mat_vec;
    mat_vec.push_back(std::make_shared<cv::Mat>());
    int w = 640;
    int h = 480;
    for (int i = 0; i < 1; i++) {
        w /= 2;
        h /= 2;
        mat_ptr cur = mat_vec[i];
        cv::Mat& in  = img;
        cur->create(cv::Size(in.cols, in.rows), in.type());
        // b.create(cv::Size(w, h), 2);
        // c.create(cv::Size(w, h), 2);
        // d.create(cv::Size(w, h), 2);
        // w /= 2;
        // h /= 2;
    }
}


int main() {
    char resolved_path[PATH_MAX];
    char* tmp = realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_setting = YAML::LoadFile(std::string(resolved_path) + "/config/config.yaml");
    
    std::string image_load_path = config_setting["dvo"]["image_load_path"] ? config_setting["dvo"]["image_load_path"].as<std::string>() 
                                                                : "/home/tingjun/code/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/";
    // TUM_loader_test();
    // rgbd_camera_test(image_load_path);
    pyramid_test(image_load_path);
    return 0;
}
