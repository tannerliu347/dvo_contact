#include "rgbd_image.h"
#include "TUM_loader.h"
#include "yaml-cpp/yaml.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <iostream>

int main() {
    char resolved_path[PATH_MAX];
    realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_setting = YAML::LoadFile(std::string(resolved_path) + "/config/config.yaml");
    
    std::string image_load_path = config_setting["dvo"]["image_load_path"] ? config_setting["dvo"]["image_load_path"].as<std::string>() 
                                                                : "/home/tingjun/code/dvo_contact/dataset/";

    dvo::TUMLoader tum_loader(image_load_path);
    // auto cur = tum_loader.getNext();
    while (tum_loader.hasNext()) {
        auto cur = tum_loader.getNext();
        cv::imshow("gray", cur.first[0]);
        cv::imshow("dept", cur.first[1]);
        std::cout << cur.second << std::endl;
        cv::waitKey(10);
    }
}

void rgbd_camera_test() {
    dvo::TUMLoader tum_loader("/home/tannerliu/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/");
    auto curImgs = tum_loader.getNext().first;
    dvo::Intrinsic intrins = tum_loader.getIntrinsic();
    std::cout << "Camera fx:\n";
    std::cout << intrins.fx() << std::endl;
    dvo::RgbdCamera rCam(640, 480, intrins);
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
        glBegin( GL_POINTS );//点设置的开始
        glColor3f(1.0,1.0,1.0);
        auto point = pc.data();
        std::cout << pc.cols() << std::endl;
        for (int i = 0; i < 640 * 480; i++, point++) {
            // auto point = pc.data();
            // std::cout << point[0] << ", " <<  point[1] << ", " << point[2] << std::endl;
            // glVertex3f(point[0], point[1], point[2]);
        } 
        glEnd();
        pangolin::FinishFrame();
    }
}


int main() {
    // TUM_loader_test();
    rgbd_camera_test();
    return 0;
}
