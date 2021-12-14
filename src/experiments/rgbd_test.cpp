#include "rgbd_image.h"
#include "TUM_loader.h"
#include "yaml-cpp/yaml.h"
#include "point_selection.h"
#include "solver.h"

#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <iostream>
#include <algorithm>
#include <iomanip>

void TUM_loader_test() {
    dvo::TUMLoader tum_loader("/home/tannerliu/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/");
    int i = 0;
    while (tum_loader.hasNext()) {
        std::vector<cv::Mat> cur_img = tum_loader.getImgs();
        dvo::AffineTransform T = tum_loader.getPose();
        float ts = tum_loader.getTimestamp();
        std::cout << "=====================\n";
        std::cout << "Timestamp: " << ts << std::endl;
        std::cout << "Rotation: " << T.rotation() << std::endl;
        std::cout << "Translation: " << T.translation() << std::endl;
        std::cout << i << std::endl;
        std::cout << "=====================\n";
        cv::imshow("gray", cur_img[0]);
        cv::imshow("dept", cur_img[1]);
        cv::waitKey(10);
        tum_loader.step();
        i++;
    }
    std::cout << i << std::endl;
    std::cout << tum_loader.teleportToFrame(100) << std::endl;
    auto imgs = tum_loader.getImgs();
}


// test buildPointcloud
void rgbd_camera_test(std::string file_path) {
    dvo::TUMLoader tum_loader(file_path);
    auto curImgs = tum_loader.getImgs();
    for (int i = 0; i < 700; i++)
        curImgs = tum_loader.getImgs();
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
    auto curImgs = tum_loader.getImgs();
    dvo::Intrinsic intrins = tum_loader.getIntrinsic();
    dvo::RgbdCamera rCam(640, 480, intrins);
    dvo::CameraPyramid camPyr(rCam);
    camPyr.build(5);
    dvo::ImagePyramidPtr img_pyramid = camPyr.create(curImgs[0], curImgs[1]);
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

void warp_test(std::string file_path) {
    dvo::TUMLoader tum_loader("/home/tannerliu/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/");
    auto curImgs = tum_loader.getImgs();
    Eigen::Quaternionf quat_ref(-0.3986, 0.6132, 0.5962, -0.3311);
    Eigen::Translation3f tran_ref(1.3563, 0.6305, 1.6380);
    Eigen::Affine3f af_ref = tran_ref * quat_ref.toRotationMatrix();
    Eigen::Quaternionf quat_cur(-0.3980, 0.6129, 0.5966, -0.3316);
    Eigen::Translation3f tran_cur(1.3543, 0.6306, 1.6360);
    Eigen::Affine3f af_cur = tran_cur * quat_cur.toRotationMatrix();
    dvo::AffineTransform T = af_ref.inverse() * af_cur;
    
    dvo::Intrinsic intrins = tum_loader.getIntrinsic();
    dvo::RgbdCamera rCam(640, 480, intrins);
    dvo::RgbdImagePtr imgPtr = rCam.create(curImgs[0], curImgs[1]);
    imgPtr->buildPointCloud();
    dvo::PointCloud pc = imgPtr->point_cloud;
    dvo::PointCloud transformed_pc;
    dvo::RgbdImage result_image(rCam);
    imgPtr->warpIntensity(T, pc, intrins, result_image, transformed_pc);
    // visualize this pointcloud
    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,320,0.2,100),
            pangolin::ModelViewLookAt(2,0,2, 0,0,0, pangolin::AxisY)
    );
    pangolin::OpenGlRenderState s_cam2(
            pangolin::ProjectionMatrix(640,480,420,420,320,320,0.2,100),
            pangolin::ModelViewLookAt(2,0,2, 0,0,0, pangolin::AxisY)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View& d_cam2 = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam2));

    pangolin::Display("multi")
            .SetBounds(0.0, 1.0, 0.0, 1.0)
            .SetLayout(pangolin::LayoutEqual)
            .AddDisplay(d_cam)
            .AddDisplay(d_cam2);

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

        d_cam2.Activate(s_cam2);
        glBegin( GL_POINTS );
        glColor3f(1.0,1.0,1.0);
        std::cout << transformed_pc.cols() << std::endl;
        for (int i = 0; i < 640 * 480; i++) {
            Eigen::Vector4f point = transformed_pc.col(i);
            glVertex3f(point(0), point(1), point(2));
        } 
        glEnd();
        pangolin::FinishFrame();
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

void pt_selection_test(YAML::Node config_setting) {
  std::string image_load_path = config_setting["dvo"]["image_load_path"].as<std::string>();
  dvo::TUMLoader tum_loader(image_load_path);
                               
  // load ref image
  std::cout << "loading ref image" << std::endl;
  auto curImgs_1 = tum_loader.getImgs();
  dvo::Intrinsic intrins_1 = tum_loader.getIntrinsic();
  dvo::RgbdCamera rCam_1(640, 480, intrins_1);
  // dvo::RgbdImagePtr imgPtr_1 = rCam_1.create(curImgs_1[0], curImgs_1[1]);
  dvo::CameraPyramid cam_pyr_1(rCam_1);

  cv::imshow("gray", curImgs_1[0]);
  cv::imshow("dept", curImgs_1[1]);
  cv::waitKey(0);

  // load cur image
  std::cout << "loading cur image" << std::endl;
  tum_loader.step();
  auto curImgs_2 = tum_loader.getImgs();
  dvo::Intrinsic intrins_2 = tum_loader.getIntrinsic();
  dvo::RgbdCamera rCam_2(640, 480, intrins_2);
  // dvo::RgbdImagePtr imgPtr_2 = rCam_2.create(curImgs_2[0], curImgs_2[1]);
  dvo::CameraPyramid cam_pyr_2(rCam_2);

  size_t level = 0;

  // Set Image pyramid coefficients:
  dvo::ImagePyramid reference(cam_pyr_1, curImgs_1[0], curImgs_1[1]);
  dvo::ImagePyramid current(cam_pyr_2, curImgs_2[0], curImgs_2[1]);

  // Eigen::Affine3d transformation;
  dvo::PtAndGradVerify selection_verify;
  selection_verify.intensity_threshold = 10.0f;
  selection_verify.depth_threshold = 10.0f;

  dvo::PtSelection reference_selection(reference, selection_verify);
  
  
  std::cout << "Setting Pyram" << std::endl;
  reference_selection.setImagePyramid(reference);
  reference_selection.getImagePyramid();

  std::cout << "Got reference_selection image pyramid" << std::endl;

  bool success = true;

  dvo::PtSelection::PtIterator first_point, last_point;
  std::cout << "Selecting points from reference level" << std::endl;
  reference_selection.select(level, first_point, last_point);
  std::cout << "Finished selection" << std::endl;

  std::cout << "Loop through the iterators" << std::endl;
  // for (auto it = first_point; it != last_point; it++) {
  //   std::cout << "x =  " << it->x << ", y = " << it->y << ", z = " << it->z << std::endl;
  // }

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
      // std::cout << pc.cols() << std::endl;
      for (auto it = first_point; it != last_point; it++) {
          // Eigen::Vector4f point = pc.col(i);
          glVertex3f(it->x, it->y, it->z);
      } 
      glEnd();
      pangolin::FinishFrame();
  }

  std::cout << "Finished loop" << std::endl;

}

void solver_test() {
    dvo::TUMLoader tum_loader("/home/tannerliu/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/");
    // tum_loader.teleportToFrame();
    // save to trajectory
    std::ofstream out_file("/home/tannerliu/dvo_contact/trajectory.txt");

    // solver
    dvo::Solver g2o_solver;
    
    char resolved_path[PATH_MAX];
    char* tmp = realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_setting = YAML::LoadFile(std::string(resolved_path) + "/config/config.yaml");

    size_t cam_width = config_setting["dvo"]["image_width"].as<size_t>();
    size_t cam_height = config_setting["dvo"]["image_height"].as<size_t>();
    // need cam_width and cam_height from loader

    dvo::Intrinsic cam_intrinsic = tum_loader.getIntrinsic(); // did not see cam_calib_ in other parts of the code

    dvo::RgbdCamera camera(cam_width, cam_height, cam_intrinsic);
    dvo::CameraPyramid cam_pyramid(camera);

    cv::Mat previous_intensity, current_intensity;
    cv::Mat previous_depth, current_depth;
    
    std::vector<cv::Mat> previous_img(2);
    std::vector<cv::Mat> current_img(2);

    previous_img = tum_loader.getImgs();
    previous_intensity = previous_img[0];
    previous_depth = previous_img[1];

    dvo::PtAndGradVerify pt_verifier;
    pt_verifier.intensity_threshold = config_setting["dvo"]["intensity_threshold"].as<float>();
    pt_verifier.depth_threshold = config_setting["dvo"]["depth_threshold"].as<float>();

    int i = 0;

    dvo::AffineTransform gt = tum_loader.getPose();
    Eigen::Isometry3d gt_start_pose;
    gt_start_pose.translation() = gt.translation().cast<double>();
    gt_start_pose.linear() = gt.rotation().cast<double>();

    while (tum_loader.hasNext())
    {
        if (!tum_loader.step() || !tum_loader.hasNext())
            break;
        current_img = tum_loader.getImgs();
        current_intensity = current_img[0];
        current_depth = current_img[1];

        // dvo::RgbdImage current_rgbd(camera);
        // current_rgbd.intensity = current_intensity;
        // current_rgbd.depth = current_depth;
        // current_rgbd.initialize();
        
        dvo::ImagePyramid previous_pyramid(cam_pyramid, previous_intensity, previous_depth);
        // dvo::ImagePyramidPtr previous_pyramid = camPyr.create(current_intensity, current_depth);
        // dvo::ImagePyramidPtr 
        size_t pyramid_level = config_setting["dvo"]["pyramid_level"].as<size_t>();
        previous_pyramid.build(pyramid_level);
        
        dvo::PtSelection point_selecter(previous_pyramid, pt_verifier);
        Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
        for (int op_level = pyramid_level-1; op_level >= 0 ; op_level--) {
            dvo::PtSelection::PtIterator start_pointer, end_pointer;
            point_selecter.select(op_level, start_pointer, end_pointer);

            dvo::RgbdImage cur_lvl_img = point_selecter.getImagePyramid().level(op_level); //place holder, this is the img2 parameter prepared for solver
            
            g2o_solver.solve(start_pointer, end_pointer, cur_lvl_img, Tcw);
        }
        // std::cout << "=================================\n";
        // std::cout << "solver results\n";
        // std::cout << Tcw.translation() << std::endl;
        // std::cout << Tcw.rotation() << std::endl;
        // std::cout << "ground truth\n";
        // std::cout << gt_pose.translation() << std::endl;
        // std::cout << gt_pose.rotation() << std::endl;
        gt_start_pose = gt_start_pose * Tcw;
        out_file << std::fixed << std::setprecision(8) << tum_loader.getTimestamp();
        out_file << " " << gt_start_pose.translation()(0) << " " << gt_start_pose.translation()(1) 
                 << " " << gt_start_pose.translation()(2) << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << "\n";

        previous_intensity = current_intensity;
        previous_depth = current_depth;
        std::cout << i++ << std::endl;

    }
    out_file.close();
}


int main() {
    char resolved_path[PATH_MAX];
    char* tmp = realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_setting = YAML::LoadFile(std::string(resolved_path) + "/config/config.yaml");
    
    std::string image_load_path = config_setting["dvo"]["image_load_path"] ? config_setting["dvo"]["image_load_path"].as<std::string>() 
                                                                : "/home/tingjun/code/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/";
    // TUM_loader_test();
    solver_test();
    // rgbd_camera_test(image_load_path);

    // pt_selection_test(config_setting);
    // pyramid_test(image_load_path);
    // warp_test(image_load_path);
    return 0;
}
