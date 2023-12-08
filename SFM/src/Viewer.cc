/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: Viewer.cc
 *
 *          Created On: Wed 04 Sep 2019 06:59:03 PM CST
 *     Licensed under The GPLv3 License [see LICENSE for details]
 *
 ************************************************************************/
/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <unistd.h>
#include <mutex>

namespace ORB_SLAM2 {

Viewer::Viewer(System* system, const std::string& string_setting_file) 
  : system_(system),
    exit_required_(false)
{
  frame_drawer_ = new FrameDrawer(system_->map_);
  map_drawer_ = new MapDrawer(system_->map_, string_setting_file);
  tracker_ = system_->tracker_;
  
  tracker_->SetViewer(this);
  tracker_->SetFrameDrawer(frame_drawer_);
  tracker_->SetMapDrawer(map_drawer_);

  cv::FileStorage fSettings(string_setting_file, cv::FileStorage::READ);
  float fps = fSettings["Camera.fps"];

  if (fps < 1) fps = 30;
  T_ = 1e3 / fps;

  // img_width_ = fSettings["Camera.width"];
  // img_height_ = fSettings["Camera.height"];
  // if (img_width_ < 1 || img_height_ < 1) {
  //   img_width_ = 640;
  //   img_height_ = 480;
  // }

  view_point_x_ = fSettings["Viewer.ViewpointX"];
  view_point_y_ = fSettings["Viewer.ViewpointY"];
  view_point_z_ = fSettings["Viewer.ViewpointZ"];
  view_point_f_ = fSettings["Viewer.ViewpointF"];

  spdlog::info("Viewer created");
}

Viewer::Viewer(System* system, const int view_width, const int view_height)
  : system_(system), 
    exit_required_(false),
    view_width_(view_width),
    view_height_(view_height)
{
  frame_drawer_ = new FrameDrawer(system_->map_);
  map_drawer_ = new MapDrawer(system_->map_);
  tracker_ = system_->tracker_;
  
  tracker_->SetViewer(this);
  tracker_->SetFrameDrawer(frame_drawer_);
  tracker_->SetMapDrawer(map_drawer_);

  T_ = 1e3 / 30;

  view_point_x_ = 0;
  view_point_y_ = -0.7;
  view_point_z_ = -1.8;
  view_point_f_ = 500;

  spdlog::info("Viewer created");
}

// Viewer::Viewer(System* system, FrameDrawer* frame_drawer,
//                MapDrawer* map_drawer, Tracking* tracker,
//                const string& string_setting_file)
//     : system_(system),
//       frame_drawer_(frame_drawer),
//       map_drawer_(map_drawer),
//       tracker_(tracker),
//       is_finish_requested_(false),
//       is_finished_(true),
//       is_follow_(true),
//       is_stopped_(true),
//       is_stop_requested_(false) {
//   cv::FileStorage fSettings(string_setting_file, cv::FileStorage::READ);
//   float fps = fSettings["Camera.fps"];

//   if (fps < 1) fps = 30;
//   T_ = 1e3 / fps;

//   img_width_ = fSettings["Camera.width"];
//   img_height_ = fSettings["Camera.height"];
//   if (img_width_ < 1 || img_height_ < 1) {
//     img_width_ = 640;
//     img_height_ = 480;
//   }

//   view_point_x_ = fSettings["Viewer.ViewpointX"];
//   view_point_y_ = fSettings["Viewer.ViewpointY"];
//   view_point_z_ = fSettings["Viewer.ViewpointZ"];
//   view_point_f_ = fSettings["Viewer.ViewpointF"];
// }

void Viewer::Run(bool off_screen) {
  auto mcs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
  int64_t msct = mcs.count();
  // to string
  std::string mscts = std::to_string(msct);
  std::string map_title = "map_" + mscts;
  std::string frame_title = "frame_" + mscts;

  auto params = off_screen ? pangolin::Params({{"scheme", "headless"}}) : pangolin::Params();
  pangolin::CreateWindowAndBind(map_title, view_width_, view_height_, params);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // // Issue specific OpenGl we might need
  if( !off_screen ){
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(view_width_, view_height_, view_point_f_, view_point_f_, view_width_ / 2,
                                 view_height_ / 2, 0.1, 1000),
      pangolin::ModelViewLookAt(view_point_x_, view_point_y_, view_point_z_, 0,
                                0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0,
                                         1.0, -((float)view_width_) / ((float)view_height_))
                              .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  // create a frame buffer object with colour and depth buffer
  pangolin::GlTexture color_buffer(view_width_ * (int)off_screen, view_height_ * (int)off_screen);
  pangolin::GlRenderBuffer depth_buffer(view_width_ * (int)off_screen, view_height_ * (int)off_screen);
  pangolin::GlFramebuffer fbo_buffer(color_buffer, depth_buffer);
  if(off_screen) fbo_buffer.Bind();

  while(!exit_required_) 
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    map_drawer_->GetCurrentOpenGLCameraMatrix(Twc);
    s_cam.SetModelViewMatrix(
          pangolin::ModelViewLookAt(view_point_x_, view_point_y_, view_point_z_,
                                    0, 0, 0, 0.0, -1.0, 0.0));
    s_cam.Follow(Twc);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    map_drawer_->DrawCurrentCamera(Twc);
    map_drawer_->DrawKeyFrames(true, false);
    map_drawer_->DrawMapPoints();

    pangolin::FinishFrame();

    {
      std::lock_guard<std::mutex> lock(mutex_map_);
      glFlush();
      pangolin::TypedImage buffer = pangolin::ReadFramebuffer(d_cam.v, "RGBA32");
      cv::Mat matbuf = cv::Mat(buffer.h, buffer.w, CV_8UC4, buffer.ptr);

      if(!matbuf.empty()) {
        matbuf.copyTo(frame_map_);
        cv::cvtColor(frame_map_, frame_map_, CV_BGRA2RGB);
        cv::flip(frame_map_, frame_map_, 0);
      } 
    }

    if(!off_screen){
      cv::Mat frame = frame_drawer_->DrawFrame();
      cv::imshow(frame_title, frame);
      cv::waitKey(5);
    }  
  }

  if(off_screen) fbo_buffer.Unbind();

  cv::destroyAllWindows();
  pangolin::DestroyWindow(map_title);
  pangolin::QuitAll();

  spdlog::info("viewer thread released");
}

cv::Mat Viewer::GetMap() {
  std::lock_guard<std::mutex> lock(mutex_map_);
  return frame_map_;
}

cv::Mat Viewer::GetOrb() {
  std::lock_guard<std::mutex> lock(mutex_orb_);
  frame_orb_ = frame_drawer_->DrawFrame();
  return frame_orb_;
}

}  // namespace ORB_SLAM2
