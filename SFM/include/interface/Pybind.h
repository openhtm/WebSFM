#ifndef __PYBIND_H__
#define __PYBIND_H__

#include <thread>
#include <memory>
#include <queue>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "System.h"
#include "Osmap.h"

#include "interface/SceneMVS.h"

#include <spdlog/spdlog.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

class ImageStream{
public:
  ImageStream(bool force_realtime = false);
  // add new image to image queue
  void addNewImage(cv::Mat& im, double time_ms);
  // get new image from image queue
  bool getNewImage(cv::Mat& im, double& time);
  // release stream
  void release();

private:
  bool force_realtime_ = false;
  bool new_img_abailable_ = false;
  bool released_ = false;
  std::mutex img_mutex_;
  std::queue<cv::Mat> img_stream_;
  std::queue<double> img_times_;

};


class Session {
public:
  // constructor from config file
  Session(const std::string voc_file, const std::string config_file, 
    bool force_realtime = true, const std::string keyframe_dir = "");
  // fast constructor from image size
  Session(const std::string voc_file, const int imwidth, const int imheight,
    bool force_realtime = true, const std::string keyframe_dir = "");

public:
  // add new image frame
  void addTrack(py::array_t<uint8_t>& input, double time_ms = -1);
  // get feature points
  py::array_t<float> getFeaturePoints();
  // get current map
  py::array_t<uint8_t> getMapVisualFrame();
  // get current orb feature
  py::array_t<uint8_t> getOrbVisualFrame();
  // get tracking status
  py::array_t<size_t> getTrackingState();
  // get camera twc
  Eigen::Matrix4d getTwc();
  // get camera twc under openGL coordinate
  Eigen::Matrix4d getTwcGL();
  // set map save status
  void setSaveMap(bool save_map, const std::string map_name = "");
  // load map
  void loadMap(const std::string filename);
  // activate loc mode
  void activateLocalization();
  // deactivate loc mode
  void deactivateLocalization();
  // set mvs save status
  void setSaveScene(bool save_scene, const std::string scene_name);
  // enable viewer thread
  void enableViewer(bool off_screen = true, int view_width = 1024, int view_height = 768);
  // stop session
  void release();
  // cancel session
  void cancel();

private:
  // run thread
  void run();
  // get image from webrtc frame
  cv::Mat getImageBGR(py::array_t<uint8_t>& input);
  // remove redundant images
  void removeRedundant();

  bool released_ = false;
	bool visualize_ = false;
  bool exit_required_ = false;
  bool save_map_ = false;
  bool save_scene_ = false;
  bool save_pcd_ = false;
  bool loc_mode_ = false;
  
  std::string map_name_ = "";
  std::string scene_name_ = "";
  std::string pcd_name_ = "";

	std::shared_ptr<ORB_SLAM2::System> psystem_ = nullptr;
	std::shared_ptr<ORB_SLAM2::Viewer> pviewer_ = nullptr;
  std::shared_ptr<ImageStream> pstream_ = nullptr;
  std::shared_ptr<ORB_SLAM2::Osmap> posmap_ = nullptr;
  std::shared_ptr<MVS::Scene> pmvs_ = nullptr;

  std::thread viewer_thread_;
  std::thread system_thread_;

};


PYBIND11_MODULE(pysfm, m) {
  m.doc() = "SFM system wrapper modified from ORB-SLAM2 ";

  py::class_<Session>(m, "Session")
    .def(py::init<const std::string, const std::string, bool, const std::string>(), py::arg("voc_file"), py::arg("config_file"), py::arg("force_realtime") = false, py::arg("keyframe_dir") = "")
    .def(py::init<const std::string, const int, const int, bool, const std::string>(), py::arg("voc_file"), py::arg("imwidth"), py::arg("imheight"), py::arg("force_realtime") = false, py::arg("keyframe_dir") = "")
    .def("enable_viewer", &Session::enableViewer, py::arg("off_screen") = true, py::arg("view_width") = 1024, py::arg("view_height") = 768)
    .def("add_track", &Session::addTrack, py::arg("image"), py::arg("time_ms") = -1)
    .def("tracking_state", &Session::getTrackingState)
    .def("get_feature_points", &Session::getFeaturePoints)
    .def("get_position_cv", &Session::getTwc)
    .def("get_position_gl", &Session::getTwcGL)
    .def("get_map_visual", &Session::getMapVisualFrame)
    .def("get_orb_visual", &Session::getOrbVisualFrame)
    .def("save_map", &Session::setSaveMap, py::arg("save_map"), py::arg("map_name") = "")
    .def("save_mvs", &Session::setSaveScene, py::arg("save_scene"), py::arg("scene_name") = "")
    .def("load_map", &Session::loadMap, py::arg("filename"))
    .def("activate_localization", &Session::activateLocalization)
    .def("activate_mapping", &Session::deactivateLocalization)
    .def("release", &Session::release)
    .def("cancel", &Session::cancel);
}

#endif // __PYBIND_H__
