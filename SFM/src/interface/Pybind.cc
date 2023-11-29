#include "interface/Pybind.h"

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <filesystem>
#include <set>

/************************************
 * Image stream for realtime process
 ************************************/
ImageStream::ImageStream(bool force_realtime)
 : force_realtime_(force_realtime)
{
  // constructor
}

// add new image to image queue
void ImageStream::addNewImage(cv::Mat& im, double time_ms) {
  if(released_) return;

  std::lock_guard<std::mutex> lock(img_mutex_);
  img_stream_.push(im);
  img_times_.push(time_ms);

  new_img_abailable_ = true;
}

// get new image from image queue
bool ImageStream::getNewImage(cv::Mat& im, double& time) {
  if(released_) return false;

  std::lock_guard<std::mutex> lock(img_mutex_);
  if(!new_img_abailable_) return false;

  do {
      im = img_stream_.front();
      img_stream_.pop();

      time = img_times_.front();
      img_times_.pop();

      if( !force_realtime_ ) break; // check force_realtime to skip frames

  } while( !img_stream_.empty() );

  new_img_abailable_ = !img_stream_.empty();

  return true;
}

// release 
void ImageStream::release() {
  released_ = true;
}


/************************************
 * Pybind interface for slam session
 ************************************/
// constructor
Session::Session(const std::string voc_file, const std::string config_file, 
  bool force_realtime, const std::string keyframe_dir) 
{
  psystem_.reset(new ORB_SLAM2::System(voc_file,config_file));
  pstream_.reset(new ImageStream(force_realtime));
  posmap_.reset(new ORB_SLAM2::Osmap(*psystem_));
  system_thread_ = std::thread(&Session::run, this);
}

// constructor
Session::Session(const std::string voc_file, const int imwidth, const int imheight, 
  bool force_realtime, const std::string keyframe_dir) 
{
  psystem_.reset(new ORB_SLAM2::System(voc_file, imwidth, imheight, keyframe_dir));
  pstream_.reset(new ImageStream(force_realtime));
  posmap_.reset(new ORB_SLAM2::Osmap(*psystem_));
  pmvs_.reset(new MVS::Scene(*psystem_));
  system_thread_ = std::thread(&Session::run, this);
}

// add track image
void Session::addTrack(py::array_t<uint8_t>& input, double time_ms){
  if(released_) return;

  cv::Mat image = getImageBGR(input).clone();
  // cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

  if(time_ms < 0) {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    time_ms = (double)ms.count();
  }

  pstream_->addNewImage(image, time_ms);
}

// get current map
py::array_t<uint8_t> Session::getMapVisualFrame() {
  if(released_) 
    return py::array_t<uint8_t>();

  if(!visualize_) {
    puts("Viewer not enabled!");
    return py::array_t<uint8_t>();
  }

  cv::Mat frame = pviewer_->GetMap();
  return py::array_t<uint8_t>({frame.rows, frame.cols, 3}, frame.data);
}

// get current orb features
py::array_t<uint8_t> Session::getOrbVisualFrame() {
  if(released_) 
    return py::array_t<uint8_t>();

  if(!visualize_) {
    puts("Viewer not enabled!");
    return py::array_t<uint8_t>();
  }

  cv::Mat frame = pviewer_->GetOrb();
  return py::array_t<uint8_t>({frame.rows, frame.cols, 3}, frame.data);
}

// get tracking status
py::array_t<size_t> Session::getTrackingState() {
  size_t data[3] = {0,0,0};

  if(!released_) {
    data[0] = psystem_->GetTrackingState();
    data[1] = psystem_->map_->KeyFramesInMap();
    data[2] = psystem_->map_->MapPointsInMap();
  }
  
  return py::array_t<size_t>({3}, data);
}

// get camera twc
Eigen::Matrix4d Session::getCameraPoseMatrix() {
  if(released_) return Eigen::Matrix4d::Identity();

  Eigen::Matrix4d Twc = psystem_->GetTwc();
  return Twc;
}

// set map save status
void Session::setSaveMap(bool save_map, const std::string map_name) {
  if(released_) return;

  if(save_map && map_name.length() > 0) {
    save_map_ = true;
    map_name_ = map_name;
    spdlog::info("Map {} will be saved when sytem stop.", map_name);
  } else {
    save_map_ = false;
  }
}

// set map mvs scene status
void Session::setSaveScene(bool save_scene, const std::string scene_name) {
  if(released_) return;

  if(save_scene && scene_name.length() > 0 && pmvs_ != nullptr) {
    save_scene_ = true;
    scene_name_ = scene_name;
    spdlog::info("Scene {} will be saved when sytem stop.", scene_name_);
  } else {
    save_scene_ = false;
  }
}

// load map
void Session::loadMap(bool track_only, const std::string filename){
  if(released_) return;

  posmap_->mapLoad(filename);
  spdlog::info("Map loaded from {}", filename);

  if(track_only) {
    psystem_->ActivateLocalizationMode();
    spdlog::info("Localization mode activated");
  }
}

// enable viewer thread
void Session::enableViewer(bool off_screen) {
  pviewer_.reset(new ORB_SLAM2::Viewer(psystem_.get()));
  viewer_thread_ = std::thread(&ORB_SLAM2::Viewer::Run, pviewer_, off_screen);
  visualize_ = true;
}

// stop session
void Session::release() {
  if(released_) return;
  pstream_->release();
  // stop viewer if enabled
  if(pviewer_ != nullptr)
    pviewer_->exit_required_ = true;
  
  exit_required_ = true;
  viewer_thread_.join();

  psystem_->Shutdown();
  system_thread_.join();
  removeRedundant();
  
  if(save_map_) {
    posmap_->options.set(ORB_SLAM2::Osmap::NO_FEATURES_DESCRIPTORS | ORB_SLAM2::Osmap::ONLY_MAPPOINTS_FEATURES, 1); 
    posmap_->mapSave(map_name_);
    spdlog::info("Map {} saved", map_name_);
  }

  if(save_scene_) {
    pmvs_->serialize(scene_name_);
  }

  psystem_.reset();
  pviewer_.reset();
  pstream_.reset();
  pmvs_.reset();

  visualize_ = false;

  cv::destroyAllWindows();

  released_ = true;
  puts("Session Released");
}

// cancel session
void Session::cancel() {
  save_map_ = false;
  release();
}

// run slam thread
void Session::run() {
  cv::Mat img;
  double time;
  while( !exit_required_ ) {
    if(pstream_->getNewImage(img, time)) {
      psystem_->TrackMonocular(img, time);
    } else 
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 60));
  }
  spdlog::info("Stop Processing New Frame");
}

// get image from webrtc frame
cv::Mat Session::getImageBGR(py::array_t<uint8_t>& input) {
  if(input.ndim() != 3) 
    throw std::runtime_error("get Image : number of dimensions must be 3");
  py::buffer_info buf = input.request();
  cv::Mat image(buf.shape[0], buf.shape[1], CV_8UC3, (uint8_t*)buf.ptr);
  return image;
}

void Session::removeRedundant() {
  if(released_)
    return;
  // remove redundant image files
  const std::string dir = psystem_->keyframe_dir_;

  auto keyframes = psystem_->map_->GetAllKeyFrames();
  std::set<unsigned long> kfids;
  for(auto& pKF : keyframes)
    kfids.insert(pKF->id_);

  size_t redundant_cnt = 0;
  for(auto& p : std::filesystem::directory_iterator(dir)) {
    std::string filename = p.path().filename().string();
    if(filename.find(".png") == std::string::npos)
      continue;
    
    uint32_t kfid = std::stoi(filename.substr(0, filename.find(".png")));
    if(kfids.find(kfid) == kfids.end()) {
      std::filesystem::remove(p.path());
      ++redundant_cnt;
    }
  }

  spdlog::info("remove {} redundant images", redundant_cnt);
}

  

