#include "interface/SceneMVS.h"
#include "opencv2/core/eigen.hpp"

#include <spdlog/spdlog.h>

namespace MVS {

Scene::Scene(ORB_SLAM2::System& system) 
  : system_(system), map_(*system.map_)
{
  // initialize  
}

void Scene::serialize(const std::string& filename) {
  // bind keyframe id
  bindKeyframeID();
  // set platform
  definePlatform();
  // set image and pose
  defineImagePose();
  // set map point
  defineStructure();

  // serialize
  bool res = _INTERFACE_NAMESPACE::ARCHIVE::SerializeSave(scene_, filename);

  if(res) spdlog::info("MVS: scene serialized to {}", filename);
  else    spdlog::error("MVS: scene serialization failed");
}

void Scene::bindKeyframeID() {
  auto keyframes = map_.GetAllKeyFrames();
  uint32_t cnt = 0;
  for(auto& pKF : keyframes) {
    if(pKF->isBad()) continue;
      kfid_map_[pKF->id_] = cnt++;
  }
}

uint32_t Scene::getBindedID(unsigned long kfid) {
  if (kfid_map_.find(kfid) == kfid_map_.end()) 
    return (uint32_t)-1;
  return kfid_map_[kfid];
}

void Scene::definePlatform() {
  // platform
  _INTERFACE_NAMESPACE::Interface::Platform platform;
  // camera
  _INTERFACE_NAMESPACE::Interface::Platform::Camera camera;
  
  // origin data
  cv::Size& imsize = system_.tracker_->imsize_;
  cv::Mat K = system_.tracker_->K_.clone();
  K.convertTo(K, CV_64F);

  // set camera
  camera.width = imsize.width;
  camera.height = imsize.height;
  camera.K = K;

  // sub-pose
  camera.R = cv::Matx33d::eye();
  camera.C = cv::Point3d(0, 0, 0);
  platform.cameras.emplace_back(camera);

  // push to scene
  scene_.platforms.emplace_back(platform);

  spdlog::info("MVS: platform defined");
}

void Scene::defineImagePose() {
  auto& platform = scene_.platforms[0];
  const std::string base_dir = system_.tracker_->keyframe_dir_;

  auto keyframes = map_.GetAllKeyFrames();
  size_t n_views = keyframes.size();
  
  scene_.images.reserve(n_views);
  platform.poses.reserve(n_views);

  for(auto& pKF : keyframes) {
    if(pKF->isBad())
      continue;
    
    _INTERFACE_NAMESPACE::Interface::Image image;
    // image source
    image.ID = getBindedID(pKF->id_);
    image.name = base_dir + std::to_string(pKF->id_) + ".png";
    // camera
    image.platformID = 0;
    image.cameraID = 0;
    // pose
    _INTERFACE_NAMESPACE::Interface::Platform::Pose pose;
    image.poseID = platform.poses.size();
    // rotation
    auto rcw = pKF->GetRotation();
    // Eigen::Matrix3d rwc = rcw.transpose();
    cv::eigen2cv(rcw, pose.R);
    // center (translation)
    auto t = pKF->GetCameraCenter();
    pose.C = cv::Point3d(t(0), t(1), t(2));

    platform.poses.push_back(pose);
    scene_.images.emplace_back(image);
  }

  spdlog::info("MVS: {} images and poses defined", n_views);
}

void Scene::defineStructure() {
  auto map_points = map_.GetAllMapPoints();
  size_t n_points = map_points.size();

  for(auto& pMP : map_points ) {
    if(pMP->isBad())
      continue;
    
    _INTERFACE_NAMESPACE::Interface::Vertex vert;
    auto& views = vert.views;

    // set observation
    auto observation = pMP->GetObservations();
    for(auto& pOb : observation) {
      if(pOb.first->isBad())
        continue;
      
      uint32_t img_id = getBindedID(pOb.first->id_);
      if(img_id == -1)
        continue;

      _INTERFACE_NAMESPACE::Interface::Vertex::View view;
      view.imageID = img_id;
      view.confidence = 0;
      views.emplace_back(view);
    }
    // sort image
    if(views.size() < 2)
      continue;

    std::sort(views.begin(), views.end(), 
      [](const _INTERFACE_NAMESPACE::Interface::Vertex::View& a, 
          const _INTERFACE_NAMESPACE::Interface::Vertex::View& b) {
        return a.imageID < b.imageID;
      });

    // set 3D position
    auto p = pMP->GetWorldPos();
    vert.X = cv::Point3f(p(0), p(1), p(2)); 

    scene_.vertices.emplace_back(vert);
  }

  spdlog::info("MVS: {} points defined", n_points);
}

}; // namespace MVS