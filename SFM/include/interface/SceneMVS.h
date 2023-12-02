#ifndef __SCENE_MVS_H__
#define __SCENE_MVS_H__

#include "InterfaceMVS.h"
#include <unordered_map>
#include "System.h"

namespace MVS {

class Scene {
public:
  // constructor
  Scene(ORB_SLAM2::System& system);
  // serialize
  void serialize(const std::string& filename, bool convert_cv2gl = true);

protected:
  // define platforms with camera intrinsic
  void definePlatform();

  // define image and pose
  void defineImagePose();

  // define point cloud
  void defineStructure();

  // map keyframe id
  void bindKeyframeID();
  uint32_t getBindedID(unsigned long kfid);

  // keyframe id maps
  std::unordered_map<unsigned long, uint32_t> kfid_map_;

  // openMVS interface
  _INTERFACE_NAMESPACE::Interface scene_;

  // Slam SFM system
  ORB_SLAM2::System &system_;
  ORB_SLAM2::Map &map_;

  // coordinate convert
  bool convert_cv2gl_ = true;

};



}; // namespace MVS

#endif //  __SCENE_MVS_H__