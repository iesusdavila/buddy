#ifndef COCO_FACE_DISPLAY_HPP_
#define COCO_FACE_DISPLAY_HPP_

#include <memory>
#include <string>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/validate_floats.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>

namespace coco_face_display
{

class CocoFaceDisplay : public rviz_common::Display
{
Q_OBJECT
public:
  CocoFaceDisplay();
  ~CocoFaceDisplay() override;

  void onInitialize() override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS: 
  void updateTopic();
  void updateFrame();
  void updatePosition();
  void updateOrientation();
  void updateDimensions();

private:
  void processMessage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void updateFacePlane();
  void updateVisibility();
  void createFacePlane();
  void destroyFacePlane();
  void createTexture();
  void updateTexture(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  Ogre::SceneNode * face_node_;
  Ogre::ManualObject * face_object_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  std::string texture_name_;
  std::string material_name_;

  rviz_common::properties::RosTopicProperty * topic_property_;
  rviz_common::properties::StringProperty * frame_property_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::VectorProperty * orientation_property_;
  rviz_common::properties::FloatProperty * width_property_;
  rviz_common::properties::FloatProperty * height_property_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr current_image_;
  bool new_image_available_;

  std::string frame_id_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  float width_;
  float height_;

  std::mutex mutex_;
};

}  // namespace coco_face_display

#endif  // COCO_FACE_DISPLAY_HPP_