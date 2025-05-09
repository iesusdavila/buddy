#include "coco_face_display/coco_face_display.hpp"

#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreVector3.h>
#include <OgreMaterialManager.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/uniform_string_stream.hpp>

namespace coco_face_display
{

CocoFaceDisplay::CocoFaceDisplay()
: face_node_(nullptr),
  face_object_(nullptr),
  texture_name_("RobotFaceTexture"),
  material_name_("RobotFaceMaterial"),
  new_image_available_(false),
  width_(0.1),
  height_(0.1)
{
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Image Topic", "/face_screen",
    "sensor_msgs/msg/Image",
    "Topic con la imagen para mostrar en la cara del robot",
    this, SLOT(updateTopic()));

  frame_property_ = new rviz_common::properties::StringProperty(
    "Reference Frame", "camera_link",
    "Reference frame for positioning the face display",
    this, SLOT(updateFrame()));

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3(0.007, -0.049, 0),
    "Position of the face display relative to the reference frame",
    this, SLOT(updatePosition()));

  orientation_property_ = new rviz_common::properties::VectorProperty(
    "Orientation (RPY)", Ogre::Vector3(0, 4.7123, 0),
    "Orientation of the face display in roll, pitch, yaw (radians)",
    this, SLOT(updateOrientation()));

  width_property_ = new rviz_common::properties::FloatProperty(
    "Width", 0.12,
    "Width of the face display in meters",
    this, SLOT(updateDimensions()));

  height_property_ = new rviz_common::properties::FloatProperty(
    "Height", 0.06,
    "Height of the face display in meters",
    this, SLOT(updateDimensions()));
}

CocoFaceDisplay::~CocoFaceDisplay()
{
  destroyFacePlane();
}

void CocoFaceDisplay::onInitialize()
{
  Display::onInitialize();
  
  if (!context_) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Context not initialized!");
    return;
  }
  
  scene_manager_ = context_->getSceneManager();
  if (!scene_manager_) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Scene manager not available!");
    return;
  }
  
  frame_id_ = frame_property_->getStdString();
  position_ = position_property_->getVector();
  
  Ogre::Vector3 rpy = orientation_property_->getVector();
  orientation_ = Ogre::Quaternion(
    Ogre::Radian(rpy.x), Ogre::Vector3::UNIT_X) *
    Ogre::Quaternion(Ogre::Radian(rpy.y), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Radian(rpy.z), Ogre::Vector3::UNIT_Z);
  
  width_ = width_property_->getFloat();
  height_ = height_property_->getFloat();
  
  if (!scene_node_) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Scene node not available!");
    return;
  }
  
  face_node_ = scene_node_->createChildSceneNode();
  face_node_->setPosition(position_);
  face_node_->setOrientation(orientation_);
  
  createTexture();
  createFacePlane();
  
  updateTopic();
}

void CocoFaceDisplay::createTexture()
{
  try {
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
      texture_.reset();
    }
    
    if (material_) {
      Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
      material_.reset();
    }
    
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      256, 256, 0, Ogre::PF_B8G8R8A8,  
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
      
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);
    
    for (size_t j = 0; j < 256; j++) {
      for (size_t i = 0; i < 256; i++) {
        *pDest++ = 255; 
        *pDest++ = 255; 
        *pDest++ = 255; 
      }
    }
    pixelBuffer->unlock();
    
    material_ = Ogre::MaterialManager::getSingleton().create(
      material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
    material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);     
    
  }
  catch (const Ogre::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), 
      "Ogre exception in createTexture: %s", e.what());
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), 
      "Exception in createTexture: %s", e.what());
  }
}

void CocoFaceDisplay::createFacePlane()
{
  if (!scene_manager_ || !face_node_) {
    if (!face_node_) {
      face_node_ = scene_node_->createChildSceneNode();
      face_node_->setPosition(position_);
      face_node_->setOrientation(orientation_);
    }
    
    if (!texture_) createTexture();
    if (!material_) createTexture();
  }

  face_object_ = scene_manager_->createManualObject();
  face_node_->attachObject(face_object_);
  
  face_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  float hw = width_ / 2.0f;
  float hh = height_ / 2.0f;
  
  face_object_->position(-hw, -hh, 0.0);
  face_object_->textureCoord(0, 1);
  
  face_object_->position(-hw, hh, 0.0);
  face_object_->textureCoord(0, 0);
  
  face_object_->position(hw, hh, 0.0);
  face_object_->textureCoord(1, 0);
  
  face_object_->position(hw, -hh, 0.0);
  face_object_->textureCoord(1, 1);
  
  face_object_->triangle(0, 1, 2);
  face_object_->triangle(0, 2, 3);
  
  face_object_->end();
}
void CocoFaceDisplay::destroyFacePlane()
{
  if (face_object_) {
    scene_manager_->destroyManualObject(face_object_);
    face_object_ = nullptr;
  }
  
  if (face_node_) {
    scene_manager_->destroySceneNode(face_node_);
    face_node_ = nullptr;
  }
}

void CocoFaceDisplay::updateTopic()
{
  std::string topic = topic_property_->getStdString();
  
  image_sub_.reset();
  
  if (topic.empty()) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", "No topic set");
    return;
  }
  
  try {
    auto node_abstraction = context_->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Node abstraction not available");
      return;
    }
    
    auto node = node_abstraction->get_raw_node();
    if (!node) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "ROS node not available");
      return;
    }
    
    image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
      topic, 
      rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        processMessage(msg);
      });
      
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", QString::fromStdString(topic));
  } 
  catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", 
      QString("Invalid topic name: ") + e.what());
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Invalid topic name: %s", e.what());
  }
  catch (const std::exception& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", 
      QString("Error: ") + e.what());
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Error subscribing to topic: %s", e.what());
  }
}

void CocoFaceDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!msg) {
      RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Received null image message");
      return;
    }
    
    current_image_ = msg;
    new_image_available_ = true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), 
      "Exception in processMessage: %s", e.what());
  }
}

void CocoFaceDisplay::updateTexture(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!msg) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Received null image message");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat bgra;
  
  try {
    if (msg->encoding == "yuv422_yuy2") {
      cv::Mat yuyv(msg->height, msg->width, CV_8UC2, const_cast<uint8_t*>(msg->data.data()));
      cv::Mat bgr;
      cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV); 
      cv_ptr = cv_bridge::CvImageConstPtr(new cv_bridge::CvImage(msg->header, "bgr8", bgr));
    } else if (msg->encoding == "bgr8") {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } else if (msg->encoding == "bgra8") {
      cv_ptr = cv_bridge::toCvShare(msg, "bgra8");
    } 
    else {
      RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Unsupported image format with %d channels", cv_ptr->image.channels());
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Exception during image conversion: %s", e.what());
    return;
  }

  if (!texture_) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Texture not initialized");
    return;
  }

  if (texture_->getWidth() != static_cast<uint32_t>(cv_ptr->image.cols) || 
      texture_->getHeight() != static_cast<uint32_t>(cv_ptr->image.rows)) {
    texture_->unload();
    texture_->setWidth(cv_ptr->image.cols);
    texture_->setHeight(cv_ptr->image.rows);
    texture_->createInternalResources();
  }

  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
  uint8_t* dest = static_cast<uint8_t*>(pixelBox.data);

  Ogre::PixelFormat format = pixelBox.format;

  if (format == Ogre::PF_B8G8R8) { 
    
    size_t dest_pitch = pixelBox.rowPitch * 3;
    size_t src_pitch = cv_ptr->image.step;
    uint8_t* cv_data = cv_ptr->image.data;

    for (size_t row = 0; row < static_cast<size_t>(cv_ptr->image.rows); row++) {
      for (size_t col = 0; col < static_cast<size_t>(cv_ptr->image.cols); col++) {
        cv::Vec3b bgr = cv_ptr->image.at<cv::Vec3b>(row, col);
        dest[(row * pixelBox.rowPitch + col) * 3 + 0] = bgr[2]; // B
        dest[(row * pixelBox.rowPitch + col) * 3 + 1] = bgr[1]; // G
        dest[(row * pixelBox.rowPitch + col) * 3 + 2] = bgr[0]; // R
      }
    }
  } 
  else if (format == Ogre::PF_B8G8R8A8) { 
    size_t dest_pitch = pixelBox.rowPitch * 4;
    size_t src_pitch = cv_ptr->image.step;
    uint8_t* cv_data = cv_ptr->image.data;

    for (size_t row = 0; row < static_cast<size_t>(cv_ptr->image.rows); row++) {
      for (size_t col = 0; col < static_cast<size_t>(cv_ptr->image.cols); col++) {
        cv::Vec4b bgra = cv_ptr->image.at<cv::Vec4b>(row, col);
        dest[(row * pixelBox.rowPitch + col) * 4 + 0] = bgra[3]; // B
        dest[(row * pixelBox.rowPitch + col) * 4 + 1] = bgra[2]; // G
        dest[(row * pixelBox.rowPitch + col) * 4 + 2] = bgra[1]; // R
        dest[(row * pixelBox.rowPitch + col) * 4 + 3] = bgra[0]; // A
      }
    }
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), 
      "Unsupported texture format: %d", static_cast<int>(format));
  }

  pixelBuffer->unlock();
}

void CocoFaceDisplay::onEnable()
{
  updateVisibility();
}

void CocoFaceDisplay::onDisable()
{
  updateVisibility();
}

void CocoFaceDisplay::updateVisibility()
{
  if (face_node_) {
    face_node_->setVisible(isEnabled());
  }
}

void CocoFaceDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  updateFacePlane();

  std::lock_guard<std::mutex> lock(mutex_);
  if (new_image_available_ && current_image_) {
    updateTexture(current_image_);
    new_image_available_ = false;
  }
}

void CocoFaceDisplay::updateFacePlane()
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (context_->getFrameManager()->getTransform(frame_id_, position, orientation))
  {
    face_node_->setPosition(position);
    face_node_->setOrientation(orientation);
    
    Ogre::Vector3 user_position = position_property_->getVector();
    Ogre::Vector3 rpy = orientation_property_->getVector();
    Ogre::Quaternion user_orientation = Ogre::Quaternion(
      Ogre::Radian(rpy.x), Ogre::Vector3::UNIT_X) *
      Ogre::Quaternion(Ogre::Radian(rpy.y), Ogre::Vector3::UNIT_Y) *
      Ogre::Quaternion(Ogre::Radian(rpy.z), Ogre::Vector3::UNIT_Z);
    
    face_node_->translate(orientation * user_position);
    face_node_->rotate(user_orientation);
    
    setStatus(rviz_common::properties::StatusProperty::Ok, "Frame", 
              QString::fromStdString(frame_id_));
  }
  else
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Frame",
              QString("No se encontró el frame [") + QString::fromStdString(frame_id_) + QString("]"));
  }
}

void CocoFaceDisplay::reset()
{
  Display::reset();
  
  std::lock_guard<std::mutex> lock(mutex_);
  current_image_.reset();
  new_image_available_ = false;
}

void CocoFaceDisplay::updateFrame()
{
  frame_id_ = frame_property_->getStdString();
}

void CocoFaceDisplay::updatePosition()
{
  if (face_node_) {
    position_ = position_property_->getVector();
    face_node_->setPosition(position_);
  }
}

void CocoFaceDisplay::updateOrientation()
{
  if (face_node_) {
    Ogre::Vector3 rpy = orientation_property_->getVector();
    orientation_ = Ogre::Quaternion(
      Ogre::Radian(rpy.x), Ogre::Vector3::UNIT_X) *
      Ogre::Quaternion(Ogre::Radian(rpy.y), Ogre::Vector3::UNIT_Y) *
      Ogre::Quaternion(Ogre::Radian(rpy.z), Ogre::Vector3::UNIT_Z);
    face_node_->setOrientation(orientation_);
  }
}

void CocoFaceDisplay::updateDimensions()
{
  width_ = width_property_->getFloat();
  height_ = height_property_->getFloat();

  if (width_ <= 0.0f || height_ <= 0.0f) {
    RCLCPP_ERROR(rclcpp::get_logger("coco_face_display"), "Dimensiones inválidas (<=0)");
    return;
  }

  if (face_object_) {
    destroyFacePlane();
    createFacePlane();
  }
}

}  // namespace coco_face_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(coco_face_display::CocoFaceDisplay, rviz_common::Display)