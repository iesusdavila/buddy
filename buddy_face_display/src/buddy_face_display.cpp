#include "buddy_face_display/buddy_face_display.hpp"

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

namespace buddy_face_display
{

BuddyFaceDisplay::BuddyFaceDisplay()
: face_node_(nullptr),
  face_object_(nullptr),
  texture_name_("RobotFaceTexture"),
  material_name_("RobotFaceMaterial"),
  new_image_available_(false),
  width_(0.1),
  height_(0.1)
{
  // topic_property_ = new rviz_common::properties::RosTopicProperty(
  //   "Image Topic", "",
  //   "sensor_msgs/msg/Image",
  //   "Topic with the image to display on the robot's face",
  //   this, SLOT(updateTopic()));
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Image Topic", "/image_raw",
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

BuddyFaceDisplay::~BuddyFaceDisplay()
{
  destroyFacePlane();
}

// void BuddyFaceDisplay::onInitialize()
// {
//   Display::onInitialize();
//   scene_manager_ = context_->getSceneManager();
//   frame_id_ = frame_property_->getStdString();
//   position_ = position_property_->getVector();
  
//   // Convert RPY to quaternion
//   Ogre::Vector3 rpy = orientation_property_->getVector();
//   orientation_ = Ogre::Quaternion(
//     Ogre::Radian(rpy.x), Ogre::Vector3::UNIT_X) *
//     Ogre::Quaternion(Ogre::Radian(rpy.y), Ogre::Vector3::UNIT_Y) *
//     Ogre::Quaternion(Ogre::Radian(rpy.z), Ogre::Vector3::UNIT_Z);
  
//   width_ = width_property_->getFloat();
//   height_ = height_property_->getFloat();
  
//   face_node_ = scene_node_->createChildSceneNode();
//   face_node_->setPosition(position_);
//   face_node_->setOrientation(orientation_);
  
//   createTexture();
//   createFacePlane();
// }

void BuddyFaceDisplay::onInitialize()
{
  Display::onInitialize();
  
  if (!context_) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Context not initialized!");
    return;
  }
  
  scene_manager_ = context_->getSceneManager();
  if (!scene_manager_) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Scene manager not available!");
    return;
  }
  
  frame_id_ = frame_property_->getStdString();
  position_ = position_property_->getVector();
  
  // Convert RPY to quaternion
  Ogre::Vector3 rpy = orientation_property_->getVector();
  orientation_ = Ogre::Quaternion(
    Ogre::Radian(rpy.x), Ogre::Vector3::UNIT_X) *
    Ogre::Quaternion(Ogre::Radian(rpy.y), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Radian(rpy.z), Ogre::Vector3::UNIT_Z);
  
  width_ = width_property_->getFloat();
  height_ = height_property_->getFloat();
  
  if (!scene_node_) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Scene node not available!");
    return;
  }
  
  face_node_ = scene_node_->createChildSceneNode();
  face_node_->setPosition(position_);
  face_node_->setOrientation(orientation_);
  
  createTexture();
  createFacePlane();
  
  // Suscríbete al tópico después de inicializar todo
  updateTopic();
}

void BuddyFaceDisplay::createTexture()
{
  try {
    // Eliminar textura y material existentes si los hay
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
      texture_.reset();
    }
    
    if (material_) {
      Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
      material_.reset();
    }
    
    // Crear nueva textura
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      256, 256, 0, Ogre::PF_B8G8R8,  // Usar BGRA8 que es compatible con OpenCV PF_BYTE_BGR
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
      
    // Crear textura blanca por defecto
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);
    
    // Llenar con píxeles blancos (formato BGRA)
    for (size_t j = 0; j < 256; j++) {
      for (size_t i = 0; i < 256; i++) {
        *pDest++ = 255; // b
        *pDest++ = 255; // g
        *pDest++ = 255; // r
      }
    }
    pixelBuffer->unlock();
    
    // Crear material
    material_ = Ogre::MaterialManager::getSingleton().create(
      material_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
    // material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    // material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);     
    
    RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), "Texture created successfully");
  }
  catch (const Ogre::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
      "Ogre exception in createTexture: %s", e.what());
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
      "Exception in createTexture: %s", e.what());
  }
}

// void BuddyFaceDisplay::createTexture()
// {
//   // Create default white texture
//   texture_ = Ogre::TextureManager::getSingleton().createManual(
//     texture_name_,
//     Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
//     Ogre::TEX_TYPE_2D,
//     256, 256, 0, Ogre::PF_R8G8B8A8,
//     Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
    
//   // Create a white texture as default
//   Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
//   pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
//   const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
//   uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);
  
//   // Fill with white pixels
//   for (size_t j = 0; j < 256; j++) {
//     for (size_t i = 0; i < 256; i++) {
//       *pDest++ = 255; // r
//       *pDest++ = 255; // g
//       *pDest++ = 255; // b
//       *pDest++ = 255; // a
//     }
//   }
//   pixelBuffer->unlock();
  
//   // Create material
//   material_ = Ogre::MaterialManager::getSingleton().create(
//     material_name_,
//     Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
//   material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
//   material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
//   material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
// }

void BuddyFaceDisplay::createFacePlane()
{
  if (!scene_manager_ || !face_node_) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Recursos de Ogre no inicializados!");
    
    // Recrear face_node_ si es necesario
    if (!face_node_) {
      face_node_ = scene_node_->createChildSceneNode();
      face_node_->setPosition(position_);
      face_node_->setOrientation(orientation_);
    }
    
    // Recrear textura y material si no existen
    if (!texture_) createTexture();
    if (!material_) createTexture();
  }

  face_object_ = scene_manager_->createManualObject();
  face_node_->attachObject(face_object_);
  
  face_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  
  float hw = width_ / 2.0f;
  float hh = height_ / 2.0f;
  
  // Define the quad for the face display
  // Bottom left
  face_object_->position(-hw, -hh, 0.0);
  face_object_->textureCoord(0, 1);
  
  // Top left
  face_object_->position(-hw, hh, 0.0);
  face_object_->textureCoord(0, 0);
  
  // Top right
  face_object_->position(hw, hh, 0.0);
  face_object_->textureCoord(1, 0);
  
  // Bottom right
  face_object_->position(hw, -hh, 0.0);
  face_object_->textureCoord(1, 1);
  
  // Define triangles
  face_object_->triangle(0, 1, 2);
  face_object_->triangle(0, 2, 3);
  
  face_object_->end();
}

// void BuddyFaceDisplay::destroyFacePlane()
// {
//   if (face_object_) {
//     scene_manager_->destroyManualObject(face_object_);
//     face_object_ = nullptr;
//   }
  
//   if (face_node_) {
//     scene_manager_->destroySceneNode(face_node_);
//     face_node_ = nullptr;
//   }
  
//   // if (material_) {
//   //   Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
//   //   material_.reset();
//   // }
  
//   // if (texture_) {
//   //   Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
//   //   texture_.reset();
//   // }
// }

void BuddyFaceDisplay::destroyFacePlane()
{
  if (face_object_) {
    scene_manager_->destroyManualObject(face_object_);
    face_object_ = nullptr;
  }
  
  if (face_node_) {
    scene_manager_->destroySceneNode(face_node_);
    face_node_ = nullptr;
  }
  
  // Descomenta estas líneas para liberar correctamente los recursos
  // if (material_) {
  //   Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
  //   material_.reset();
  // }
  
  // if (texture_) {
  //   Ogre::TextureManager::getSingleton().remove(texture_->getHandle());
  //   texture_.reset();
  // }
}

// void BuddyFaceDisplay::updateTopic()
// {
//   if (!context_) {
//     RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Contexto no inicializado!");
//     return;
//   }

//   std::string topic = topic_property_->getStdString();
//   image_sub_.reset(); 

//   if (topic.empty()) return;
//   // if (topic.empty()) {
//   //   RCLCPP_WARN(rclcpp::get_logger("buddy_face_display"), "Topic vacío. Usando '/image_raw' por defecto.");
//   //   topic = "/image_raw";
//   //   topic_property_->setString(topic);
//   // }
  
//   // if (topic.empty()) {
//   //   image_sub_.reset();
//   //   return;
//   // }

//   // try {
//   //   auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
//   //   image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
//   //     topic, 10, 
//   //     std::bind(&BuddyFaceDisplay::processMessage, this, std::placeholders::_1));
//   //   setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
//   // } catch (const rclcpp::exceptions::InvalidTopicNameError & e) {
//     // setStatus(
//     //   rviz_common::properties::StatusProperty::Error, "Topic",
//     //   QString("Error subscribing: ") + e.what());
//   // }

//   try { 
//     auto node_abstraction = context_->getRosNodeAbstraction().lock();
//     if (!node_abstraction) throw std::runtime_error("Node abstraction no disponible");
    
//     auto node = node_abstraction->get_raw_node();
//     if (!node) throw std::runtime_error("Nodo ROS2 inválido");

//     image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
//       topic, 10,
//       [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
//         std::lock_guard<std::mutex> lock(mutex_);
//         processMessage(msg);
//       }
//     );
//     setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
//   } catch (const std::exception& e) {
//     RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Error: %s", e.what());
//     setStatus(
//       rviz_common::properties::StatusProperty::Error, "Topic",
//       QString("Error subscribing: ") + e.what());
//   }
// }

void BuddyFaceDisplay::updateTopic()
{
  std::string topic = topic_property_->getStdString();
  
  // Reset current subscription
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
    
    // Create a new subscription with a lambda function callback
    image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
      topic, 
      rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        processMessage(msg);
      });
      
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", QString::fromStdString(topic));
    RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), "Subscribed to topic: %s", topic.c_str());
  } 
  catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", 
      QString("Invalid topic name: ") + e.what());
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Invalid topic name: %s", e.what());
  }
  catch (const std::exception& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", 
      QString("Error: ") + e.what());
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Error subscribing to topic: %s", e.what());
  }
}

// void BuddyFaceDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
// {
//   std::lock_guard<std::mutex> lock(mutex_);
//   current_image_ = msg;
//   new_image_available_ = true;
// }

void BuddyFaceDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!msg) {
      RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), "Received null image message");
      return;
    }
    
    // Verificar dimensiones razonables para evitar errores de memoria
    if (msg->width > 4096 || msg->height > 4096) {
      RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), 
        "Image dimensions too large: %dx%d", msg->width, msg->height);
      return;
    }
    
    // Log más detalles sobre la imagen recibida
    RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), 
      "Received image: %dx%d, encoding: %s, step: %d", 
      msg->width, msg->height, msg->encoding.c_str(), msg->step);
    
    current_image_ = msg;
    new_image_available_ = true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
      "Exception in processMessage: %s", e.what());
  }
}

// void BuddyFaceDisplay::updateTexture(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
// {
//   std::lock_guard<std::mutex> lock(mutex_);
//   cv_bridge::CvImageConstPtr cv_ptr;
//   try {
//     cv_ptr = cv_bridge::toCvShare(msg, "bgra8");
//   } catch (cv_bridge::Exception& e) {
//     // If the image type is not BGRA8, convert it
//     try {
//       cv_ptr = cv_bridge::toCvShare(msg);
//       cv::Mat bgra;
//       if (cv_ptr->image.channels() == 1) {
//         cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_GRAY2BGRA);
//       } else if (cv_ptr->image.channels() == 3) {
//         cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_BGR2BGRA);
//       } else {
//         RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
//           "Unsupported image format, expected BGRA8, BGR8, or MONO8");
//         return;
//       }
//       cv_ptr = cv_bridge::CvImageConstPtr(new cv_bridge::CvImage(msg->header, "bgra8", bgra));
//     } catch (cv_bridge::Exception& e) {
//       RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
//         "CV bridge exception: %s", e.what());
//       return;
//     }
//   }

//   // Resize texture if needed
//   if (texture_->getWidth() != static_cast<uint32_t>(cv_ptr->image.cols) || 
//       texture_->getHeight() != static_cast<uint32_t>(cv_ptr->image.rows)) {
//     texture_->unload();
//     texture_->setWidth(cv_ptr->image.cols);
//     texture_->setHeight(cv_ptr->image.rows);
//     texture_->createInternalResources();
//   }

//   // Copy image data to texture
//   Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
//   pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
//   const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
//   uint8_t* dest = static_cast<uint8_t*>(pixelBox.data);

//   // Make sure the pixel formats align
//   Ogre::PixelFormat format = pixelBox.format;
//   if (format == Ogre::PF_R8G8B8A8 || format == Ogre::PF_A8R8G8B8) {
//     size_t dest_pitch = pixelBox.rowPitch * Ogre::PixelUtil::getNumElemBytes(format);
//     size_t src_pitch = cv_ptr->image.step;
//     uint8_t* cv_data = cv_ptr->image.data;

//     for (size_t row = 0; row < static_cast<size_t>(cv_ptr->image.rows); row++) {
//       memcpy(dest, cv_data, src_pitch);
//       dest += dest_pitch;
//       cv_data += src_pitch;
//     }
//   } else {
//     RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
//       "Unsupported texture format");
//   }

//   pixelBuffer->unlock();
// }

void BuddyFaceDisplay::updateTexture(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), "Updating texture with new image");

  if (!msg) {
    RCLCPP_WARN(rclcpp::get_logger("buddy_face_display"), "Received null image message");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), 
    "Processing image: %dx%d, encoding: %s", 
    msg->width, msg->height, msg->encoding.c_str());

  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat bgra;
  
  try {
    // Manejo especial para YUV422_YUY2
    if (msg->encoding == "yuv422_yuy2") {
      RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), 
        "The image is in YUV422_YUY2 format. Converting to BGRA8 first");

      // cv::Mat yuyv(msg->height, msg->width, CV_8UC2, const_cast<uint8_t*>(msg->data.data()));
      // cv::Mat bgr;
      // cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV); // Conversión directa a BGR
      // cv_ptr = cv_bridge::CvImageConstPtr(new cv_bridge::CvImage(msg->header, "bgr8", bgr));

      cv::Mat yuyv(msg->height, msg->width, CV_8UC2, const_cast<uint8_t*>(msg->data.data()));

      cv::Mat bgr;
      cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);  // Usar COLOR_YUV2BGR_YUY2 en lugar de COLOR_YUV2BGR_YUYV

      // Verificar si la conversión fue exitosa
      if (bgr.empty()) {
          RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Conversión YUV422 a BGR fallida!");
          return;
      }

      // Forzar 3 canales (BGR)
      if (bgr.channels() != 3) {
          RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Canal BGR incorrecto: %d", bgr.channels());
          return;
      }

      cv_ptr = cv_bridge::CvImageConstPtr(new cv_bridge::CvImage(msg->header, "bgr8", bgr));

    } else {
      // Intentar convertir directamente para otros formatos
      try {
        cv_ptr = cv_bridge::toCvShare(msg, "bgra8");

        // if (msg->encoding == "bgra8") {
        //   cv_ptr = cv_bridge::toCvShare(msg, "bgra8");
        // } else {
        //   cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        //   cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_BGR2BGRA);
        //   cv_ptr = cv_bridge::CvImageConstPtr(new cv_bridge::CvImage(msg->header, "bgra8", bgra));
        // }
      } catch (cv_bridge::Exception& e) {
        // Convertir desde otros formatos si la conversión directa falla
        cv_ptr = cv_bridge::toCvShare(msg);
        
        if (cv_ptr->image.channels() == 1) {
          cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_GRAY2BGRA);
        } 
        else if (cv_ptr->image.channels() == 3) {
          cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_BGR2BGRA);
        } 
        else if (cv_ptr->image.channels() == 4) {
          if (cv_ptr->encoding == "rgba8") {
            cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_RGBA2BGRA);
          } else {
            bgra = cv_ptr->image;
          }
        } 
        else {
          RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
            "Unsupported image format with %d channels", cv_ptr->image.channels());
          return;
        }
        
        cv_ptr = cv_bridge::CvImageConstPtr(new cv_bridge::CvImage(msg->header, "bgra8", bgra));
      }
    }
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
      "OpenCV exception: %s", e.what());
    return;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
      "Exception during image conversion: %s", e.what());
    return;
  }

  // Check if we have a valid OpenGL texture
  if (!texture_) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Texture not initialized");
    return;
  }

  // Resize texture if needed to match image dimensions
  if (texture_->getWidth() != static_cast<uint32_t>(cv_ptr->image.cols) || 
      texture_->getHeight() != static_cast<uint32_t>(cv_ptr->image.rows)) {
    
    RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), 
      "Resizing texture to %dx%d", cv_ptr->image.cols, cv_ptr->image.rows);
    
    texture_->unload();
    texture_->setWidth(cv_ptr->image.cols);
    texture_->setHeight(cv_ptr->image.rows);
    texture_->createInternalResources();
  }

  // Copy image data to texture
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
  uint8_t* dest = static_cast<uint8_t*>(pixelBox.data);

  // Make sure the pixel formats align
  Ogre::PixelFormat format = pixelBox.format;
  RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), 
    "Texture format: %d, PF_B8G8R8A8=%d", 
    static_cast<int>(format), static_cast<int>(Ogre::PF_B8G8R8A8));

  if (format == Ogre::PF_B8G8R8) { 
    
    size_t dest_pitch = pixelBox.rowPitch * 3;//Ogre::PixelUtil::getNumElemBytes(format);
    size_t src_pitch = cv_ptr->image.step;
    uint8_t* cv_data = cv_ptr->image.data;

    // Print some debug info
    RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), 
      "Copying %dx%d image to texture. Format: %d", 
      cv_ptr->image.cols, cv_ptr->image.rows, static_cast<int>(format));

    // Copy row by row
    for (size_t row = 0; row < static_cast<size_t>(cv_ptr->image.rows); row++) {
      memcpy(dest, cv_data, src_pitch);
      dest += dest_pitch;
      cv_data += src_pitch;
    }
  } 
  else {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), 
      "Unsupported texture format: %d", static_cast<int>(format));
  }

  pixelBuffer->unlock();
  
  // Indicate texture has been updated
  RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), "Texture updated successfully");
}

void BuddyFaceDisplay::onEnable()
{
  updateVisibility();
}

void BuddyFaceDisplay::onDisable()
{
  updateVisibility();
}

void BuddyFaceDisplay::updateVisibility()
{
  if (face_node_) {
    face_node_->setVisible(isEnabled());
  }
}

void BuddyFaceDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  // Actualizar la posición y orientación según el frame
  updateFacePlane();

  std::lock_guard<std::mutex> lock(mutex_);
  if (new_image_available_ && current_image_) {
    updateTexture(current_image_);
    new_image_available_ = false;
  }
}

// void BuddyFaceDisplay::update(float wall_dt, float ros_dt) {
//   (void)wall_dt;
//   (void)ros_dt;

//   // Actualizar transformación del frame
//   Ogre::Vector3 position;
//   Ogre::Quaternion orientation;
//   if (!context_->getFrameManager()->getTransform(frame_id_, rclcpp::Time(0), position, orientation)) {
//     RCLCPP_ERROR_THROTTLE(
//       rclcpp::get_logger("buddy_face_display"),
//       *context_->getClock(), 1000,
//       "Error transforming frame '%s'", frame_id_.c_str());
//     return;
//   }

//   if (face_node_) {
//     face_node_->setPosition(position);
//     face_node_->setOrientation(orientation);
//   }

//   // Actualizar textura si hay nueva imagen
//   std::lock_guard<std::mutex> lock(mutex_);
//   if (new_image_available_ && current_image_) {
//     updateTexture(current_image_);
//     new_image_available_ = false;
//   }
// }

void BuddyFaceDisplay::updateFacePlane()
{
  // Obtener la transformación desde el frame de referencia
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (context_->getFrameManager()->getTransform(frame_id_, position, orientation))
  {
    // Aplicar la transformación base del frame
    face_node_->setPosition(position);
    face_node_->setOrientation(orientation);
    
    // Aplicar el offset adicional configurado por el usuario
    Ogre::Vector3 user_position = position_property_->getVector();
    Ogre::Vector3 rpy = orientation_property_->getVector();
    Ogre::Quaternion user_orientation = Ogre::Quaternion(
      Ogre::Radian(rpy.x), Ogre::Vector3::UNIT_X) *
      Ogre::Quaternion(Ogre::Radian(rpy.y), Ogre::Vector3::UNIT_Y) *
      Ogre::Quaternion(Ogre::Radian(rpy.z), Ogre::Vector3::UNIT_Z);
    
    // Aplicar posición y orientación relativa
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

void BuddyFaceDisplay::reset()
{
  Display::reset();
  
  std::lock_guard<std::mutex> lock(mutex_);
  current_image_.reset();
  new_image_available_ = false;
}

// Slots for property updates
void BuddyFaceDisplay::updateFrame()
{
  frame_id_ = frame_property_->getStdString();
}

void BuddyFaceDisplay::updatePosition()
{
  if (face_node_) {
    position_ = position_property_->getVector();
    face_node_->setPosition(position_);
  }
}

void BuddyFaceDisplay::updateOrientation()
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

void BuddyFaceDisplay::updateDimensions()
{
  width_ = width_property_->getFloat();
  height_ = height_property_->getFloat();

  RCLCPP_INFO(rclcpp::get_logger("buddy_face_display"), "Updating dimensions: %.2f x %.2f", width_, height_);

  // Validar dimensiones
  if (width_ <= 0.0f || height_ <= 0.0f) {
    RCLCPP_ERROR(rclcpp::get_logger("buddy_face_display"), "Dimensiones inválidas (<=0)");
    return;
  }

  if (face_object_) {
    destroyFacePlane();
    createFacePlane();
  }
}

}  // namespace buddy_face_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(buddy_face_display::BuddyFaceDisplay, rviz_common::Display)