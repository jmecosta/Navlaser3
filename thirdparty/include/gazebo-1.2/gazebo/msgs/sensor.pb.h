// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sensor.proto

#ifndef PROTOBUF_sensor_2eproto__INCLUDED
#define PROTOBUF_sensor_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2004001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_message_reflection.h>
#include "pose.pb.h"
#include "camerasensor.pb.h"
#include "raysensor.pb.h"
#include "contactsensor.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_sensor_2eproto();
void protobuf_AssignDesc_sensor_2eproto();
void protobuf_ShutdownFile_sensor_2eproto();

class Sensor;

// ===================================================================

class Sensor : public ::google::protobuf::Message {
 public:
  Sensor();
  virtual ~Sensor();
  
  Sensor(const Sensor& from);
  
  inline Sensor& operator=(const Sensor& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const Sensor& default_instance();
  
  void Swap(Sensor* other);
  
  // implements Message ----------------------------------------------
  
  Sensor* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Sensor& from);
  void MergeFrom(const Sensor& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // required string name = 1;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 1;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  inline ::std::string* release_name();
  
  // required string parent = 2;
  inline bool has_parent() const;
  inline void clear_parent();
  static const int kParentFieldNumber = 2;
  inline const ::std::string& parent() const;
  inline void set_parent(const ::std::string& value);
  inline void set_parent(const char* value);
  inline void set_parent(const char* value, size_t size);
  inline ::std::string* mutable_parent();
  inline ::std::string* release_parent();
  
  // required string type = 3;
  inline bool has_type() const;
  inline void clear_type();
  static const int kTypeFieldNumber = 3;
  inline const ::std::string& type() const;
  inline void set_type(const ::std::string& value);
  inline void set_type(const char* value);
  inline void set_type(const char* value, size_t size);
  inline ::std::string* mutable_type();
  inline ::std::string* release_type();
  
  // optional bool always_on = 4;
  inline bool has_always_on() const;
  inline void clear_always_on();
  static const int kAlwaysOnFieldNumber = 4;
  inline bool always_on() const;
  inline void set_always_on(bool value);
  
  // optional double update_rate = 5;
  inline bool has_update_rate() const;
  inline void clear_update_rate();
  static const int kUpdateRateFieldNumber = 5;
  inline double update_rate() const;
  inline void set_update_rate(double value);
  
  // optional .gazebo.msgs.Pose pose = 6;
  inline bool has_pose() const;
  inline void clear_pose();
  static const int kPoseFieldNumber = 6;
  inline const ::gazebo::msgs::Pose& pose() const;
  inline ::gazebo::msgs::Pose* mutable_pose();
  inline ::gazebo::msgs::Pose* release_pose();
  
  // optional .gazebo.msgs.CameraSensor camera = 7;
  inline bool has_camera() const;
  inline void clear_camera();
  static const int kCameraFieldNumber = 7;
  inline const ::gazebo::msgs::CameraSensor& camera() const;
  inline ::gazebo::msgs::CameraSensor* mutable_camera();
  inline ::gazebo::msgs::CameraSensor* release_camera();
  
  // optional .gazebo.msgs.RaySensor ray = 8;
  inline bool has_ray() const;
  inline void clear_ray();
  static const int kRayFieldNumber = 8;
  inline const ::gazebo::msgs::RaySensor& ray() const;
  inline ::gazebo::msgs::RaySensor* mutable_ray();
  inline ::gazebo::msgs::RaySensor* release_ray();
  
  // optional .gazebo.msgs.ContactSensor contact = 9;
  inline bool has_contact() const;
  inline void clear_contact();
  static const int kContactFieldNumber = 9;
  inline const ::gazebo::msgs::ContactSensor& contact() const;
  inline ::gazebo::msgs::ContactSensor* mutable_contact();
  inline ::gazebo::msgs::ContactSensor* release_contact();
  
  // optional bool visualize = 10;
  inline bool has_visualize() const;
  inline void clear_visualize();
  static const int kVisualizeFieldNumber = 10;
  inline bool visualize() const;
  inline void set_visualize(bool value);
  
  // optional string topic = 11;
  inline bool has_topic() const;
  inline void clear_topic();
  static const int kTopicFieldNumber = 11;
  inline const ::std::string& topic() const;
  inline void set_topic(const ::std::string& value);
  inline void set_topic(const char* value);
  inline void set_topic(const char* value, size_t size);
  inline ::std::string* mutable_topic();
  inline ::std::string* release_topic();
  
  // @@protoc_insertion_point(class_scope:gazebo.msgs.Sensor)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_parent();
  inline void clear_has_parent();
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_always_on();
  inline void clear_has_always_on();
  inline void set_has_update_rate();
  inline void clear_has_update_rate();
  inline void set_has_pose();
  inline void clear_has_pose();
  inline void set_has_camera();
  inline void clear_has_camera();
  inline void set_has_ray();
  inline void clear_has_ray();
  inline void set_has_contact();
  inline void clear_has_contact();
  inline void set_has_visualize();
  inline void clear_has_visualize();
  inline void set_has_topic();
  inline void clear_has_topic();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::std::string* name_;
  ::std::string* parent_;
  ::std::string* type_;
  double update_rate_;
  ::gazebo::msgs::Pose* pose_;
  ::gazebo::msgs::CameraSensor* camera_;
  ::gazebo::msgs::RaySensor* ray_;
  ::gazebo::msgs::ContactSensor* contact_;
  ::std::string* topic_;
  bool always_on_;
  bool visualize_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(11 + 31) / 32];
  
  friend void  protobuf_AddDesc_sensor_2eproto();
  friend void protobuf_AssignDesc_sensor_2eproto();
  friend void protobuf_ShutdownFile_sensor_2eproto();
  
  void InitAsDefaultInstance();
  static Sensor* default_instance_;
};
// ===================================================================


// ===================================================================

// Sensor

// required string name = 1;
inline bool Sensor::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Sensor::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Sensor::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Sensor::clear_name() {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    name_->clear();
  }
  clear_has_name();
}
inline const ::std::string& Sensor::name() const {
  return *name_;
}
inline void Sensor::set_name(const ::std::string& value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Sensor::set_name(const char* value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Sensor::set_name(const char* value, size_t size) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Sensor::mutable_name() {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  return name_;
}
inline ::std::string* Sensor::release_name() {
  clear_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = name_;
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// required string parent = 2;
inline bool Sensor::has_parent() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Sensor::set_has_parent() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Sensor::clear_has_parent() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Sensor::clear_parent() {
  if (parent_ != &::google::protobuf::internal::kEmptyString) {
    parent_->clear();
  }
  clear_has_parent();
}
inline const ::std::string& Sensor::parent() const {
  return *parent_;
}
inline void Sensor::set_parent(const ::std::string& value) {
  set_has_parent();
  if (parent_ == &::google::protobuf::internal::kEmptyString) {
    parent_ = new ::std::string;
  }
  parent_->assign(value);
}
inline void Sensor::set_parent(const char* value) {
  set_has_parent();
  if (parent_ == &::google::protobuf::internal::kEmptyString) {
    parent_ = new ::std::string;
  }
  parent_->assign(value);
}
inline void Sensor::set_parent(const char* value, size_t size) {
  set_has_parent();
  if (parent_ == &::google::protobuf::internal::kEmptyString) {
    parent_ = new ::std::string;
  }
  parent_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Sensor::mutable_parent() {
  set_has_parent();
  if (parent_ == &::google::protobuf::internal::kEmptyString) {
    parent_ = new ::std::string;
  }
  return parent_;
}
inline ::std::string* Sensor::release_parent() {
  clear_has_parent();
  if (parent_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = parent_;
    parent_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// required string type = 3;
inline bool Sensor::has_type() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Sensor::set_has_type() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Sensor::clear_has_type() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Sensor::clear_type() {
  if (type_ != &::google::protobuf::internal::kEmptyString) {
    type_->clear();
  }
  clear_has_type();
}
inline const ::std::string& Sensor::type() const {
  return *type_;
}
inline void Sensor::set_type(const ::std::string& value) {
  set_has_type();
  if (type_ == &::google::protobuf::internal::kEmptyString) {
    type_ = new ::std::string;
  }
  type_->assign(value);
}
inline void Sensor::set_type(const char* value) {
  set_has_type();
  if (type_ == &::google::protobuf::internal::kEmptyString) {
    type_ = new ::std::string;
  }
  type_->assign(value);
}
inline void Sensor::set_type(const char* value, size_t size) {
  set_has_type();
  if (type_ == &::google::protobuf::internal::kEmptyString) {
    type_ = new ::std::string;
  }
  type_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Sensor::mutable_type() {
  set_has_type();
  if (type_ == &::google::protobuf::internal::kEmptyString) {
    type_ = new ::std::string;
  }
  return type_;
}
inline ::std::string* Sensor::release_type() {
  clear_has_type();
  if (type_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = type_;
    type_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// optional bool always_on = 4;
inline bool Sensor::has_always_on() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Sensor::set_has_always_on() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Sensor::clear_has_always_on() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Sensor::clear_always_on() {
  always_on_ = false;
  clear_has_always_on();
}
inline bool Sensor::always_on() const {
  return always_on_;
}
inline void Sensor::set_always_on(bool value) {
  set_has_always_on();
  always_on_ = value;
}

// optional double update_rate = 5;
inline bool Sensor::has_update_rate() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Sensor::set_has_update_rate() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Sensor::clear_has_update_rate() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Sensor::clear_update_rate() {
  update_rate_ = 0;
  clear_has_update_rate();
}
inline double Sensor::update_rate() const {
  return update_rate_;
}
inline void Sensor::set_update_rate(double value) {
  set_has_update_rate();
  update_rate_ = value;
}

// optional .gazebo.msgs.Pose pose = 6;
inline bool Sensor::has_pose() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Sensor::set_has_pose() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Sensor::clear_has_pose() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Sensor::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& Sensor::pose() const {
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* Sensor::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) pose_ = new ::gazebo::msgs::Pose;
  return pose_;
}
inline ::gazebo::msgs::Pose* Sensor::release_pose() {
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}

// optional .gazebo.msgs.CameraSensor camera = 7;
inline bool Sensor::has_camera() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Sensor::set_has_camera() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Sensor::clear_has_camera() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Sensor::clear_camera() {
  if (camera_ != NULL) camera_->::gazebo::msgs::CameraSensor::Clear();
  clear_has_camera();
}
inline const ::gazebo::msgs::CameraSensor& Sensor::camera() const {
  return camera_ != NULL ? *camera_ : *default_instance_->camera_;
}
inline ::gazebo::msgs::CameraSensor* Sensor::mutable_camera() {
  set_has_camera();
  if (camera_ == NULL) camera_ = new ::gazebo::msgs::CameraSensor;
  return camera_;
}
inline ::gazebo::msgs::CameraSensor* Sensor::release_camera() {
  clear_has_camera();
  ::gazebo::msgs::CameraSensor* temp = camera_;
  camera_ = NULL;
  return temp;
}

// optional .gazebo.msgs.RaySensor ray = 8;
inline bool Sensor::has_ray() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Sensor::set_has_ray() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Sensor::clear_has_ray() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Sensor::clear_ray() {
  if (ray_ != NULL) ray_->::gazebo::msgs::RaySensor::Clear();
  clear_has_ray();
}
inline const ::gazebo::msgs::RaySensor& Sensor::ray() const {
  return ray_ != NULL ? *ray_ : *default_instance_->ray_;
}
inline ::gazebo::msgs::RaySensor* Sensor::mutable_ray() {
  set_has_ray();
  if (ray_ == NULL) ray_ = new ::gazebo::msgs::RaySensor;
  return ray_;
}
inline ::gazebo::msgs::RaySensor* Sensor::release_ray() {
  clear_has_ray();
  ::gazebo::msgs::RaySensor* temp = ray_;
  ray_ = NULL;
  return temp;
}

// optional .gazebo.msgs.ContactSensor contact = 9;
inline bool Sensor::has_contact() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void Sensor::set_has_contact() {
  _has_bits_[0] |= 0x00000100u;
}
inline void Sensor::clear_has_contact() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void Sensor::clear_contact() {
  if (contact_ != NULL) contact_->::gazebo::msgs::ContactSensor::Clear();
  clear_has_contact();
}
inline const ::gazebo::msgs::ContactSensor& Sensor::contact() const {
  return contact_ != NULL ? *contact_ : *default_instance_->contact_;
}
inline ::gazebo::msgs::ContactSensor* Sensor::mutable_contact() {
  set_has_contact();
  if (contact_ == NULL) contact_ = new ::gazebo::msgs::ContactSensor;
  return contact_;
}
inline ::gazebo::msgs::ContactSensor* Sensor::release_contact() {
  clear_has_contact();
  ::gazebo::msgs::ContactSensor* temp = contact_;
  contact_ = NULL;
  return temp;
}

// optional bool visualize = 10;
inline bool Sensor::has_visualize() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void Sensor::set_has_visualize() {
  _has_bits_[0] |= 0x00000200u;
}
inline void Sensor::clear_has_visualize() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void Sensor::clear_visualize() {
  visualize_ = false;
  clear_has_visualize();
}
inline bool Sensor::visualize() const {
  return visualize_;
}
inline void Sensor::set_visualize(bool value) {
  set_has_visualize();
  visualize_ = value;
}

// optional string topic = 11;
inline bool Sensor::has_topic() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void Sensor::set_has_topic() {
  _has_bits_[0] |= 0x00000400u;
}
inline void Sensor::clear_has_topic() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void Sensor::clear_topic() {
  if (topic_ != &::google::protobuf::internal::kEmptyString) {
    topic_->clear();
  }
  clear_has_topic();
}
inline const ::std::string& Sensor::topic() const {
  return *topic_;
}
inline void Sensor::set_topic(const ::std::string& value) {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  topic_->assign(value);
}
inline void Sensor::set_topic(const char* value) {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  topic_->assign(value);
}
inline void Sensor::set_topic(const char* value, size_t size) {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  topic_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Sensor::mutable_topic() {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  return topic_;
}
inline ::std::string* Sensor::release_topic() {
  clear_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = topic_;
    topic_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Sensor const> ConstSensorPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_sensor_2eproto__INCLUDED
