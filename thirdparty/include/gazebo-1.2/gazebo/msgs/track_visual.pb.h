// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: track_visual.proto

#ifndef PROTOBUF_track_5fvisual_2eproto__INCLUDED
#define PROTOBUF_track_5fvisual_2eproto__INCLUDED

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
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_track_5fvisual_2eproto();
void protobuf_AssignDesc_track_5fvisual_2eproto();
void protobuf_ShutdownFile_track_5fvisual_2eproto();

class TrackVisual;

// ===================================================================

class TrackVisual : public ::google::protobuf::Message {
 public:
  TrackVisual();
  virtual ~TrackVisual();
  
  TrackVisual(const TrackVisual& from);
  
  inline TrackVisual& operator=(const TrackVisual& from) {
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
  static const TrackVisual& default_instance();
  
  void Swap(TrackVisual* other);
  
  // implements Message ----------------------------------------------
  
  TrackVisual* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const TrackVisual& from);
  void MergeFrom(const TrackVisual& from);
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
  
  // optional bool inherit_orientation = 2;
  inline bool has_inherit_orientation() const;
  inline void clear_inherit_orientation();
  static const int kInheritOrientationFieldNumber = 2;
  inline bool inherit_orientation() const;
  inline void set_inherit_orientation(bool value);
  
  // optional double min_dist = 3;
  inline bool has_min_dist() const;
  inline void clear_min_dist();
  static const int kMinDistFieldNumber = 3;
  inline double min_dist() const;
  inline void set_min_dist(double value);
  
  // optional double max_dist = 4;
  inline bool has_max_dist() const;
  inline void clear_max_dist();
  static const int kMaxDistFieldNumber = 4;
  inline double max_dist() const;
  inline void set_max_dist(double value);
  
  // @@protoc_insertion_point(class_scope:gazebo.msgs.TrackVisual)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_inherit_orientation();
  inline void clear_has_inherit_orientation();
  inline void set_has_min_dist();
  inline void clear_has_min_dist();
  inline void set_has_max_dist();
  inline void clear_has_max_dist();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::std::string* name_;
  double min_dist_;
  double max_dist_;
  bool inherit_orientation_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(4 + 31) / 32];
  
  friend void  protobuf_AddDesc_track_5fvisual_2eproto();
  friend void protobuf_AssignDesc_track_5fvisual_2eproto();
  friend void protobuf_ShutdownFile_track_5fvisual_2eproto();
  
  void InitAsDefaultInstance();
  static TrackVisual* default_instance_;
};
// ===================================================================


// ===================================================================

// TrackVisual

// required string name = 1;
inline bool TrackVisual::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void TrackVisual::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void TrackVisual::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void TrackVisual::clear_name() {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    name_->clear();
  }
  clear_has_name();
}
inline const ::std::string& TrackVisual::name() const {
  return *name_;
}
inline void TrackVisual::set_name(const ::std::string& value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void TrackVisual::set_name(const char* value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void TrackVisual::set_name(const char* value, size_t size) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* TrackVisual::mutable_name() {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  return name_;
}
inline ::std::string* TrackVisual::release_name() {
  clear_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = name_;
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// optional bool inherit_orientation = 2;
inline bool TrackVisual::has_inherit_orientation() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void TrackVisual::set_has_inherit_orientation() {
  _has_bits_[0] |= 0x00000002u;
}
inline void TrackVisual::clear_has_inherit_orientation() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void TrackVisual::clear_inherit_orientation() {
  inherit_orientation_ = false;
  clear_has_inherit_orientation();
}
inline bool TrackVisual::inherit_orientation() const {
  return inherit_orientation_;
}
inline void TrackVisual::set_inherit_orientation(bool value) {
  set_has_inherit_orientation();
  inherit_orientation_ = value;
}

// optional double min_dist = 3;
inline bool TrackVisual::has_min_dist() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void TrackVisual::set_has_min_dist() {
  _has_bits_[0] |= 0x00000004u;
}
inline void TrackVisual::clear_has_min_dist() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void TrackVisual::clear_min_dist() {
  min_dist_ = 0;
  clear_has_min_dist();
}
inline double TrackVisual::min_dist() const {
  return min_dist_;
}
inline void TrackVisual::set_min_dist(double value) {
  set_has_min_dist();
  min_dist_ = value;
}

// optional double max_dist = 4;
inline bool TrackVisual::has_max_dist() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void TrackVisual::set_has_max_dist() {
  _has_bits_[0] |= 0x00000008u;
}
inline void TrackVisual::clear_has_max_dist() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void TrackVisual::clear_max_dist() {
  max_dist_ = 0;
  clear_has_max_dist();
}
inline double TrackVisual::max_dist() const {
  return max_dist_;
}
inline void TrackVisual::set_max_dist(double value) {
  set_has_max_dist();
  max_dist_ = value;
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

typedef const boost::shared_ptr<gazebo::msgs::TrackVisual const> ConstTrackVisualPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_track_5fvisual_2eproto__INCLUDED
