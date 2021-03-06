// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: contact.proto

#ifndef PROTOBUF_contact_2eproto__INCLUDED
#define PROTOBUF_contact_2eproto__INCLUDED

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
#include "vector3d.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_contact_2eproto();
void protobuf_AssignDesc_contact_2eproto();
void protobuf_ShutdownFile_contact_2eproto();

class Contact;

// ===================================================================

class Contact : public ::google::protobuf::Message {
 public:
  Contact();
  virtual ~Contact();
  
  Contact(const Contact& from);
  
  inline Contact& operator=(const Contact& from) {
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
  static const Contact& default_instance();
  
  void Swap(Contact* other);
  
  // implements Message ----------------------------------------------
  
  Contact* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Contact& from);
  void MergeFrom(const Contact& from);
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
  
  // required string collision1 = 1;
  inline bool has_collision1() const;
  inline void clear_collision1();
  static const int kCollision1FieldNumber = 1;
  inline const ::std::string& collision1() const;
  inline void set_collision1(const ::std::string& value);
  inline void set_collision1(const char* value);
  inline void set_collision1(const char* value, size_t size);
  inline ::std::string* mutable_collision1();
  inline ::std::string* release_collision1();
  
  // required string collision2 = 2;
  inline bool has_collision2() const;
  inline void clear_collision2();
  static const int kCollision2FieldNumber = 2;
  inline const ::std::string& collision2() const;
  inline void set_collision2(const ::std::string& value);
  inline void set_collision2(const char* value);
  inline void set_collision2(const char* value, size_t size);
  inline ::std::string* mutable_collision2();
  inline ::std::string* release_collision2();
  
  // repeated .gazebo.msgs.Vector3d position = 3;
  inline int position_size() const;
  inline void clear_position();
  static const int kPositionFieldNumber = 3;
  inline const ::gazebo::msgs::Vector3d& position(int index) const;
  inline ::gazebo::msgs::Vector3d* mutable_position(int index);
  inline ::gazebo::msgs::Vector3d* add_position();
  inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
      position() const;
  inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
      mutable_position();
  
  // repeated .gazebo.msgs.Vector3d normal = 4;
  inline int normal_size() const;
  inline void clear_normal();
  static const int kNormalFieldNumber = 4;
  inline const ::gazebo::msgs::Vector3d& normal(int index) const;
  inline ::gazebo::msgs::Vector3d* mutable_normal(int index);
  inline ::gazebo::msgs::Vector3d* add_normal();
  inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
      normal() const;
  inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
      mutable_normal();
  
  // repeated double depth = 5;
  inline int depth_size() const;
  inline void clear_depth();
  static const int kDepthFieldNumber = 5;
  inline double depth(int index) const;
  inline void set_depth(int index, double value);
  inline void add_depth(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      depth() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_depth();
  
  // @@protoc_insertion_point(class_scope:gazebo.msgs.Contact)
 private:
  inline void set_has_collision1();
  inline void clear_has_collision1();
  inline void set_has_collision2();
  inline void clear_has_collision2();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::std::string* collision1_;
  ::std::string* collision2_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d > position_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d > normal_;
  ::google::protobuf::RepeatedField< double > depth_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(5 + 31) / 32];
  
  friend void  protobuf_AddDesc_contact_2eproto();
  friend void protobuf_AssignDesc_contact_2eproto();
  friend void protobuf_ShutdownFile_contact_2eproto();
  
  void InitAsDefaultInstance();
  static Contact* default_instance_;
};
// ===================================================================


// ===================================================================

// Contact

// required string collision1 = 1;
inline bool Contact::has_collision1() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Contact::set_has_collision1() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Contact::clear_has_collision1() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Contact::clear_collision1() {
  if (collision1_ != &::google::protobuf::internal::kEmptyString) {
    collision1_->clear();
  }
  clear_has_collision1();
}
inline const ::std::string& Contact::collision1() const {
  return *collision1_;
}
inline void Contact::set_collision1(const ::std::string& value) {
  set_has_collision1();
  if (collision1_ == &::google::protobuf::internal::kEmptyString) {
    collision1_ = new ::std::string;
  }
  collision1_->assign(value);
}
inline void Contact::set_collision1(const char* value) {
  set_has_collision1();
  if (collision1_ == &::google::protobuf::internal::kEmptyString) {
    collision1_ = new ::std::string;
  }
  collision1_->assign(value);
}
inline void Contact::set_collision1(const char* value, size_t size) {
  set_has_collision1();
  if (collision1_ == &::google::protobuf::internal::kEmptyString) {
    collision1_ = new ::std::string;
  }
  collision1_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Contact::mutable_collision1() {
  set_has_collision1();
  if (collision1_ == &::google::protobuf::internal::kEmptyString) {
    collision1_ = new ::std::string;
  }
  return collision1_;
}
inline ::std::string* Contact::release_collision1() {
  clear_has_collision1();
  if (collision1_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = collision1_;
    collision1_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// required string collision2 = 2;
inline bool Contact::has_collision2() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Contact::set_has_collision2() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Contact::clear_has_collision2() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Contact::clear_collision2() {
  if (collision2_ != &::google::protobuf::internal::kEmptyString) {
    collision2_->clear();
  }
  clear_has_collision2();
}
inline const ::std::string& Contact::collision2() const {
  return *collision2_;
}
inline void Contact::set_collision2(const ::std::string& value) {
  set_has_collision2();
  if (collision2_ == &::google::protobuf::internal::kEmptyString) {
    collision2_ = new ::std::string;
  }
  collision2_->assign(value);
}
inline void Contact::set_collision2(const char* value) {
  set_has_collision2();
  if (collision2_ == &::google::protobuf::internal::kEmptyString) {
    collision2_ = new ::std::string;
  }
  collision2_->assign(value);
}
inline void Contact::set_collision2(const char* value, size_t size) {
  set_has_collision2();
  if (collision2_ == &::google::protobuf::internal::kEmptyString) {
    collision2_ = new ::std::string;
  }
  collision2_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Contact::mutable_collision2() {
  set_has_collision2();
  if (collision2_ == &::google::protobuf::internal::kEmptyString) {
    collision2_ = new ::std::string;
  }
  return collision2_;
}
inline ::std::string* Contact::release_collision2() {
  clear_has_collision2();
  if (collision2_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = collision2_;
    collision2_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// repeated .gazebo.msgs.Vector3d position = 3;
inline int Contact::position_size() const {
  return position_.size();
}
inline void Contact::clear_position() {
  position_.Clear();
}
inline const ::gazebo::msgs::Vector3d& Contact::position(int index) const {
  return position_.Get(index);
}
inline ::gazebo::msgs::Vector3d* Contact::mutable_position(int index) {
  return position_.Mutable(index);
}
inline ::gazebo::msgs::Vector3d* Contact::add_position() {
  return position_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
Contact::position() const {
  return position_;
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
Contact::mutable_position() {
  return &position_;
}

// repeated .gazebo.msgs.Vector3d normal = 4;
inline int Contact::normal_size() const {
  return normal_.size();
}
inline void Contact::clear_normal() {
  normal_.Clear();
}
inline const ::gazebo::msgs::Vector3d& Contact::normal(int index) const {
  return normal_.Get(index);
}
inline ::gazebo::msgs::Vector3d* Contact::mutable_normal(int index) {
  return normal_.Mutable(index);
}
inline ::gazebo::msgs::Vector3d* Contact::add_normal() {
  return normal_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
Contact::normal() const {
  return normal_;
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
Contact::mutable_normal() {
  return &normal_;
}

// repeated double depth = 5;
inline int Contact::depth_size() const {
  return depth_.size();
}
inline void Contact::clear_depth() {
  depth_.Clear();
}
inline double Contact::depth(int index) const {
  return depth_.Get(index);
}
inline void Contact::set_depth(int index, double value) {
  depth_.Set(index, value);
}
inline void Contact::add_depth(double value) {
  depth_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
Contact::depth() const {
  return depth_;
}
inline ::google::protobuf::RepeatedField< double >*
Contact::mutable_depth() {
  return &depth_;
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

typedef const boost::shared_ptr<gazebo::msgs::Contact const> ConstContactPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_contact_2eproto__INCLUDED
