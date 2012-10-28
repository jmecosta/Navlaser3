// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: subscribe.proto

#ifndef PROTOBUF_subscribe_2eproto__INCLUDED
#define PROTOBUF_subscribe_2eproto__INCLUDED

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
void  protobuf_AddDesc_subscribe_2eproto();
void protobuf_AssignDesc_subscribe_2eproto();
void protobuf_ShutdownFile_subscribe_2eproto();

class Subscribe;

// ===================================================================

class Subscribe : public ::google::protobuf::Message {
 public:
  Subscribe();
  virtual ~Subscribe();
  
  Subscribe(const Subscribe& from);
  
  inline Subscribe& operator=(const Subscribe& from) {
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
  static const Subscribe& default_instance();
  
  void Swap(Subscribe* other);
  
  // implements Message ----------------------------------------------
  
  Subscribe* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Subscribe& from);
  void MergeFrom(const Subscribe& from);
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
  
  // required string topic = 1;
  inline bool has_topic() const;
  inline void clear_topic();
  static const int kTopicFieldNumber = 1;
  inline const ::std::string& topic() const;
  inline void set_topic(const ::std::string& value);
  inline void set_topic(const char* value);
  inline void set_topic(const char* value, size_t size);
  inline ::std::string* mutable_topic();
  inline ::std::string* release_topic();
  
  // required string host = 2;
  inline bool has_host() const;
  inline void clear_host();
  static const int kHostFieldNumber = 2;
  inline const ::std::string& host() const;
  inline void set_host(const ::std::string& value);
  inline void set_host(const char* value);
  inline void set_host(const char* value, size_t size);
  inline ::std::string* mutable_host();
  inline ::std::string* release_host();
  
  // required uint32 port = 3;
  inline bool has_port() const;
  inline void clear_port();
  static const int kPortFieldNumber = 3;
  inline ::google::protobuf::uint32 port() const;
  inline void set_port(::google::protobuf::uint32 value);
  
  // required string msg_type = 4;
  inline bool has_msg_type() const;
  inline void clear_msg_type();
  static const int kMsgTypeFieldNumber = 4;
  inline const ::std::string& msg_type() const;
  inline void set_msg_type(const ::std::string& value);
  inline void set_msg_type(const char* value);
  inline void set_msg_type(const char* value, size_t size);
  inline ::std::string* mutable_msg_type();
  inline ::std::string* release_msg_type();
  
  // optional bool latching = 5 [default = false];
  inline bool has_latching() const;
  inline void clear_latching();
  static const int kLatchingFieldNumber = 5;
  inline bool latching() const;
  inline void set_latching(bool value);
  
  // @@protoc_insertion_point(class_scope:gazebo.msgs.Subscribe)
 private:
  inline void set_has_topic();
  inline void clear_has_topic();
  inline void set_has_host();
  inline void clear_has_host();
  inline void set_has_port();
  inline void clear_has_port();
  inline void set_has_msg_type();
  inline void clear_has_msg_type();
  inline void set_has_latching();
  inline void clear_has_latching();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::std::string* topic_;
  ::std::string* host_;
  ::std::string* msg_type_;
  ::google::protobuf::uint32 port_;
  bool latching_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(5 + 31) / 32];
  
  friend void  protobuf_AddDesc_subscribe_2eproto();
  friend void protobuf_AssignDesc_subscribe_2eproto();
  friend void protobuf_ShutdownFile_subscribe_2eproto();
  
  void InitAsDefaultInstance();
  static Subscribe* default_instance_;
};
// ===================================================================


// ===================================================================

// Subscribe

// required string topic = 1;
inline bool Subscribe::has_topic() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Subscribe::set_has_topic() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Subscribe::clear_has_topic() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Subscribe::clear_topic() {
  if (topic_ != &::google::protobuf::internal::kEmptyString) {
    topic_->clear();
  }
  clear_has_topic();
}
inline const ::std::string& Subscribe::topic() const {
  return *topic_;
}
inline void Subscribe::set_topic(const ::std::string& value) {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  topic_->assign(value);
}
inline void Subscribe::set_topic(const char* value) {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  topic_->assign(value);
}
inline void Subscribe::set_topic(const char* value, size_t size) {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  topic_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Subscribe::mutable_topic() {
  set_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    topic_ = new ::std::string;
  }
  return topic_;
}
inline ::std::string* Subscribe::release_topic() {
  clear_has_topic();
  if (topic_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = topic_;
    topic_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// required string host = 2;
inline bool Subscribe::has_host() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Subscribe::set_has_host() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Subscribe::clear_has_host() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Subscribe::clear_host() {
  if (host_ != &::google::protobuf::internal::kEmptyString) {
    host_->clear();
  }
  clear_has_host();
}
inline const ::std::string& Subscribe::host() const {
  return *host_;
}
inline void Subscribe::set_host(const ::std::string& value) {
  set_has_host();
  if (host_ == &::google::protobuf::internal::kEmptyString) {
    host_ = new ::std::string;
  }
  host_->assign(value);
}
inline void Subscribe::set_host(const char* value) {
  set_has_host();
  if (host_ == &::google::protobuf::internal::kEmptyString) {
    host_ = new ::std::string;
  }
  host_->assign(value);
}
inline void Subscribe::set_host(const char* value, size_t size) {
  set_has_host();
  if (host_ == &::google::protobuf::internal::kEmptyString) {
    host_ = new ::std::string;
  }
  host_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Subscribe::mutable_host() {
  set_has_host();
  if (host_ == &::google::protobuf::internal::kEmptyString) {
    host_ = new ::std::string;
  }
  return host_;
}
inline ::std::string* Subscribe::release_host() {
  clear_has_host();
  if (host_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = host_;
    host_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// required uint32 port = 3;
inline bool Subscribe::has_port() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Subscribe::set_has_port() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Subscribe::clear_has_port() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Subscribe::clear_port() {
  port_ = 0u;
  clear_has_port();
}
inline ::google::protobuf::uint32 Subscribe::port() const {
  return port_;
}
inline void Subscribe::set_port(::google::protobuf::uint32 value) {
  set_has_port();
  port_ = value;
}

// required string msg_type = 4;
inline bool Subscribe::has_msg_type() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Subscribe::set_has_msg_type() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Subscribe::clear_has_msg_type() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Subscribe::clear_msg_type() {
  if (msg_type_ != &::google::protobuf::internal::kEmptyString) {
    msg_type_->clear();
  }
  clear_has_msg_type();
}
inline const ::std::string& Subscribe::msg_type() const {
  return *msg_type_;
}
inline void Subscribe::set_msg_type(const ::std::string& value) {
  set_has_msg_type();
  if (msg_type_ == &::google::protobuf::internal::kEmptyString) {
    msg_type_ = new ::std::string;
  }
  msg_type_->assign(value);
}
inline void Subscribe::set_msg_type(const char* value) {
  set_has_msg_type();
  if (msg_type_ == &::google::protobuf::internal::kEmptyString) {
    msg_type_ = new ::std::string;
  }
  msg_type_->assign(value);
}
inline void Subscribe::set_msg_type(const char* value, size_t size) {
  set_has_msg_type();
  if (msg_type_ == &::google::protobuf::internal::kEmptyString) {
    msg_type_ = new ::std::string;
  }
  msg_type_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Subscribe::mutable_msg_type() {
  set_has_msg_type();
  if (msg_type_ == &::google::protobuf::internal::kEmptyString) {
    msg_type_ = new ::std::string;
  }
  return msg_type_;
}
inline ::std::string* Subscribe::release_msg_type() {
  clear_has_msg_type();
  if (msg_type_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = msg_type_;
    msg_type_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}

// optional bool latching = 5 [default = false];
inline bool Subscribe::has_latching() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Subscribe::set_has_latching() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Subscribe::clear_has_latching() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Subscribe::clear_latching() {
  latching_ = false;
  clear_has_latching();
}
inline bool Subscribe::latching() const {
  return latching_;
}
inline void Subscribe::set_latching(bool value) {
  set_has_latching();
  latching_ = value;
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

typedef const boost::shared_ptr<gazebo::msgs::Subscribe const> ConstSubscribePtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_subscribe_2eproto__INCLUDED
