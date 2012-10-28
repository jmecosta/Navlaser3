// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gz_string_v.proto

#ifndef PROTOBUF_gz_5fstring_5fv_2eproto__INCLUDED
#define PROTOBUF_gz_5fstring_5fv_2eproto__INCLUDED

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
void  protobuf_AddDesc_gz_5fstring_5fv_2eproto();
void protobuf_AssignDesc_gz_5fstring_5fv_2eproto();
void protobuf_ShutdownFile_gz_5fstring_5fv_2eproto();

class GzString_V;

// ===================================================================

class GzString_V : public ::google::protobuf::Message {
 public:
  GzString_V();
  virtual ~GzString_V();
  
  GzString_V(const GzString_V& from);
  
  inline GzString_V& operator=(const GzString_V& from) {
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
  static const GzString_V& default_instance();
  
  void Swap(GzString_V* other);
  
  // implements Message ----------------------------------------------
  
  GzString_V* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const GzString_V& from);
  void MergeFrom(const GzString_V& from);
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
  
  // repeated string data = 1;
  inline int data_size() const;
  inline void clear_data();
  static const int kDataFieldNumber = 1;
  inline const ::std::string& data(int index) const;
  inline ::std::string* mutable_data(int index);
  inline void set_data(int index, const ::std::string& value);
  inline void set_data(int index, const char* value);
  inline void set_data(int index, const char* value, size_t size);
  inline ::std::string* add_data();
  inline void add_data(const ::std::string& value);
  inline void add_data(const char* value);
  inline void add_data(const char* value, size_t size);
  inline const ::google::protobuf::RepeatedPtrField< ::std::string>& data() const;
  inline ::google::protobuf::RepeatedPtrField< ::std::string>* mutable_data();
  
  // @@protoc_insertion_point(class_scope:gazebo.msgs.GzString_V)
 private:
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::google::protobuf::RepeatedPtrField< ::std::string> data_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(1 + 31) / 32];
  
  friend void  protobuf_AddDesc_gz_5fstring_5fv_2eproto();
  friend void protobuf_AssignDesc_gz_5fstring_5fv_2eproto();
  friend void protobuf_ShutdownFile_gz_5fstring_5fv_2eproto();
  
  void InitAsDefaultInstance();
  static GzString_V* default_instance_;
};
// ===================================================================


// ===================================================================

// GzString_V

// repeated string data = 1;
inline int GzString_V::data_size() const {
  return data_.size();
}
inline void GzString_V::clear_data() {
  data_.Clear();
}
inline const ::std::string& GzString_V::data(int index) const {
  return data_.Get(index);
}
inline ::std::string* GzString_V::mutable_data(int index) {
  return data_.Mutable(index);
}
inline void GzString_V::set_data(int index, const ::std::string& value) {
  data_.Mutable(index)->assign(value);
}
inline void GzString_V::set_data(int index, const char* value) {
  data_.Mutable(index)->assign(value);
}
inline void GzString_V::set_data(int index, const char* value, size_t size) {
  data_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
}
inline ::std::string* GzString_V::add_data() {
  return data_.Add();
}
inline void GzString_V::add_data(const ::std::string& value) {
  data_.Add()->assign(value);
}
inline void GzString_V::add_data(const char* value) {
  data_.Add()->assign(value);
}
inline void GzString_V::add_data(const char* value, size_t size) {
  data_.Add()->assign(reinterpret_cast<const char*>(value), size);
}
inline const ::google::protobuf::RepeatedPtrField< ::std::string>&
GzString_V::data() const {
  return data_;
}
inline ::google::protobuf::RepeatedPtrField< ::std::string>*
GzString_V::mutable_data() {
  return &data_;
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

typedef const boost::shared_ptr<gazebo::msgs::GzString_V const> ConstGzString_VPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_gz_5fstring_5fv_2eproto__INCLUDED
