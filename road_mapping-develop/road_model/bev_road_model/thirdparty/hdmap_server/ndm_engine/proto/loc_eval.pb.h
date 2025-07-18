// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: loc_eval.proto

#ifndef PROTOBUF_loc_5feval_2eproto__INCLUDED
#define PROTOBUF_loc_5feval_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "sense.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_loc_5feval_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsSessionInfoImpl();
void InitDefaultsSessionInfo();
inline void InitDefaults() {
  InitDefaultsSessionInfo();
}
}  // namespace protobuf_loc_5feval_2eproto
namespace ndm_proto {
class SessionInfo;
class SessionInfoDefaultTypeInternal;
extern SessionInfoDefaultTypeInternal _SessionInfo_default_instance_;
}  // namespace ndm_proto
namespace ndm_proto {

// ===================================================================

class SessionInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:ndm_proto.SessionInfo) */ {
 public:
  SessionInfo();
  virtual ~SessionInfo();

  SessionInfo(const SessionInfo& from);

  inline SessionInfo& operator=(const SessionInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SessionInfo(SessionInfo&& from) noexcept
    : SessionInfo() {
    *this = ::std::move(from);
  }

  inline SessionInfo& operator=(SessionInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SessionInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SessionInfo* internal_default_instance() {
    return reinterpret_cast<const SessionInfo*>(
               &_SessionInfo_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(SessionInfo* other);
  friend void swap(SessionInfo& a, SessionInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SessionInfo* New() const PROTOBUF_FINAL { return New(NULL); }

  SessionInfo* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SessionInfo& from);
  void MergeFrom(const SessionInfo& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(SessionInfo* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional string start_pack_name = 7;
  bool has_start_pack_name() const;
  void clear_start_pack_name();
  static const int kStartPackNameFieldNumber = 7;
  const ::std::string& start_pack_name() const;
  void set_start_pack_name(const ::std::string& value);
  #if LANG_CXX11
  void set_start_pack_name(::std::string&& value);
  #endif
  void set_start_pack_name(const char* value);
  void set_start_pack_name(const char* value, size_t size);
  ::std::string* mutable_start_pack_name();
  ::std::string* release_start_pack_name();
  void set_allocated_start_pack_name(::std::string* start_pack_name);

  // optional string end_pack_name = 8;
  bool has_end_pack_name() const;
  void clear_end_pack_name();
  static const int kEndPackNameFieldNumber = 8;
  const ::std::string& end_pack_name() const;
  void set_end_pack_name(const ::std::string& value);
  #if LANG_CXX11
  void set_end_pack_name(::std::string&& value);
  #endif
  void set_end_pack_name(const char* value);
  void set_end_pack_name(const char* value, size_t size);
  ::std::string* mutable_end_pack_name();
  ::std::string* release_end_pack_name();
  void set_allocated_end_pack_name(::std::string* end_pack_name);

  // required .ndm_proto.Vector3D error_value = 11;
  bool has_error_value() const;
  void clear_error_value();
  static const int kErrorValueFieldNumber = 11;
  const ::ndm_proto::Vector3D& error_value() const;
  ::ndm_proto::Vector3D* release_error_value();
  ::ndm_proto::Vector3D* mutable_error_value();
  void set_allocated_error_value(::ndm_proto::Vector3D* error_value);

  // required .ndm_proto.Vector3D location = 12;
  bool has_location() const;
  void clear_location();
  static const int kLocationFieldNumber = 12;
  const ::ndm_proto::Vector3D& location() const;
  ::ndm_proto::Vector3D* release_location();
  ::ndm_proto::Vector3D* mutable_location();
  void set_allocated_location(::ndm_proto::Vector3D* location);

  // required uint32 session_type = 1;
  bool has_session_type() const;
  void clear_session_type();
  static const int kSessionTypeFieldNumber = 1;
  ::google::protobuf::uint32 session_type() const;
  void set_session_type(::google::protobuf::uint32 value);

  // optional uint32 start_frame_id = 2;
  bool has_start_frame_id() const;
  void clear_start_frame_id();
  static const int kStartFrameIdFieldNumber = 2;
  ::google::protobuf::uint32 start_frame_id() const;
  void set_start_frame_id(::google::protobuf::uint32 value);

  // required double distance = 4;
  bool has_distance() const;
  void clear_distance();
  static const int kDistanceFieldNumber = 4;
  double distance() const;
  void set_distance(double value);

  // optional uint32 end_frame_id = 3;
  bool has_end_frame_id() const;
  void clear_end_frame_id();
  static const int kEndFrameIdFieldNumber = 3;
  ::google::protobuf::uint32 end_frame_id() const;
  void set_end_frame_id(::google::protobuf::uint32 value);

  // optional uint32 start_pack_frame_id = 5;
  bool has_start_pack_frame_id() const;
  void clear_start_pack_frame_id();
  static const int kStartPackFrameIdFieldNumber = 5;
  ::google::protobuf::uint32 start_pack_frame_id() const;
  void set_start_pack_frame_id(::google::protobuf::uint32 value);

  // required uint64 start_ts = 9;
  bool has_start_ts() const;
  void clear_start_ts();
  static const int kStartTsFieldNumber = 9;
  ::google::protobuf::uint64 start_ts() const;
  void set_start_ts(::google::protobuf::uint64 value);

  // required uint64 end_ts = 10;
  bool has_end_ts() const;
  void clear_end_ts();
  static const int kEndTsFieldNumber = 10;
  ::google::protobuf::uint64 end_ts() const;
  void set_end_ts(::google::protobuf::uint64 value);

  // optional uint32 end_pack_frame_id = 6;
  bool has_end_pack_frame_id() const;
  void clear_end_pack_frame_id();
  static const int kEndPackFrameIdFieldNumber = 6;
  ::google::protobuf::uint32 end_pack_frame_id() const;
  void set_end_pack_frame_id(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:ndm_proto.SessionInfo)
 private:
  void set_has_session_type();
  void clear_has_session_type();
  void set_has_start_frame_id();
  void clear_has_start_frame_id();
  void set_has_end_frame_id();
  void clear_has_end_frame_id();
  void set_has_distance();
  void clear_has_distance();
  void set_has_start_pack_frame_id();
  void clear_has_start_pack_frame_id();
  void set_has_end_pack_frame_id();
  void clear_has_end_pack_frame_id();
  void set_has_start_pack_name();
  void clear_has_start_pack_name();
  void set_has_end_pack_name();
  void clear_has_end_pack_name();
  void set_has_start_ts();
  void clear_has_start_ts();
  void set_has_end_ts();
  void clear_has_end_ts();
  void set_has_error_value();
  void clear_has_error_value();
  void set_has_location();
  void clear_has_location();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr start_pack_name_;
  ::google::protobuf::internal::ArenaStringPtr end_pack_name_;
  ::ndm_proto::Vector3D* error_value_;
  ::ndm_proto::Vector3D* location_;
  ::google::protobuf::uint32 session_type_;
  ::google::protobuf::uint32 start_frame_id_;
  double distance_;
  ::google::protobuf::uint32 end_frame_id_;
  ::google::protobuf::uint32 start_pack_frame_id_;
  ::google::protobuf::uint64 start_ts_;
  ::google::protobuf::uint64 end_ts_;
  ::google::protobuf::uint32 end_pack_frame_id_;
  friend struct ::protobuf_loc_5feval_2eproto::TableStruct;
  friend void ::protobuf_loc_5feval_2eproto::InitDefaultsSessionInfoImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SessionInfo

// required uint32 session_type = 1;
inline bool SessionInfo::has_session_type() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SessionInfo::set_has_session_type() {
  _has_bits_[0] |= 0x00000010u;
}
inline void SessionInfo::clear_has_session_type() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void SessionInfo::clear_session_type() {
  session_type_ = 0u;
  clear_has_session_type();
}
inline ::google::protobuf::uint32 SessionInfo::session_type() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.session_type)
  return session_type_;
}
inline void SessionInfo::set_session_type(::google::protobuf::uint32 value) {
  set_has_session_type();
  session_type_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.session_type)
}

// optional uint32 start_frame_id = 2;
inline bool SessionInfo::has_start_frame_id() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void SessionInfo::set_has_start_frame_id() {
  _has_bits_[0] |= 0x00000020u;
}
inline void SessionInfo::clear_has_start_frame_id() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void SessionInfo::clear_start_frame_id() {
  start_frame_id_ = 0u;
  clear_has_start_frame_id();
}
inline ::google::protobuf::uint32 SessionInfo::start_frame_id() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.start_frame_id)
  return start_frame_id_;
}
inline void SessionInfo::set_start_frame_id(::google::protobuf::uint32 value) {
  set_has_start_frame_id();
  start_frame_id_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.start_frame_id)
}

// optional uint32 end_frame_id = 3;
inline bool SessionInfo::has_end_frame_id() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void SessionInfo::set_has_end_frame_id() {
  _has_bits_[0] |= 0x00000080u;
}
inline void SessionInfo::clear_has_end_frame_id() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void SessionInfo::clear_end_frame_id() {
  end_frame_id_ = 0u;
  clear_has_end_frame_id();
}
inline ::google::protobuf::uint32 SessionInfo::end_frame_id() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.end_frame_id)
  return end_frame_id_;
}
inline void SessionInfo::set_end_frame_id(::google::protobuf::uint32 value) {
  set_has_end_frame_id();
  end_frame_id_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.end_frame_id)
}

// required double distance = 4;
inline bool SessionInfo::has_distance() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void SessionInfo::set_has_distance() {
  _has_bits_[0] |= 0x00000040u;
}
inline void SessionInfo::clear_has_distance() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void SessionInfo::clear_distance() {
  distance_ = 0;
  clear_has_distance();
}
inline double SessionInfo::distance() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.distance)
  return distance_;
}
inline void SessionInfo::set_distance(double value) {
  set_has_distance();
  distance_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.distance)
}

// optional uint32 start_pack_frame_id = 5;
inline bool SessionInfo::has_start_pack_frame_id() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void SessionInfo::set_has_start_pack_frame_id() {
  _has_bits_[0] |= 0x00000100u;
}
inline void SessionInfo::clear_has_start_pack_frame_id() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void SessionInfo::clear_start_pack_frame_id() {
  start_pack_frame_id_ = 0u;
  clear_has_start_pack_frame_id();
}
inline ::google::protobuf::uint32 SessionInfo::start_pack_frame_id() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.start_pack_frame_id)
  return start_pack_frame_id_;
}
inline void SessionInfo::set_start_pack_frame_id(::google::protobuf::uint32 value) {
  set_has_start_pack_frame_id();
  start_pack_frame_id_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.start_pack_frame_id)
}

// optional uint32 end_pack_frame_id = 6;
inline bool SessionInfo::has_end_pack_frame_id() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void SessionInfo::set_has_end_pack_frame_id() {
  _has_bits_[0] |= 0x00000800u;
}
inline void SessionInfo::clear_has_end_pack_frame_id() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void SessionInfo::clear_end_pack_frame_id() {
  end_pack_frame_id_ = 0u;
  clear_has_end_pack_frame_id();
}
inline ::google::protobuf::uint32 SessionInfo::end_pack_frame_id() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.end_pack_frame_id)
  return end_pack_frame_id_;
}
inline void SessionInfo::set_end_pack_frame_id(::google::protobuf::uint32 value) {
  set_has_end_pack_frame_id();
  end_pack_frame_id_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.end_pack_frame_id)
}

// optional string start_pack_name = 7;
inline bool SessionInfo::has_start_pack_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SessionInfo::set_has_start_pack_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SessionInfo::clear_has_start_pack_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SessionInfo::clear_start_pack_name() {
  start_pack_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_start_pack_name();
}
inline const ::std::string& SessionInfo::start_pack_name() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.start_pack_name)
  return start_pack_name_.GetNoArena();
}
inline void SessionInfo::set_start_pack_name(const ::std::string& value) {
  set_has_start_pack_name();
  start_pack_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.start_pack_name)
}
#if LANG_CXX11
inline void SessionInfo::set_start_pack_name(::std::string&& value) {
  set_has_start_pack_name();
  start_pack_name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:ndm_proto.SessionInfo.start_pack_name)
}
#endif
inline void SessionInfo::set_start_pack_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_start_pack_name();
  start_pack_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:ndm_proto.SessionInfo.start_pack_name)
}
inline void SessionInfo::set_start_pack_name(const char* value, size_t size) {
  set_has_start_pack_name();
  start_pack_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:ndm_proto.SessionInfo.start_pack_name)
}
inline ::std::string* SessionInfo::mutable_start_pack_name() {
  set_has_start_pack_name();
  // @@protoc_insertion_point(field_mutable:ndm_proto.SessionInfo.start_pack_name)
  return start_pack_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SessionInfo::release_start_pack_name() {
  // @@protoc_insertion_point(field_release:ndm_proto.SessionInfo.start_pack_name)
  clear_has_start_pack_name();
  return start_pack_name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SessionInfo::set_allocated_start_pack_name(::std::string* start_pack_name) {
  if (start_pack_name != NULL) {
    set_has_start_pack_name();
  } else {
    clear_has_start_pack_name();
  }
  start_pack_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), start_pack_name);
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.SessionInfo.start_pack_name)
}

// optional string end_pack_name = 8;
inline bool SessionInfo::has_end_pack_name() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SessionInfo::set_has_end_pack_name() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SessionInfo::clear_has_end_pack_name() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SessionInfo::clear_end_pack_name() {
  end_pack_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_end_pack_name();
}
inline const ::std::string& SessionInfo::end_pack_name() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.end_pack_name)
  return end_pack_name_.GetNoArena();
}
inline void SessionInfo::set_end_pack_name(const ::std::string& value) {
  set_has_end_pack_name();
  end_pack_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.end_pack_name)
}
#if LANG_CXX11
inline void SessionInfo::set_end_pack_name(::std::string&& value) {
  set_has_end_pack_name();
  end_pack_name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:ndm_proto.SessionInfo.end_pack_name)
}
#endif
inline void SessionInfo::set_end_pack_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_end_pack_name();
  end_pack_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:ndm_proto.SessionInfo.end_pack_name)
}
inline void SessionInfo::set_end_pack_name(const char* value, size_t size) {
  set_has_end_pack_name();
  end_pack_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:ndm_proto.SessionInfo.end_pack_name)
}
inline ::std::string* SessionInfo::mutable_end_pack_name() {
  set_has_end_pack_name();
  // @@protoc_insertion_point(field_mutable:ndm_proto.SessionInfo.end_pack_name)
  return end_pack_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SessionInfo::release_end_pack_name() {
  // @@protoc_insertion_point(field_release:ndm_proto.SessionInfo.end_pack_name)
  clear_has_end_pack_name();
  return end_pack_name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SessionInfo::set_allocated_end_pack_name(::std::string* end_pack_name) {
  if (end_pack_name != NULL) {
    set_has_end_pack_name();
  } else {
    clear_has_end_pack_name();
  }
  end_pack_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), end_pack_name);
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.SessionInfo.end_pack_name)
}

// required uint64 start_ts = 9;
inline bool SessionInfo::has_start_ts() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void SessionInfo::set_has_start_ts() {
  _has_bits_[0] |= 0x00000200u;
}
inline void SessionInfo::clear_has_start_ts() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void SessionInfo::clear_start_ts() {
  start_ts_ = GOOGLE_ULONGLONG(0);
  clear_has_start_ts();
}
inline ::google::protobuf::uint64 SessionInfo::start_ts() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.start_ts)
  return start_ts_;
}
inline void SessionInfo::set_start_ts(::google::protobuf::uint64 value) {
  set_has_start_ts();
  start_ts_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.start_ts)
}

// required uint64 end_ts = 10;
inline bool SessionInfo::has_end_ts() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void SessionInfo::set_has_end_ts() {
  _has_bits_[0] |= 0x00000400u;
}
inline void SessionInfo::clear_has_end_ts() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void SessionInfo::clear_end_ts() {
  end_ts_ = GOOGLE_ULONGLONG(0);
  clear_has_end_ts();
}
inline ::google::protobuf::uint64 SessionInfo::end_ts() const {
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.end_ts)
  return end_ts_;
}
inline void SessionInfo::set_end_ts(::google::protobuf::uint64 value) {
  set_has_end_ts();
  end_ts_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.SessionInfo.end_ts)
}

// required .ndm_proto.Vector3D error_value = 11;
inline bool SessionInfo::has_error_value() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SessionInfo::set_has_error_value() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SessionInfo::clear_has_error_value() {
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::ndm_proto::Vector3D& SessionInfo::error_value() const {
  const ::ndm_proto::Vector3D* p = error_value_;
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.error_value)
  return p != NULL ? *p : *reinterpret_cast<const ::ndm_proto::Vector3D*>(
      &::ndm_proto::_Vector3D_default_instance_);
}
inline ::ndm_proto::Vector3D* SessionInfo::release_error_value() {
  // @@protoc_insertion_point(field_release:ndm_proto.SessionInfo.error_value)
  clear_has_error_value();
  ::ndm_proto::Vector3D* temp = error_value_;
  error_value_ = NULL;
  return temp;
}
inline ::ndm_proto::Vector3D* SessionInfo::mutable_error_value() {
  set_has_error_value();
  if (error_value_ == NULL) {
    error_value_ = new ::ndm_proto::Vector3D;
  }
  // @@protoc_insertion_point(field_mutable:ndm_proto.SessionInfo.error_value)
  return error_value_;
}
inline void SessionInfo::set_allocated_error_value(::ndm_proto::Vector3D* error_value) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(error_value_);
  }
  if (error_value) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      error_value = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, error_value, submessage_arena);
    }
    set_has_error_value();
  } else {
    clear_has_error_value();
  }
  error_value_ = error_value;
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.SessionInfo.error_value)
}

// required .ndm_proto.Vector3D location = 12;
inline bool SessionInfo::has_location() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SessionInfo::set_has_location() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SessionInfo::clear_has_location() {
  _has_bits_[0] &= ~0x00000008u;
}
inline const ::ndm_proto::Vector3D& SessionInfo::location() const {
  const ::ndm_proto::Vector3D* p = location_;
  // @@protoc_insertion_point(field_get:ndm_proto.SessionInfo.location)
  return p != NULL ? *p : *reinterpret_cast<const ::ndm_proto::Vector3D*>(
      &::ndm_proto::_Vector3D_default_instance_);
}
inline ::ndm_proto::Vector3D* SessionInfo::release_location() {
  // @@protoc_insertion_point(field_release:ndm_proto.SessionInfo.location)
  clear_has_location();
  ::ndm_proto::Vector3D* temp = location_;
  location_ = NULL;
  return temp;
}
inline ::ndm_proto::Vector3D* SessionInfo::mutable_location() {
  set_has_location();
  if (location_ == NULL) {
    location_ = new ::ndm_proto::Vector3D;
  }
  // @@protoc_insertion_point(field_mutable:ndm_proto.SessionInfo.location)
  return location_;
}
inline void SessionInfo::set_allocated_location(::ndm_proto::Vector3D* location) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(location_);
  }
  if (location) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      location = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, location, submessage_arena);
    }
    set_has_location();
  } else {
    clear_has_location();
  }
  location_ = location;
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.SessionInfo.location)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace ndm_proto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_loc_5feval_2eproto__INCLUDED
