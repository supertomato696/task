// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ego_layer.proto

#ifndef PROTOBUF_ego_5flayer_2eproto__INCLUDED
#define PROTOBUF_ego_5flayer_2eproto__INCLUDED

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
#include "base.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_ego_5flayer_2eproto {
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
void InitDefaultsEgoLayerImpl();
void InitDefaultsEgoLayer();
inline void InitDefaults() {
  InitDefaultsEgoLayer();
}
}  // namespace protobuf_ego_5flayer_2eproto
namespace ndm_proto {
class EgoLayer;
class EgoLayerDefaultTypeInternal;
extern EgoLayerDefaultTypeInternal _EgoLayer_default_instance_;
}  // namespace ndm_proto
namespace ndm_proto {

// ===================================================================

class EgoLayer : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:ndm_proto.EgoLayer) */ {
 public:
  EgoLayer();
  virtual ~EgoLayer();

  EgoLayer(const EgoLayer& from);

  inline EgoLayer& operator=(const EgoLayer& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  EgoLayer(EgoLayer&& from) noexcept
    : EgoLayer() {
    *this = ::std::move(from);
  }

  inline EgoLayer& operator=(EgoLayer&& from) noexcept {
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
  static const EgoLayer& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const EgoLayer* internal_default_instance() {
    return reinterpret_cast<const EgoLayer*>(
               &_EgoLayer_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(EgoLayer* other);
  friend void swap(EgoLayer& a, EgoLayer& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline EgoLayer* New() const PROTOBUF_FINAL { return New(NULL); }

  EgoLayer* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const EgoLayer& from);
  void MergeFrom(const EgoLayer& from);
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
  void InternalSwap(EgoLayer* other);
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

  // required string id = 1;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 1;
  const ::std::string& id() const;
  void set_id(const ::std::string& value);
  #if LANG_CXX11
  void set_id(::std::string&& value);
  #endif
  void set_id(const char* value);
  void set_id(const char* value, size_t size);
  ::std::string* mutable_id();
  ::std::string* release_id();
  void set_allocated_id(::std::string* id);

  // optional string licence_plate = 6;
  bool has_licence_plate() const;
  void clear_licence_plate();
  static const int kLicencePlateFieldNumber = 6;
  const ::std::string& licence_plate() const;
  void set_licence_plate(const ::std::string& value);
  #if LANG_CXX11
  void set_licence_plate(::std::string&& value);
  #endif
  void set_licence_plate(const char* value);
  void set_licence_plate(const char* value, size_t size);
  ::std::string* mutable_licence_plate();
  ::std::string* release_licence_plate();
  void set_allocated_licence_plate(::std::string* licence_plate);

  // optional .ndm_proto.VehicleType vehicle_type = 3;
  bool has_vehicle_type() const;
  void clear_vehicle_type();
  static const int kVehicleTypeFieldNumber = 3;
  const ::ndm_proto::VehicleType& vehicle_type() const;
  ::ndm_proto::VehicleType* release_vehicle_type();
  ::ndm_proto::VehicleType* mutable_vehicle_type();
  void set_allocated_vehicle_type(::ndm_proto::VehicleType* vehicle_type);

  // optional .ndm_proto.Point lwh = 4;
  bool has_lwh() const;
  void clear_lwh();
  static const int kLwhFieldNumber = 4;
  const ::ndm_proto::Point& lwh() const;
  ::ndm_proto::Point* release_lwh();
  ::ndm_proto::Point* mutable_lwh();
  void set_allocated_lwh(::ndm_proto::Point* lwh);

  // optional .ndm_proto.Trajectory relative_traj = 7;
  bool has_relative_traj() const;
  void clear_relative_traj();
  static const int kRelativeTrajFieldNumber = 7;
  const ::ndm_proto::Trajectory& relative_traj() const;
  ::ndm_proto::Trajectory* release_relative_traj();
  ::ndm_proto::Trajectory* mutable_relative_traj();
  void set_allocated_relative_traj(::ndm_proto::Trajectory* relative_traj);

  // optional .ndm_proto.Trajectory absolute_traj = 8;
  bool has_absolute_traj() const;
  void clear_absolute_traj();
  static const int kAbsoluteTrajFieldNumber = 8;
  const ::ndm_proto::Trajectory& absolute_traj() const;
  ::ndm_proto::Trajectory* release_absolute_traj();
  ::ndm_proto::Trajectory* mutable_absolute_traj();
  void set_allocated_absolute_traj(::ndm_proto::Trajectory* absolute_traj);

  // required uint64 stamp = 2;
  bool has_stamp() const;
  void clear_stamp();
  static const int kStampFieldNumber = 2;
  ::google::protobuf::uint64 stamp() const;
  void set_stamp(::google::protobuf::uint64 value);

  // optional float weight = 5;
  bool has_weight() const;
  void clear_weight();
  static const int kWeightFieldNumber = 5;
  float weight() const;
  void set_weight(float value);

  // optional bool fixed = 9;
  bool has_fixed() const;
  void clear_fixed();
  static const int kFixedFieldNumber = 9;
  bool fixed() const;
  void set_fixed(bool value);

  // @@protoc_insertion_point(class_scope:ndm_proto.EgoLayer)
 private:
  void set_has_id();
  void clear_has_id();
  void set_has_stamp();
  void clear_has_stamp();
  void set_has_vehicle_type();
  void clear_has_vehicle_type();
  void set_has_lwh();
  void clear_has_lwh();
  void set_has_weight();
  void clear_has_weight();
  void set_has_licence_plate();
  void clear_has_licence_plate();
  void set_has_relative_traj();
  void clear_has_relative_traj();
  void set_has_absolute_traj();
  void clear_has_absolute_traj();
  void set_has_fixed();
  void clear_has_fixed();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr id_;
  ::google::protobuf::internal::ArenaStringPtr licence_plate_;
  ::ndm_proto::VehicleType* vehicle_type_;
  ::ndm_proto::Point* lwh_;
  ::ndm_proto::Trajectory* relative_traj_;
  ::ndm_proto::Trajectory* absolute_traj_;
  ::google::protobuf::uint64 stamp_;
  float weight_;
  bool fixed_;
  friend struct ::protobuf_ego_5flayer_2eproto::TableStruct;
  friend void ::protobuf_ego_5flayer_2eproto::InitDefaultsEgoLayerImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// EgoLayer

// required string id = 1;
inline bool EgoLayer::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void EgoLayer::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void EgoLayer::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void EgoLayer::clear_id() {
  id_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_id();
}
inline const ::std::string& EgoLayer::id() const {
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.id)
  return id_.GetNoArena();
}
inline void EgoLayer::set_id(const ::std::string& value) {
  set_has_id();
  id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:ndm_proto.EgoLayer.id)
}
#if LANG_CXX11
inline void EgoLayer::set_id(::std::string&& value) {
  set_has_id();
  id_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:ndm_proto.EgoLayer.id)
}
#endif
inline void EgoLayer::set_id(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_id();
  id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:ndm_proto.EgoLayer.id)
}
inline void EgoLayer::set_id(const char* value, size_t size) {
  set_has_id();
  id_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:ndm_proto.EgoLayer.id)
}
inline ::std::string* EgoLayer::mutable_id() {
  set_has_id();
  // @@protoc_insertion_point(field_mutable:ndm_proto.EgoLayer.id)
  return id_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* EgoLayer::release_id() {
  // @@protoc_insertion_point(field_release:ndm_proto.EgoLayer.id)
  clear_has_id();
  return id_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void EgoLayer::set_allocated_id(::std::string* id) {
  if (id != NULL) {
    set_has_id();
  } else {
    clear_has_id();
  }
  id_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), id);
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.EgoLayer.id)
}

// required uint64 stamp = 2;
inline bool EgoLayer::has_stamp() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void EgoLayer::set_has_stamp() {
  _has_bits_[0] |= 0x00000040u;
}
inline void EgoLayer::clear_has_stamp() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void EgoLayer::clear_stamp() {
  stamp_ = GOOGLE_ULONGLONG(0);
  clear_has_stamp();
}
inline ::google::protobuf::uint64 EgoLayer::stamp() const {
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.stamp)
  return stamp_;
}
inline void EgoLayer::set_stamp(::google::protobuf::uint64 value) {
  set_has_stamp();
  stamp_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.EgoLayer.stamp)
}

// optional .ndm_proto.VehicleType vehicle_type = 3;
inline bool EgoLayer::has_vehicle_type() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void EgoLayer::set_has_vehicle_type() {
  _has_bits_[0] |= 0x00000004u;
}
inline void EgoLayer::clear_has_vehicle_type() {
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::ndm_proto::VehicleType& EgoLayer::vehicle_type() const {
  const ::ndm_proto::VehicleType* p = vehicle_type_;
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.vehicle_type)
  return p != NULL ? *p : *reinterpret_cast<const ::ndm_proto::VehicleType*>(
      &::ndm_proto::_VehicleType_default_instance_);
}
inline ::ndm_proto::VehicleType* EgoLayer::release_vehicle_type() {
  // @@protoc_insertion_point(field_release:ndm_proto.EgoLayer.vehicle_type)
  clear_has_vehicle_type();
  ::ndm_proto::VehicleType* temp = vehicle_type_;
  vehicle_type_ = NULL;
  return temp;
}
inline ::ndm_proto::VehicleType* EgoLayer::mutable_vehicle_type() {
  set_has_vehicle_type();
  if (vehicle_type_ == NULL) {
    vehicle_type_ = new ::ndm_proto::VehicleType;
  }
  // @@protoc_insertion_point(field_mutable:ndm_proto.EgoLayer.vehicle_type)
  return vehicle_type_;
}
inline void EgoLayer::set_allocated_vehicle_type(::ndm_proto::VehicleType* vehicle_type) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(vehicle_type_);
  }
  if (vehicle_type) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      vehicle_type = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, vehicle_type, submessage_arena);
    }
    set_has_vehicle_type();
  } else {
    clear_has_vehicle_type();
  }
  vehicle_type_ = vehicle_type;
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.EgoLayer.vehicle_type)
}

// optional .ndm_proto.Point lwh = 4;
inline bool EgoLayer::has_lwh() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void EgoLayer::set_has_lwh() {
  _has_bits_[0] |= 0x00000008u;
}
inline void EgoLayer::clear_has_lwh() {
  _has_bits_[0] &= ~0x00000008u;
}
inline const ::ndm_proto::Point& EgoLayer::lwh() const {
  const ::ndm_proto::Point* p = lwh_;
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.lwh)
  return p != NULL ? *p : *reinterpret_cast<const ::ndm_proto::Point*>(
      &::ndm_proto::_Point_default_instance_);
}
inline ::ndm_proto::Point* EgoLayer::release_lwh() {
  // @@protoc_insertion_point(field_release:ndm_proto.EgoLayer.lwh)
  clear_has_lwh();
  ::ndm_proto::Point* temp = lwh_;
  lwh_ = NULL;
  return temp;
}
inline ::ndm_proto::Point* EgoLayer::mutable_lwh() {
  set_has_lwh();
  if (lwh_ == NULL) {
    lwh_ = new ::ndm_proto::Point;
  }
  // @@protoc_insertion_point(field_mutable:ndm_proto.EgoLayer.lwh)
  return lwh_;
}
inline void EgoLayer::set_allocated_lwh(::ndm_proto::Point* lwh) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(lwh_);
  }
  if (lwh) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      lwh = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, lwh, submessage_arena);
    }
    set_has_lwh();
  } else {
    clear_has_lwh();
  }
  lwh_ = lwh;
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.EgoLayer.lwh)
}

// optional float weight = 5;
inline bool EgoLayer::has_weight() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void EgoLayer::set_has_weight() {
  _has_bits_[0] |= 0x00000080u;
}
inline void EgoLayer::clear_has_weight() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void EgoLayer::clear_weight() {
  weight_ = 0;
  clear_has_weight();
}
inline float EgoLayer::weight() const {
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.weight)
  return weight_;
}
inline void EgoLayer::set_weight(float value) {
  set_has_weight();
  weight_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.EgoLayer.weight)
}

// optional string licence_plate = 6;
inline bool EgoLayer::has_licence_plate() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void EgoLayer::set_has_licence_plate() {
  _has_bits_[0] |= 0x00000002u;
}
inline void EgoLayer::clear_has_licence_plate() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void EgoLayer::clear_licence_plate() {
  licence_plate_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_licence_plate();
}
inline const ::std::string& EgoLayer::licence_plate() const {
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.licence_plate)
  return licence_plate_.GetNoArena();
}
inline void EgoLayer::set_licence_plate(const ::std::string& value) {
  set_has_licence_plate();
  licence_plate_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:ndm_proto.EgoLayer.licence_plate)
}
#if LANG_CXX11
inline void EgoLayer::set_licence_plate(::std::string&& value) {
  set_has_licence_plate();
  licence_plate_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:ndm_proto.EgoLayer.licence_plate)
}
#endif
inline void EgoLayer::set_licence_plate(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_licence_plate();
  licence_plate_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:ndm_proto.EgoLayer.licence_plate)
}
inline void EgoLayer::set_licence_plate(const char* value, size_t size) {
  set_has_licence_plate();
  licence_plate_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:ndm_proto.EgoLayer.licence_plate)
}
inline ::std::string* EgoLayer::mutable_licence_plate() {
  set_has_licence_plate();
  // @@protoc_insertion_point(field_mutable:ndm_proto.EgoLayer.licence_plate)
  return licence_plate_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* EgoLayer::release_licence_plate() {
  // @@protoc_insertion_point(field_release:ndm_proto.EgoLayer.licence_plate)
  clear_has_licence_plate();
  return licence_plate_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void EgoLayer::set_allocated_licence_plate(::std::string* licence_plate) {
  if (licence_plate != NULL) {
    set_has_licence_plate();
  } else {
    clear_has_licence_plate();
  }
  licence_plate_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), licence_plate);
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.EgoLayer.licence_plate)
}

// optional .ndm_proto.Trajectory relative_traj = 7;
inline bool EgoLayer::has_relative_traj() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void EgoLayer::set_has_relative_traj() {
  _has_bits_[0] |= 0x00000010u;
}
inline void EgoLayer::clear_has_relative_traj() {
  _has_bits_[0] &= ~0x00000010u;
}
inline const ::ndm_proto::Trajectory& EgoLayer::relative_traj() const {
  const ::ndm_proto::Trajectory* p = relative_traj_;
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.relative_traj)
  return p != NULL ? *p : *reinterpret_cast<const ::ndm_proto::Trajectory*>(
      &::ndm_proto::_Trajectory_default_instance_);
}
inline ::ndm_proto::Trajectory* EgoLayer::release_relative_traj() {
  // @@protoc_insertion_point(field_release:ndm_proto.EgoLayer.relative_traj)
  clear_has_relative_traj();
  ::ndm_proto::Trajectory* temp = relative_traj_;
  relative_traj_ = NULL;
  return temp;
}
inline ::ndm_proto::Trajectory* EgoLayer::mutable_relative_traj() {
  set_has_relative_traj();
  if (relative_traj_ == NULL) {
    relative_traj_ = new ::ndm_proto::Trajectory;
  }
  // @@protoc_insertion_point(field_mutable:ndm_proto.EgoLayer.relative_traj)
  return relative_traj_;
}
inline void EgoLayer::set_allocated_relative_traj(::ndm_proto::Trajectory* relative_traj) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(relative_traj_);
  }
  if (relative_traj) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      relative_traj = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, relative_traj, submessage_arena);
    }
    set_has_relative_traj();
  } else {
    clear_has_relative_traj();
  }
  relative_traj_ = relative_traj;
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.EgoLayer.relative_traj)
}

// optional .ndm_proto.Trajectory absolute_traj = 8;
inline bool EgoLayer::has_absolute_traj() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void EgoLayer::set_has_absolute_traj() {
  _has_bits_[0] |= 0x00000020u;
}
inline void EgoLayer::clear_has_absolute_traj() {
  _has_bits_[0] &= ~0x00000020u;
}
inline const ::ndm_proto::Trajectory& EgoLayer::absolute_traj() const {
  const ::ndm_proto::Trajectory* p = absolute_traj_;
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.absolute_traj)
  return p != NULL ? *p : *reinterpret_cast<const ::ndm_proto::Trajectory*>(
      &::ndm_proto::_Trajectory_default_instance_);
}
inline ::ndm_proto::Trajectory* EgoLayer::release_absolute_traj() {
  // @@protoc_insertion_point(field_release:ndm_proto.EgoLayer.absolute_traj)
  clear_has_absolute_traj();
  ::ndm_proto::Trajectory* temp = absolute_traj_;
  absolute_traj_ = NULL;
  return temp;
}
inline ::ndm_proto::Trajectory* EgoLayer::mutable_absolute_traj() {
  set_has_absolute_traj();
  if (absolute_traj_ == NULL) {
    absolute_traj_ = new ::ndm_proto::Trajectory;
  }
  // @@protoc_insertion_point(field_mutable:ndm_proto.EgoLayer.absolute_traj)
  return absolute_traj_;
}
inline void EgoLayer::set_allocated_absolute_traj(::ndm_proto::Trajectory* absolute_traj) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(absolute_traj_);
  }
  if (absolute_traj) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      absolute_traj = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, absolute_traj, submessage_arena);
    }
    set_has_absolute_traj();
  } else {
    clear_has_absolute_traj();
  }
  absolute_traj_ = absolute_traj;
  // @@protoc_insertion_point(field_set_allocated:ndm_proto.EgoLayer.absolute_traj)
}

// optional bool fixed = 9;
inline bool EgoLayer::has_fixed() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void EgoLayer::set_has_fixed() {
  _has_bits_[0] |= 0x00000100u;
}
inline void EgoLayer::clear_has_fixed() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void EgoLayer::clear_fixed() {
  fixed_ = false;
  clear_has_fixed();
}
inline bool EgoLayer::fixed() const {
  // @@protoc_insertion_point(field_get:ndm_proto.EgoLayer.fixed)
  return fixed_;
}
inline void EgoLayer::set_fixed(bool value) {
  set_has_fixed();
  fixed_ = value;
  // @@protoc_insertion_point(field_set:ndm_proto.EgoLayer.fixed)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace ndm_proto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_ego_5flayer_2eproto__INCLUDED
