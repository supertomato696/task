// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: confidence/confidence.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_confidence_2fconfidence_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_confidence_2fconfidence_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3017000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3017003 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "metadata/metadata.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_confidence_2fconfidence_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_confidence_2fconfidence_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_confidence_2fconfidence_2eproto;
namespace RoadPB {
class DataQuality;
struct DataQualityDefaultTypeInternal;
extern DataQualityDefaultTypeInternal _DataQuality_default_instance_;
}  // namespace RoadPB
PROTOBUF_NAMESPACE_OPEN
template<> ::RoadPB::DataQuality* Arena::CreateMaybeMessage<::RoadPB::DataQuality>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace RoadPB {

// ===================================================================

class DataQuality final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:RoadPB.DataQuality) */ {
 public:
  inline DataQuality() : DataQuality(nullptr) {}
  ~DataQuality() override;
  explicit constexpr DataQuality(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  DataQuality(const DataQuality& from);
  DataQuality(DataQuality&& from) noexcept
    : DataQuality() {
    *this = ::std::move(from);
  }

  inline DataQuality& operator=(const DataQuality& from) {
    CopyFrom(from);
    return *this;
  }
  inline DataQuality& operator=(DataQuality&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const DataQuality& default_instance() {
    return *internal_default_instance();
  }
  static inline const DataQuality* internal_default_instance() {
    return reinterpret_cast<const DataQuality*>(
               &_DataQuality_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DataQuality& a, DataQuality& b) {
    a.Swap(&b);
  }
  inline void Swap(DataQuality* other) {
    if (other == this) return;
    if (GetOwningArena() == other->GetOwningArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(DataQuality* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline DataQuality* New() const final {
    return new DataQuality();
  }

  DataQuality* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<DataQuality>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const DataQuality& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const DataQuality& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to, const ::PROTOBUF_NAMESPACE_ID::Message&from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(DataQuality* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "RoadPB.DataQuality";
  }
  protected:
  explicit DataQuality(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kIdFieldNumber = 1,
    kFeatIdFieldNumber = 8,
    kDataSourceFieldNumber = 2,
    kDataWeatherFieldNumber = 3,
    kSensorTypeFieldNumber = 4,
    kWorkMannerFieldNumber = 5,
    kCreateTimeFieldNumber = 6,
    kConfidenceFieldNumber = 7,
  };
  // optional .RoadPB.FeatureID id = 1;
  bool has_id() const;
  private:
  bool _internal_has_id() const;
  public:
  void clear_id();
  const ::RoadPB::FeatureID& id() const;
  PROTOBUF_MUST_USE_RESULT ::RoadPB::FeatureID* release_id();
  ::RoadPB::FeatureID* mutable_id();
  void set_allocated_id(::RoadPB::FeatureID* id);
  private:
  const ::RoadPB::FeatureID& _internal_id() const;
  ::RoadPB::FeatureID* _internal_mutable_id();
  public:
  void unsafe_arena_set_allocated_id(
      ::RoadPB::FeatureID* id);
  ::RoadPB::FeatureID* unsafe_arena_release_id();

  // optional .RoadPB.FeatureID feat_id = 8;
  bool has_feat_id() const;
  private:
  bool _internal_has_feat_id() const;
  public:
  void clear_feat_id();
  const ::RoadPB::FeatureID& feat_id() const;
  PROTOBUF_MUST_USE_RESULT ::RoadPB::FeatureID* release_feat_id();
  ::RoadPB::FeatureID* mutable_feat_id();
  void set_allocated_feat_id(::RoadPB::FeatureID* feat_id);
  private:
  const ::RoadPB::FeatureID& _internal_feat_id() const;
  ::RoadPB::FeatureID* _internal_mutable_feat_id();
  public:
  void unsafe_arena_set_allocated_feat_id(
      ::RoadPB::FeatureID* feat_id);
  ::RoadPB::FeatureID* unsafe_arena_release_feat_id();

  // optional int32 data_source = 2;
  bool has_data_source() const;
  private:
  bool _internal_has_data_source() const;
  public:
  void clear_data_source();
  ::PROTOBUF_NAMESPACE_ID::int32 data_source() const;
  void set_data_source(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_data_source() const;
  void _internal_set_data_source(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional int32 data_weather = 3;
  bool has_data_weather() const;
  private:
  bool _internal_has_data_weather() const;
  public:
  void clear_data_weather();
  ::PROTOBUF_NAMESPACE_ID::int32 data_weather() const;
  void set_data_weather(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_data_weather() const;
  void _internal_set_data_weather(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional int32 sensor_type = 4;
  bool has_sensor_type() const;
  private:
  bool _internal_has_sensor_type() const;
  public:
  void clear_sensor_type();
  ::PROTOBUF_NAMESPACE_ID::int32 sensor_type() const;
  void set_sensor_type(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_sensor_type() const;
  void _internal_set_sensor_type(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional int32 work_manner = 5;
  bool has_work_manner() const;
  private:
  bool _internal_has_work_manner() const;
  public:
  void clear_work_manner();
  ::PROTOBUF_NAMESPACE_ID::int32 work_manner() const;
  void set_work_manner(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_work_manner() const;
  void _internal_set_work_manner(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // optional int64 create_time = 6;
  bool has_create_time() const;
  private:
  bool _internal_has_create_time() const;
  public:
  void clear_create_time();
  ::PROTOBUF_NAMESPACE_ID::int64 create_time() const;
  void set_create_time(::PROTOBUF_NAMESPACE_ID::int64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int64 _internal_create_time() const;
  void _internal_set_create_time(::PROTOBUF_NAMESPACE_ID::int64 value);
  public:

  // optional int32 confidence = 7;
  bool has_confidence() const;
  private:
  bool _internal_has_confidence() const;
  public:
  void clear_confidence();
  ::PROTOBUF_NAMESPACE_ID::int32 confidence() const;
  void set_confidence(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_confidence() const;
  void _internal_set_confidence(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:RoadPB.DataQuality)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::RoadPB::FeatureID* id_;
  ::RoadPB::FeatureID* feat_id_;
  ::PROTOBUF_NAMESPACE_ID::int32 data_source_;
  ::PROTOBUF_NAMESPACE_ID::int32 data_weather_;
  ::PROTOBUF_NAMESPACE_ID::int32 sensor_type_;
  ::PROTOBUF_NAMESPACE_ID::int32 work_manner_;
  ::PROTOBUF_NAMESPACE_ID::int64 create_time_;
  ::PROTOBUF_NAMESPACE_ID::int32 confidence_;
  friend struct ::TableStruct_confidence_2fconfidence_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DataQuality

// optional .RoadPB.FeatureID id = 1;
inline bool DataQuality::_internal_has_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || id_ != nullptr);
  return value;
}
inline bool DataQuality::has_id() const {
  return _internal_has_id();
}
inline const ::RoadPB::FeatureID& DataQuality::_internal_id() const {
  const ::RoadPB::FeatureID* p = id_;
  return p != nullptr ? *p : reinterpret_cast<const ::RoadPB::FeatureID&>(
      ::RoadPB::_FeatureID_default_instance_);
}
inline const ::RoadPB::FeatureID& DataQuality::id() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.id)
  return _internal_id();
}
inline void DataQuality::unsafe_arena_set_allocated_id(
    ::RoadPB::FeatureID* id) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  id_ = id;
  if (id) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:RoadPB.DataQuality.id)
}
inline ::RoadPB::FeatureID* DataQuality::release_id() {
  _has_bits_[0] &= ~0x00000001u;
  ::RoadPB::FeatureID* temp = id_;
  id_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::RoadPB::FeatureID* DataQuality::unsafe_arena_release_id() {
  // @@protoc_insertion_point(field_release:RoadPB.DataQuality.id)
  _has_bits_[0] &= ~0x00000001u;
  ::RoadPB::FeatureID* temp = id_;
  id_ = nullptr;
  return temp;
}
inline ::RoadPB::FeatureID* DataQuality::_internal_mutable_id() {
  _has_bits_[0] |= 0x00000001u;
  if (id_ == nullptr) {
    auto* p = CreateMaybeMessage<::RoadPB::FeatureID>(GetArenaForAllocation());
    id_ = p;
  }
  return id_;
}
inline ::RoadPB::FeatureID* DataQuality::mutable_id() {
  ::RoadPB::FeatureID* _msg = _internal_mutable_id();
  // @@protoc_insertion_point(field_mutable:RoadPB.DataQuality.id)
  return _msg;
}
inline void DataQuality::set_allocated_id(::RoadPB::FeatureID* id) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  if (id) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id));
    if (message_arena != submessage_arena) {
      id = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, id, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  id_ = id;
  // @@protoc_insertion_point(field_set_allocated:RoadPB.DataQuality.id)
}

// optional int32 data_source = 2;
inline bool DataQuality::_internal_has_data_source() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool DataQuality::has_data_source() const {
  return _internal_has_data_source();
}
inline void DataQuality::clear_data_source() {
  data_source_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::_internal_data_source() const {
  return data_source_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::data_source() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.data_source)
  return _internal_data_source();
}
inline void DataQuality::_internal_set_data_source(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000004u;
  data_source_ = value;
}
inline void DataQuality::set_data_source(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_data_source(value);
  // @@protoc_insertion_point(field_set:RoadPB.DataQuality.data_source)
}

// optional int32 data_weather = 3;
inline bool DataQuality::_internal_has_data_weather() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool DataQuality::has_data_weather() const {
  return _internal_has_data_weather();
}
inline void DataQuality::clear_data_weather() {
  data_weather_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::_internal_data_weather() const {
  return data_weather_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::data_weather() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.data_weather)
  return _internal_data_weather();
}
inline void DataQuality::_internal_set_data_weather(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000008u;
  data_weather_ = value;
}
inline void DataQuality::set_data_weather(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_data_weather(value);
  // @@protoc_insertion_point(field_set:RoadPB.DataQuality.data_weather)
}

// optional int32 sensor_type = 4;
inline bool DataQuality::_internal_has_sensor_type() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool DataQuality::has_sensor_type() const {
  return _internal_has_sensor_type();
}
inline void DataQuality::clear_sensor_type() {
  sensor_type_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::_internal_sensor_type() const {
  return sensor_type_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::sensor_type() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.sensor_type)
  return _internal_sensor_type();
}
inline void DataQuality::_internal_set_sensor_type(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000010u;
  sensor_type_ = value;
}
inline void DataQuality::set_sensor_type(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_sensor_type(value);
  // @@protoc_insertion_point(field_set:RoadPB.DataQuality.sensor_type)
}

// optional int32 work_manner = 5;
inline bool DataQuality::_internal_has_work_manner() const {
  bool value = (_has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool DataQuality::has_work_manner() const {
  return _internal_has_work_manner();
}
inline void DataQuality::clear_work_manner() {
  work_manner_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::_internal_work_manner() const {
  return work_manner_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::work_manner() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.work_manner)
  return _internal_work_manner();
}
inline void DataQuality::_internal_set_work_manner(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000020u;
  work_manner_ = value;
}
inline void DataQuality::set_work_manner(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_work_manner(value);
  // @@protoc_insertion_point(field_set:RoadPB.DataQuality.work_manner)
}

// optional int64 create_time = 6;
inline bool DataQuality::_internal_has_create_time() const {
  bool value = (_has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool DataQuality::has_create_time() const {
  return _internal_has_create_time();
}
inline void DataQuality::clear_create_time() {
  create_time_ = int64_t{0};
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 DataQuality::_internal_create_time() const {
  return create_time_;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 DataQuality::create_time() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.create_time)
  return _internal_create_time();
}
inline void DataQuality::_internal_set_create_time(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _has_bits_[0] |= 0x00000040u;
  create_time_ = value;
}
inline void DataQuality::set_create_time(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _internal_set_create_time(value);
  // @@protoc_insertion_point(field_set:RoadPB.DataQuality.create_time)
}

// optional int32 confidence = 7;
inline bool DataQuality::_internal_has_confidence() const {
  bool value = (_has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool DataQuality::has_confidence() const {
  return _internal_has_confidence();
}
inline void DataQuality::clear_confidence() {
  confidence_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::_internal_confidence() const {
  return confidence_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 DataQuality::confidence() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.confidence)
  return _internal_confidence();
}
inline void DataQuality::_internal_set_confidence(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000080u;
  confidence_ = value;
}
inline void DataQuality::set_confidence(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_confidence(value);
  // @@protoc_insertion_point(field_set:RoadPB.DataQuality.confidence)
}

// optional .RoadPB.FeatureID feat_id = 8;
inline bool DataQuality::_internal_has_feat_id() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || feat_id_ != nullptr);
  return value;
}
inline bool DataQuality::has_feat_id() const {
  return _internal_has_feat_id();
}
inline const ::RoadPB::FeatureID& DataQuality::_internal_feat_id() const {
  const ::RoadPB::FeatureID* p = feat_id_;
  return p != nullptr ? *p : reinterpret_cast<const ::RoadPB::FeatureID&>(
      ::RoadPB::_FeatureID_default_instance_);
}
inline const ::RoadPB::FeatureID& DataQuality::feat_id() const {
  // @@protoc_insertion_point(field_get:RoadPB.DataQuality.feat_id)
  return _internal_feat_id();
}
inline void DataQuality::unsafe_arena_set_allocated_feat_id(
    ::RoadPB::FeatureID* feat_id) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(feat_id_);
  }
  feat_id_ = feat_id;
  if (feat_id) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:RoadPB.DataQuality.feat_id)
}
inline ::RoadPB::FeatureID* DataQuality::release_feat_id() {
  _has_bits_[0] &= ~0x00000002u;
  ::RoadPB::FeatureID* temp = feat_id_;
  feat_id_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::RoadPB::FeatureID* DataQuality::unsafe_arena_release_feat_id() {
  // @@protoc_insertion_point(field_release:RoadPB.DataQuality.feat_id)
  _has_bits_[0] &= ~0x00000002u;
  ::RoadPB::FeatureID* temp = feat_id_;
  feat_id_ = nullptr;
  return temp;
}
inline ::RoadPB::FeatureID* DataQuality::_internal_mutable_feat_id() {
  _has_bits_[0] |= 0x00000002u;
  if (feat_id_ == nullptr) {
    auto* p = CreateMaybeMessage<::RoadPB::FeatureID>(GetArenaForAllocation());
    feat_id_ = p;
  }
  return feat_id_;
}
inline ::RoadPB::FeatureID* DataQuality::mutable_feat_id() {
  ::RoadPB::FeatureID* _msg = _internal_mutable_feat_id();
  // @@protoc_insertion_point(field_mutable:RoadPB.DataQuality.feat_id)
  return _msg;
}
inline void DataQuality::set_allocated_feat_id(::RoadPB::FeatureID* feat_id) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(feat_id_);
  }
  if (feat_id) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(feat_id));
    if (message_arena != submessage_arena) {
      feat_id = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, feat_id, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  feat_id_ = feat_id;
  // @@protoc_insertion_point(field_set_allocated:RoadPB.DataQuality.feat_id)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace RoadPB

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_confidence_2fconfidence_2eproto
