// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: confidence/confidence.proto

#include "confidence/confidence.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace RoadPB {
constexpr DataQuality::DataQuality(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : id_(nullptr)
  , feat_id_(nullptr)
  , data_source_(0)
  , data_weather_(0)
  , sensor_type_(0)
  , work_manner_(0)
  , create_time_(int64_t{0})
  , confidence_(0){}
struct DataQualityDefaultTypeInternal {
  constexpr DataQualityDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~DataQualityDefaultTypeInternal() {}
  union {
    DataQuality _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT DataQualityDefaultTypeInternal _DataQuality_default_instance_;
}  // namespace RoadPB
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_confidence_2fconfidence_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_confidence_2fconfidence_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_confidence_2fconfidence_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_confidence_2fconfidence_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, id_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, data_source_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, data_weather_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, sensor_type_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, work_manner_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, create_time_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, confidence_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::DataQuality, feat_id_),
  0,
  2,
  3,
  4,
  5,
  6,
  7,
  1,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 13, sizeof(::RoadPB::DataQuality)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::RoadPB::_DataQuality_default_instance_),
};

const char descriptor_table_protodef_confidence_2fconfidence_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\033confidence/confidence.proto\022\006RoadPB\032\027m"
  "etadata/metadata.proto\"\316\001\n\013DataQuality\022\035"
  "\n\002id\030\001 \001(\0132\021.RoadPB.FeatureID\022\023\n\013data_so"
  "urce\030\002 \001(\005\022\024\n\014data_weather\030\003 \001(\005\022\023\n\013sens"
  "or_type\030\004 \001(\005\022\023\n\013work_manner\030\005 \001(\005\022\023\n\013cr"
  "eate_time\030\006 \001(\003\022\022\n\nconfidence\030\007 \001(\005\022\"\n\007f"
  "eat_id\030\010 \001(\0132\021.RoadPB.FeatureID"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_confidence_2fconfidence_2eproto_deps[1] = {
  &::descriptor_table_metadata_2fmetadata_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_confidence_2fconfidence_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_confidence_2fconfidence_2eproto = {
  false, false, 271, descriptor_table_protodef_confidence_2fconfidence_2eproto, "confidence/confidence.proto", 
  &descriptor_table_confidence_2fconfidence_2eproto_once, descriptor_table_confidence_2fconfidence_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_confidence_2fconfidence_2eproto::offsets,
  file_level_metadata_confidence_2fconfidence_2eproto, file_level_enum_descriptors_confidence_2fconfidence_2eproto, file_level_service_descriptors_confidence_2fconfidence_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_confidence_2fconfidence_2eproto_getter() {
  return &descriptor_table_confidence_2fconfidence_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_confidence_2fconfidence_2eproto(&descriptor_table_confidence_2fconfidence_2eproto);
namespace RoadPB {

// ===================================================================

class DataQuality::_Internal {
 public:
  using HasBits = decltype(std::declval<DataQuality>()._has_bits_);
  static const ::RoadPB::FeatureID& id(const DataQuality* msg);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_data_source(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_data_weather(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_sensor_type(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_work_manner(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_create_time(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_confidence(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static const ::RoadPB::FeatureID& feat_id(const DataQuality* msg);
  static void set_has_feat_id(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::RoadPB::FeatureID&
DataQuality::_Internal::id(const DataQuality* msg) {
  return *msg->id_;
}
const ::RoadPB::FeatureID&
DataQuality::_Internal::feat_id(const DataQuality* msg) {
  return *msg->feat_id_;
}
void DataQuality::clear_id() {
  if (id_ != nullptr) id_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void DataQuality::clear_feat_id() {
  if (feat_id_ != nullptr) feat_id_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
DataQuality::DataQuality(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:RoadPB.DataQuality)
}
DataQuality::DataQuality(const DataQuality& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_id()) {
    id_ = new ::RoadPB::FeatureID(*from.id_);
  } else {
    id_ = nullptr;
  }
  if (from._internal_has_feat_id()) {
    feat_id_ = new ::RoadPB::FeatureID(*from.feat_id_);
  } else {
    feat_id_ = nullptr;
  }
  ::memcpy(&data_source_, &from.data_source_,
    static_cast<size_t>(reinterpret_cast<char*>(&confidence_) -
    reinterpret_cast<char*>(&data_source_)) + sizeof(confidence_));
  // @@protoc_insertion_point(copy_constructor:RoadPB.DataQuality)
}

inline void DataQuality::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&id_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&confidence_) -
    reinterpret_cast<char*>(&id_)) + sizeof(confidence_));
}

DataQuality::~DataQuality() {
  // @@protoc_insertion_point(destructor:RoadPB.DataQuality)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void DataQuality::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete feat_id_;
}

void DataQuality::ArenaDtor(void* object) {
  DataQuality* _this = reinterpret_cast< DataQuality* >(object);
  (void)_this;
}
void DataQuality::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void DataQuality::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void DataQuality::Clear() {
// @@protoc_insertion_point(message_clear_start:RoadPB.DataQuality)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != nullptr);
      id_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(feat_id_ != nullptr);
      feat_id_->Clear();
    }
  }
  if (cached_has_bits & 0x000000fcu) {
    ::memset(&data_source_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&confidence_) -
        reinterpret_cast<char*>(&data_source_)) + sizeof(confidence_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* DataQuality::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .RoadPB.FeatureID id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_id(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 data_source = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_data_source(&has_bits);
          data_source_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 data_weather = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_data_weather(&has_bits);
          data_weather_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 sensor_type = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          _Internal::set_has_sensor_type(&has_bits);
          sensor_type_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 work_manner = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          _Internal::set_has_work_manner(&has_bits);
          work_manner_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int64 create_time = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 48)) {
          _Internal::set_has_create_time(&has_bits);
          create_time_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 confidence = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 56)) {
          _Internal::set_has_confidence(&has_bits);
          confidence_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .RoadPB.FeatureID feat_id = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 66)) {
          ptr = ctx->ParseMessage(_internal_mutable_feat_id(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag == 0) || ((tag & 7) == 4)) {
          CHK_(ptr);
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* DataQuality::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:RoadPB.DataQuality)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .RoadPB.FeatureID id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::id(this), target, stream);
  }

  // optional int32 data_source = 2;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_data_source(), target);
  }

  // optional int32 data_weather = 3;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(3, this->_internal_data_weather(), target);
  }

  // optional int32 sensor_type = 4;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(4, this->_internal_sensor_type(), target);
  }

  // optional int32 work_manner = 5;
  if (cached_has_bits & 0x00000020u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(5, this->_internal_work_manner(), target);
  }

  // optional int64 create_time = 6;
  if (cached_has_bits & 0x00000040u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(6, this->_internal_create_time(), target);
  }

  // optional int32 confidence = 7;
  if (cached_has_bits & 0x00000080u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(7, this->_internal_confidence(), target);
  }

  // optional .RoadPB.FeatureID feat_id = 8;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        8, _Internal::feat_id(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:RoadPB.DataQuality)
  return target;
}

size_t DataQuality::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:RoadPB.DataQuality)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional .RoadPB.FeatureID id = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *id_);
    }

    // optional .RoadPB.FeatureID feat_id = 8;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *feat_id_);
    }

    // optional int32 data_source = 2;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_data_source());
    }

    // optional int32 data_weather = 3;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_data_weather());
    }

    // optional int32 sensor_type = 4;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_sensor_type());
    }

    // optional int32 work_manner = 5;
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_work_manner());
    }

    // optional int64 create_time = 6;
    if (cached_has_bits & 0x00000040u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
          this->_internal_create_time());
    }

    // optional int32 confidence = 7;
    if (cached_has_bits & 0x00000080u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_confidence());
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData DataQuality::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    DataQuality::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*DataQuality::GetClassData() const { return &_class_data_; }

void DataQuality::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to,
                      const ::PROTOBUF_NAMESPACE_ID::Message&from) {
  static_cast<DataQuality *>(to)->MergeFrom(
      static_cast<const DataQuality &>(from));
}


void DataQuality::MergeFrom(const DataQuality& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:RoadPB.DataQuality)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_id()->::RoadPB::FeatureID::MergeFrom(from._internal_id());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_feat_id()->::RoadPB::FeatureID::MergeFrom(from._internal_feat_id());
    }
    if (cached_has_bits & 0x00000004u) {
      data_source_ = from.data_source_;
    }
    if (cached_has_bits & 0x00000008u) {
      data_weather_ = from.data_weather_;
    }
    if (cached_has_bits & 0x00000010u) {
      sensor_type_ = from.sensor_type_;
    }
    if (cached_has_bits & 0x00000020u) {
      work_manner_ = from.work_manner_;
    }
    if (cached_has_bits & 0x00000040u) {
      create_time_ = from.create_time_;
    }
    if (cached_has_bits & 0x00000080u) {
      confidence_ = from.confidence_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void DataQuality::CopyFrom(const DataQuality& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:RoadPB.DataQuality)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DataQuality::IsInitialized() const {
  return true;
}

void DataQuality::InternalSwap(DataQuality* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(DataQuality, confidence_)
      + sizeof(DataQuality::confidence_)
      - PROTOBUF_FIELD_OFFSET(DataQuality, id_)>(
          reinterpret_cast<char*>(&id_),
          reinterpret_cast<char*>(&other->id_));
}

::PROTOBUF_NAMESPACE_ID::Metadata DataQuality::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_confidence_2fconfidence_2eproto_getter, &descriptor_table_confidence_2fconfidence_2eproto_once,
      file_level_metadata_confidence_2fconfidence_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace RoadPB
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::RoadPB::DataQuality* Arena::CreateMaybeMessage< ::RoadPB::DataQuality >(Arena* arena) {
  return Arena::CreateMessageInternal< ::RoadPB::DataQuality >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
