// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: metadata/metadata.proto

#include "metadata/metadata.pb.h"

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
namespace RoadPB {
class FeatureIDDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<FeatureID> _instance;
} _FeatureID_default_instance_;
}  // namespace RoadPB
static void InitDefaultsscc_info_FeatureID_metadata_2fmetadata_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::RoadPB::_FeatureID_default_instance_;
    new (ptr) ::RoadPB::FeatureID();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::RoadPB::FeatureID::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_FeatureID_metadata_2fmetadata_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_FeatureID_metadata_2fmetadata_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_metadata_2fmetadata_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_metadata_2fmetadata_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_metadata_2fmetadata_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_metadata_2fmetadata_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::RoadPB::FeatureID, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::FeatureID, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::RoadPB::FeatureID, tileid_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::FeatureID, type_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::FeatureID, id_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::FeatureID, version_),
  PROTOBUF_FIELD_OFFSET(::RoadPB::FeatureID, is_deleted_),
  0,
  1,
  2,
  3,
  4,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::RoadPB::FeatureID)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::RoadPB::_FeatureID_default_instance_),
};

const char descriptor_table_protodef_metadata_2fmetadata_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\027metadata/metadata.proto\022\006RoadPB\"\223\002\n\tFe"
  "atureID\022\016\n\006tileid\030\001 \001(\005\022\014\n\004type\030\002 \001(\005\022\n\n"
  "\002id\030\003 \001(\003\022\017\n\007version\030\004 \001(\003\022\022\n\nis_deleted"
  "\030\005 \001(\010\"\266\001\n\010FeatType\022\013\n\007UNKNOWN\020\000\022\010\n\004LINK"
  "\020\001\022\010\n\004NODE\020\002\022\010\n\004LANE\020\003\022\021\n\rLANE_BOUNDARY\020"
  "\004\022\016\n\nLANE_GROUP\020\005\022\014\n\010JUNCTION\020\006\022\020\n\014TRAFF"
  "IC_INFO\020\007\022\020\n\014POSITION_OBJ\020\010\022\021\n\rROAD_BOUN"
  "DARY\020\t\022\016\n\nCONFIDENCE\020\n\022\007\n\003ODD\020\013"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_metadata_2fmetadata_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_metadata_2fmetadata_2eproto_sccs[1] = {
  &scc_info_FeatureID_metadata_2fmetadata_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_metadata_2fmetadata_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_metadata_2fmetadata_2eproto = {
  false, false, descriptor_table_protodef_metadata_2fmetadata_2eproto, "metadata/metadata.proto", 311,
  &descriptor_table_metadata_2fmetadata_2eproto_once, descriptor_table_metadata_2fmetadata_2eproto_sccs, descriptor_table_metadata_2fmetadata_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_metadata_2fmetadata_2eproto::offsets,
  file_level_metadata_metadata_2fmetadata_2eproto, 1, file_level_enum_descriptors_metadata_2fmetadata_2eproto, file_level_service_descriptors_metadata_2fmetadata_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_metadata_2fmetadata_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_metadata_2fmetadata_2eproto)), true);
namespace RoadPB {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* FeatureID_FeatType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_metadata_2fmetadata_2eproto);
  return file_level_enum_descriptors_metadata_2fmetadata_2eproto[0];
}
bool FeatureID_FeatType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr FeatureID_FeatType FeatureID::UNKNOWN;
constexpr FeatureID_FeatType FeatureID::LINK;
constexpr FeatureID_FeatType FeatureID::NODE;
constexpr FeatureID_FeatType FeatureID::LANE;
constexpr FeatureID_FeatType FeatureID::LANE_BOUNDARY;
constexpr FeatureID_FeatType FeatureID::LANE_GROUP;
constexpr FeatureID_FeatType FeatureID::JUNCTION;
constexpr FeatureID_FeatType FeatureID::TRAFFIC_INFO;
constexpr FeatureID_FeatType FeatureID::POSITION_OBJ;
constexpr FeatureID_FeatType FeatureID::ROAD_BOUNDARY;
constexpr FeatureID_FeatType FeatureID::CONFIDENCE;
constexpr FeatureID_FeatType FeatureID::ODD;
constexpr FeatureID_FeatType FeatureID::FeatType_MIN;
constexpr FeatureID_FeatType FeatureID::FeatType_MAX;
constexpr int FeatureID::FeatType_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

void FeatureID::InitAsDefaultInstance() {
}
class FeatureID::_Internal {
 public:
  using HasBits = decltype(std::declval<FeatureID>()._has_bits_);
  static void set_has_tileid(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_type(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_version(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_is_deleted(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
};

FeatureID::FeatureID(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:RoadPB.FeatureID)
}
FeatureID::FeatureID(const FeatureID& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&tileid_, &from.tileid_,
    static_cast<size_t>(reinterpret_cast<char*>(&is_deleted_) -
    reinterpret_cast<char*>(&tileid_)) + sizeof(is_deleted_));
  // @@protoc_insertion_point(copy_constructor:RoadPB.FeatureID)
}

void FeatureID::SharedCtor() {
  ::memset(&tileid_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&is_deleted_) -
      reinterpret_cast<char*>(&tileid_)) + sizeof(is_deleted_));
}

FeatureID::~FeatureID() {
  // @@protoc_insertion_point(destructor:RoadPB.FeatureID)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void FeatureID::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
}

void FeatureID::ArenaDtor(void* object) {
  FeatureID* _this = reinterpret_cast< FeatureID* >(object);
  (void)_this;
}
void FeatureID::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void FeatureID::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const FeatureID& FeatureID::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_FeatureID_metadata_2fmetadata_2eproto.base);
  return *internal_default_instance();
}


void FeatureID::Clear() {
// @@protoc_insertion_point(message_clear_start:RoadPB.FeatureID)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    ::memset(&tileid_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&is_deleted_) -
        reinterpret_cast<char*>(&tileid_)) + sizeof(is_deleted_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* FeatureID::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional int32 tileid = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_tileid(&has_bits);
          tileid_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 type = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_type(&has_bits);
          type_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int64 id = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_id(&has_bits);
          id_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int64 version = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          _Internal::set_has_version(&has_bits);
          version_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool is_deleted = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          _Internal::set_has_is_deleted(&has_bits);
          is_deleted_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
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

::PROTOBUF_NAMESPACE_ID::uint8* FeatureID::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:RoadPB.FeatureID)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 tileid = 1;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(1, this->_internal_tileid(), target);
  }

  // optional int32 type = 2;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_type(), target);
  }

  // optional int64 id = 3;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(3, this->_internal_id(), target);
  }

  // optional int64 version = 4;
  if (cached_has_bits & 0x00000008u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt64ToArray(4, this->_internal_version(), target);
  }

  // optional bool is_deleted = 5;
  if (cached_has_bits & 0x00000010u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(5, this->_internal_is_deleted(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:RoadPB.FeatureID)
  return target;
}

size_t FeatureID::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:RoadPB.FeatureID)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    // optional int32 tileid = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_tileid());
    }

    // optional int32 type = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_type());
    }

    // optional int64 id = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
          this->_internal_id());
    }

    // optional int64 version = 4;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int64Size(
          this->_internal_version());
    }

    // optional bool is_deleted = 5;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 1;
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

void FeatureID::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:RoadPB.FeatureID)
  GOOGLE_DCHECK_NE(&from, this);
  const FeatureID* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<FeatureID>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:RoadPB.FeatureID)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:RoadPB.FeatureID)
    MergeFrom(*source);
  }
}

void FeatureID::MergeFrom(const FeatureID& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:RoadPB.FeatureID)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000001fu) {
    if (cached_has_bits & 0x00000001u) {
      tileid_ = from.tileid_;
    }
    if (cached_has_bits & 0x00000002u) {
      type_ = from.type_;
    }
    if (cached_has_bits & 0x00000004u) {
      id_ = from.id_;
    }
    if (cached_has_bits & 0x00000008u) {
      version_ = from.version_;
    }
    if (cached_has_bits & 0x00000010u) {
      is_deleted_ = from.is_deleted_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void FeatureID::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:RoadPB.FeatureID)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void FeatureID::CopyFrom(const FeatureID& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:RoadPB.FeatureID)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FeatureID::IsInitialized() const {
  return true;
}

void FeatureID::InternalSwap(FeatureID* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(FeatureID, is_deleted_)
      + sizeof(FeatureID::is_deleted_)
      - PROTOBUF_FIELD_OFFSET(FeatureID, tileid_)>(
          reinterpret_cast<char*>(&tileid_),
          reinterpret_cast<char*>(&other->tileid_));
}

::PROTOBUF_NAMESPACE_ID::Metadata FeatureID::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace RoadPB
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::RoadPB::FeatureID* Arena::CreateMaybeMessage< ::RoadPB::FeatureID >(Arena* arena) {
  return Arena::CreateMessageInternal< ::RoadPB::FeatureID >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
