﻿// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_PCDDEFINE_HADMAPFBS_H_
#define FLATBUFFERS_GENERATED_PCDDEFINE_HADMAPFBS_H_

#include "flatbuffers/flatbuffers.h"

namespace HadmapFBS {

struct PointXYZIRGB;

struct Vec3f;

struct Quaternionf;

struct PCLHeader;
struct PCLHeaderT;

struct PointCloud;
struct PointCloudT;

enum PointType {
  PointType_XYZIRGB = 1,
  PointType_XYZIRGB_Frame = 129,
  PointType_MIN = PointType_XYZIRGB,
  PointType_MAX = PointType_XYZIRGB_Frame
};

inline PointType (&EnumValuesPointType())[2] {
  static PointType values[] = {
    PointType_XYZIRGB,
    PointType_XYZIRGB_Frame
  };
  return values;
}

MANUALLY_ALIGNED_STRUCT(4) PointXYZIRGB FLATBUFFERS_FINAL_CLASS {
 private:
  float x_;
  float y_;
  float z_;
  uint8_t intensity_;
  uint8_t r_;
  uint8_t g_;
  uint8_t b_;

 public:
  PointXYZIRGB() {
    memset(this, 0, sizeof(PointXYZIRGB));
  }
  PointXYZIRGB(const PointXYZIRGB &_o) {
    memcpy(this, &_o, sizeof(PointXYZIRGB));
  }
  PointXYZIRGB(float _x, float _y, float _z, uint8_t _intensity, uint8_t _r, uint8_t _g, uint8_t _b)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)),
        z_(flatbuffers::EndianScalar(_z)),
        intensity_(flatbuffers::EndianScalar(_intensity)),
        r_(flatbuffers::EndianScalar(_r)),
        g_(flatbuffers::EndianScalar(_g)),
        b_(flatbuffers::EndianScalar(_b)) {
  }
  float x() const {
    return flatbuffers::EndianScalar(x_);
  }
  float y() const {
    return flatbuffers::EndianScalar(y_);
  }
  float z() const {
    return flatbuffers::EndianScalar(z_);
  }
  uint8_t intensity() const {
    return flatbuffers::EndianScalar(intensity_);
  }
  uint8_t r() const {
    return flatbuffers::EndianScalar(r_);
  }
  uint8_t g() const {
    return flatbuffers::EndianScalar(g_);
  }
  uint8_t b() const {
    return flatbuffers::EndianScalar(b_);
  }
};
STRUCT_END(PointXYZIRGB, 16);

MANUALLY_ALIGNED_STRUCT(4) Vec3f FLATBUFFERS_FINAL_CLASS {
 private:
  float x_;
  float y_;
  float z_;

 public:
  Vec3f() {
    memset(this, 0, sizeof(Vec3f));
  }
  Vec3f(const Vec3f &_o) {
    memcpy(this, &_o, sizeof(Vec3f));
  }
  Vec3f(float _x, float _y, float _z)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)),
        z_(flatbuffers::EndianScalar(_z)) {
  }
  float x() const {
    return flatbuffers::EndianScalar(x_);
  }
  float y() const {
    return flatbuffers::EndianScalar(y_);
  }
  float z() const {
    return flatbuffers::EndianScalar(z_);
  }
};
STRUCT_END(Vec3f, 12);

MANUALLY_ALIGNED_STRUCT(4) Quaternionf FLATBUFFERS_FINAL_CLASS {
 private:
  float x_;
  float y_;
  float z_;
  float w_;

 public:
  Quaternionf() {
    memset(this, 0, sizeof(Quaternionf));
  }
  Quaternionf(const Quaternionf &_o) {
    memcpy(this, &_o, sizeof(Quaternionf));
  }
  Quaternionf(float _x, float _y, float _z, float _w)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)),
        z_(flatbuffers::EndianScalar(_z)),
        w_(flatbuffers::EndianScalar(_w)) {
  }
  float x() const {
    return flatbuffers::EndianScalar(x_);
  }
  float y() const {
    return flatbuffers::EndianScalar(y_);
  }
  float z() const {
    return flatbuffers::EndianScalar(z_);
  }
  float w() const {
    return flatbuffers::EndianScalar(w_);
  }
};
STRUCT_END(Quaternionf, 16);

struct PCLHeaderT : public flatbuffers::NativeTable {
  typedef PCLHeader TableType;
  uint32_t seq;
  uint64_t stamp;
  std::string frame_id;
  PCLHeaderT()
      : seq(0),
        stamp(0) {
  }
};

struct PCLHeader FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef PCLHeaderT NativeTableType;
  enum {
    VT_SEQ = 4,
    VT_STAMP = 6,
    VT_FRAME_ID = 8
  };
  uint32_t seq() const {
    return GetField<uint32_t>(VT_SEQ, 0);
  }
  uint64_t stamp() const {
    return GetField<uint64_t>(VT_STAMP, 0);
  }
  const flatbuffers::String *frame_id() const {
    return GetPointer<const flatbuffers::String *>(VT_FRAME_ID);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<uint32_t>(verifier, VT_SEQ) &&
           VerifyField<uint64_t>(verifier, VT_STAMP) &&
           VerifyOffset(verifier, VT_FRAME_ID) &&
           verifier.Verify(frame_id()) &&
           verifier.EndTable();
  }
  PCLHeaderT *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(PCLHeaderT *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static flatbuffers::Offset<PCLHeader> Pack(flatbuffers::FlatBufferBuilder &_fbb, const PCLHeaderT* _o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct PCLHeaderBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_seq(uint32_t seq) {
    fbb_.AddElement<uint32_t>(PCLHeader::VT_SEQ, seq, 0);
  }
  void add_stamp(uint64_t stamp) {
    fbb_.AddElement<uint64_t>(PCLHeader::VT_STAMP, stamp, 0);
  }
  void add_frame_id(flatbuffers::Offset<flatbuffers::String> frame_id) {
    fbb_.AddOffset(PCLHeader::VT_FRAME_ID, frame_id);
  }
  PCLHeaderBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  PCLHeaderBuilder &operator=(const PCLHeaderBuilder &);
  flatbuffers::Offset<PCLHeader> Finish() {
    const auto end = fbb_.EndTable(start_, 3);
    auto o = flatbuffers::Offset<PCLHeader>(end);
    return o;
  }
};

inline flatbuffers::Offset<PCLHeader> CreatePCLHeader(
    flatbuffers::FlatBufferBuilder &_fbb,
    uint32_t seq = 0,
    uint64_t stamp = 0,
    flatbuffers::Offset<flatbuffers::String> frame_id = 0) {
  PCLHeaderBuilder builder_(_fbb);
  builder_.add_stamp(stamp);
  builder_.add_frame_id(frame_id);
  builder_.add_seq(seq);
  return builder_.Finish();
}

inline flatbuffers::Offset<PCLHeader> CreatePCLHeaderDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    uint32_t seq = 0,
    uint64_t stamp = 0,
    const char *frame_id = nullptr) {
  return HadmapFBS::CreatePCLHeader(
      _fbb,
      seq,
      stamp,
      frame_id ? _fbb.CreateString(frame_id) : 0);
}

flatbuffers::Offset<PCLHeader> CreatePCLHeader(flatbuffers::FlatBufferBuilder &_fbb, const PCLHeaderT *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct PointCloudT : public flatbuffers::NativeTable {
  typedef PointCloud TableType;
  std::unique_ptr<PCLHeaderT> header;
  uint32_t width;
  uint32_t height;
  bool is_dense;
  std::unique_ptr<Vec3f> sensor_origin;
  std::unique_ptr<Quaternionf> sensor_orientation;
  PointType point_type;
  std::vector<PointXYZIRGB> points_xyzirgb;
  std::vector<uint32_t> points_mask;
  PointCloudT()
      : width(0),
        height(0),
        is_dense(false),
        point_type(PointType_XYZIRGB_Frame) {
  }
};

struct PointCloud FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  typedef PointCloudT NativeTableType;
  enum {
    VT_HEADER = 4,
    VT_WIDTH = 6,
    VT_HEIGHT = 8,
    VT_IS_DENSE = 10,
    VT_SENSOR_ORIGIN = 12,
    VT_SENSOR_ORIENTATION = 14,
    VT_POINT_TYPE = 16,
    VT_POINTS_XYZIRGB = 18,
    VT_POINTS_MASK = 20
  };
  const PCLHeader *header() const {
    return GetPointer<const PCLHeader *>(VT_HEADER);
  }
  uint32_t width() const {
    return GetField<uint32_t>(VT_WIDTH, 0);
  }
  uint32_t height() const {
    return GetField<uint32_t>(VT_HEIGHT, 0);
  }
  bool is_dense() const {
    return GetField<uint8_t>(VT_IS_DENSE, 0) != 0;
  }
  const Vec3f *sensor_origin() const {
    return GetStruct<const Vec3f *>(VT_SENSOR_ORIGIN);
  }
  const Quaternionf *sensor_orientation() const {
    return GetStruct<const Quaternionf *>(VT_SENSOR_ORIENTATION);
  }
  PointType point_type() const {
    return static_cast<PointType>(GetField<uint8_t>(VT_POINT_TYPE, 129));
  }
  const flatbuffers::Vector<const PointXYZIRGB *> *points_xyzirgb() const {
    return GetPointer<const flatbuffers::Vector<const PointXYZIRGB *> *>(VT_POINTS_XYZIRGB);
  }
  const flatbuffers::Vector<uint32_t> *points_mask() const {
    return GetPointer<const flatbuffers::Vector<uint32_t> *>(VT_POINTS_MASK);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_HEADER) &&
           verifier.VerifyTable(header()) &&
           VerifyField<uint32_t>(verifier, VT_WIDTH) &&
           VerifyField<uint32_t>(verifier, VT_HEIGHT) &&
           VerifyField<uint8_t>(verifier, VT_IS_DENSE) &&
           VerifyField<Vec3f>(verifier, VT_SENSOR_ORIGIN) &&
           VerifyField<Quaternionf>(verifier, VT_SENSOR_ORIENTATION) &&
           VerifyField<uint8_t>(verifier, VT_POINT_TYPE) &&
           VerifyOffset(verifier, VT_POINTS_XYZIRGB) &&
           verifier.Verify(points_xyzirgb()) &&
           VerifyOffset(verifier, VT_POINTS_MASK) &&
           verifier.Verify(points_mask()) &&
           verifier.EndTable();
  }
  PointCloudT *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  void UnPackTo(PointCloudT *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
  static flatbuffers::Offset<PointCloud> Pack(flatbuffers::FlatBufferBuilder &_fbb, const PointCloudT* _o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct PointCloudBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_header(flatbuffers::Offset<PCLHeader> header) {
    fbb_.AddOffset(PointCloud::VT_HEADER, header);
  }
  void add_width(uint32_t width) {
    fbb_.AddElement<uint32_t>(PointCloud::VT_WIDTH, width, 0);
  }
  void add_height(uint32_t height) {
    fbb_.AddElement<uint32_t>(PointCloud::VT_HEIGHT, height, 0);
  }
  void add_is_dense(bool is_dense) {
    fbb_.AddElement<uint8_t>(PointCloud::VT_IS_DENSE, static_cast<uint8_t>(is_dense), 0);
  }
  void add_sensor_origin(const Vec3f *sensor_origin) {
    fbb_.AddStruct(PointCloud::VT_SENSOR_ORIGIN, sensor_origin);
  }
  void add_sensor_orientation(const Quaternionf *sensor_orientation) {
    fbb_.AddStruct(PointCloud::VT_SENSOR_ORIENTATION, sensor_orientation);
  }
  void add_point_type(PointType point_type) {
    fbb_.AddElement<uint8_t>(PointCloud::VT_POINT_TYPE, static_cast<uint8_t>(point_type), 129);
  }
  void add_points_xyzirgb(flatbuffers::Offset<flatbuffers::Vector<const PointXYZIRGB *>> points_xyzirgb) {
    fbb_.AddOffset(PointCloud::VT_POINTS_XYZIRGB, points_xyzirgb);
  }
  void add_points_mask(flatbuffers::Offset<flatbuffers::Vector<uint32_t>> points_mask) {
    fbb_.AddOffset(PointCloud::VT_POINTS_MASK, points_mask);
  }
  PointCloudBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  PointCloudBuilder &operator=(const PointCloudBuilder &);
  flatbuffers::Offset<PointCloud> Finish() {
    const auto end = fbb_.EndTable(start_, 9);
    auto o = flatbuffers::Offset<PointCloud>(end);
    return o;
  }
};

inline flatbuffers::Offset<PointCloud> CreatePointCloud(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<PCLHeader> header = 0,
    uint32_t width = 0,
    uint32_t height = 0,
    bool is_dense = false,
    const Vec3f *sensor_origin = 0,
    const Quaternionf *sensor_orientation = 0,
    PointType point_type = PointType_XYZIRGB_Frame,
    flatbuffers::Offset<flatbuffers::Vector<const PointXYZIRGB *>> points_xyzirgb = 0,
    flatbuffers::Offset<flatbuffers::Vector<uint32_t>> points_mask = 0) {
  PointCloudBuilder builder_(_fbb);
  builder_.add_points_mask(points_mask);
  builder_.add_points_xyzirgb(points_xyzirgb);
  builder_.add_sensor_orientation(sensor_orientation);
  builder_.add_sensor_origin(sensor_origin);
  builder_.add_height(height);
  builder_.add_width(width);
  builder_.add_header(header);
  builder_.add_point_type(point_type);
  builder_.add_is_dense(is_dense);
  return builder_.Finish();
}

inline flatbuffers::Offset<PointCloud> CreatePointCloudDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<PCLHeader> header = 0,
    uint32_t width = 0,
    uint32_t height = 0,
    bool is_dense = false,
    const Vec3f *sensor_origin = 0,
    const Quaternionf *sensor_orientation = 0,
    PointType point_type = PointType_XYZIRGB_Frame,
    const std::vector<const PointXYZIRGB *> *points_xyzirgb = nullptr,
    const std::vector<uint32_t> *points_mask = nullptr) {
  return HadmapFBS::CreatePointCloud(
      _fbb,
      header,
      width,
      height,
      is_dense,
      sensor_origin,
      sensor_orientation,
      point_type,
      points_xyzirgb ? _fbb.CreateVector<const PointXYZIRGB *>(*points_xyzirgb) : 0,
      points_mask ? _fbb.CreateVector<uint32_t>(*points_mask) : 0);
}

flatbuffers::Offset<PointCloud> CreatePointCloud(flatbuffers::FlatBufferBuilder &_fbb, const PointCloudT *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

inline PCLHeaderT *PCLHeader::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
  auto _o = new PCLHeaderT();
  UnPackTo(_o, _resolver);
  return _o;
}

inline void PCLHeader::UnPackTo(PCLHeaderT *_o, const flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = seq(); _o->seq = _e; };
  { auto _e = stamp(); _o->stamp = _e; };
  { auto _e = frame_id(); if (_e) _o->frame_id = _e->str(); };
}

inline flatbuffers::Offset<PCLHeader> PCLHeader::Pack(flatbuffers::FlatBufferBuilder &_fbb, const PCLHeaderT* _o, const flatbuffers::rehasher_function_t *_rehasher) {
  return CreatePCLHeader(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<PCLHeader> CreatePCLHeader(flatbuffers::FlatBufferBuilder &_fbb, const PCLHeaderT *_o, const flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  auto _seq = _o->seq;
  auto _stamp = _o->stamp;
  auto _frame_id = _o->frame_id.size() ? _fbb.CreateString(_o->frame_id) : 0;
  return HadmapFBS::CreatePCLHeader(
      _fbb,
      _seq,
      _stamp,
      _frame_id);
}

inline PointCloudT *PointCloud::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
  auto _o = new PointCloudT();
  UnPackTo(_o, _resolver);
  return _o;
}

inline void PointCloud::UnPackTo(PointCloudT *_o, const flatbuffers::resolver_function_t *_resolver) const {
  (void)_o;
  (void)_resolver;
  { auto _e = header(); if (_e) _o->header = std::unique_ptr<PCLHeaderT>(_e->UnPack(_resolver)); };
  { auto _e = width(); _o->width = _e; };
  { auto _e = height(); _o->height = _e; };
  { auto _e = is_dense(); _o->is_dense = _e; };
  { auto _e = sensor_origin(); if (_e) _o->sensor_origin = std::unique_ptr<Vec3f>(new Vec3f(*_e)); };
  { auto _e = sensor_orientation(); if (_e) _o->sensor_orientation = std::unique_ptr<Quaternionf>(new Quaternionf(*_e)); };
  { auto _e = point_type(); _o->point_type = _e; };
  { auto _e = points_xyzirgb(); if (_e) { _o->points_xyzirgb.resize(_e->size()); for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) { _o->points_xyzirgb[_i] = *_e->Get(_i); } } };
  { auto _e = points_mask(); if (_e) { _o->points_mask.resize(_e->size()); for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) { _o->points_mask[_i] = _e->Get(_i); } } };
}

inline flatbuffers::Offset<PointCloud> PointCloud::Pack(flatbuffers::FlatBufferBuilder &_fbb, const PointCloudT* _o, const flatbuffers::rehasher_function_t *_rehasher) {
  return CreatePointCloud(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<PointCloud> CreatePointCloud(flatbuffers::FlatBufferBuilder &_fbb, const PointCloudT *_o, const flatbuffers::rehasher_function_t *_rehasher) {
  (void)_rehasher;
  (void)_o;
  auto _header = _o->header ? CreatePCLHeader(_fbb, _o->header.get(), _rehasher) : 0;
  auto _width = _o->width;
  auto _height = _o->height;
  auto _is_dense = _o->is_dense;
  auto _sensor_origin = _o->sensor_origin ? _o->sensor_origin.get() : 0;
  auto _sensor_orientation = _o->sensor_orientation ? _o->sensor_orientation.get() : 0;
  auto _point_type = _o->point_type;
  auto _points_xyzirgb = _o->points_xyzirgb.size() ? _fbb.CreateVectorOfStructs(_o->points_xyzirgb) : 0;
  auto _points_mask = _o->points_mask.size() ? _fbb.CreateVector(_o->points_mask) : 0;
  return HadmapFBS::CreatePointCloud(
      _fbb,
      _header,
      _width,
      _height,
      _is_dense,
      _sensor_origin,
      _sensor_orientation,
      _point_type,
      _points_xyzirgb,
      _points_mask);
}

inline const HadmapFBS::PointCloud *GetPointCloud(const void *buf) {
  return flatbuffers::GetRoot<HadmapFBS::PointCloud>(buf);
}

inline bool VerifyPointCloudBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<HadmapFBS::PointCloud>(nullptr);
}

inline void FinishPointCloudBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<HadmapFBS::PointCloud> root) {
  fbb.Finish(root);
}

inline std::unique_ptr<PointCloudT> UnPackPointCloud(
    const void *buf,
    const flatbuffers::resolver_function_t *res = nullptr) {
  return std::unique_ptr<PointCloudT>(GetPointCloud(buf)->UnPack(res));
}

}  // namespace HadmapFBS

#endif  // FLATBUFFERS_GENERATED_PCDDEFINE_HADMAPFBS_H_
