// Protocol Buffers - Google's data interchange format
// Copyright 2008 Google Inc.  All rights reserved.
// http://code.google.com/p/protobuf/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: kenton@google.com (Kenton Varda)
//  Based on original Protocol Buffers design by
//  Sanjay Ghemawat, Jeff Dean, and others.

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/stubs/stl_util-inl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/wire_format.h>

namespace google {
namespace protobuf {

UnknownFieldSet::UnknownFieldSet()
  : fields_(NULL) {}

UnknownFieldSet::~UnknownFieldSet() {
  Clear();
  delete fields_;
}

void UnknownFieldSet::ClearFallback() {
  GOOGLE_DCHECK(fields_ != nullptr);
  for (int i = 0; i < fields_->size(); i++) {
    (*fields_)[i].Delete();
  }
  fields_->clear();
}

void UnknownFieldSet::MergeFrom(const UnknownFieldSet& other) {
  for (int i = 0; i < other.field_count(); i++) {
    AddField(other.field(i));
  }
}

int UnknownFieldSet::SpaceUsedExcludingSelf() const {
  if (fields_ == nullptr) return 0;

  int total_size = sizeof(*fields_) + sizeof(UnknownField) * fields_->size();
  for (int i = 0; i < fields_->size(); i++) {
    const UnknownField& field = (*fields_)[i];
    switch (field.type()) {
      case UnknownField::TYPE_LENGTH_DELIMITED:
        total_size += sizeof(*field.length_delimited_) +
          internal::StringSpaceUsedExcludingSelf(*field.length_delimited_);
        break;
      case UnknownField::TYPE_GROUP:
        total_size += field.group_->SpaceUsed();
        break;
      default:
        break;
    }
  }
  return total_size;
}

int UnknownFieldSet::SpaceUsed() const {
  return sizeof(*this) + SpaceUsedExcludingSelf();
}

void UnknownFieldSet::AddVarint(int number, uint64 value) {
  if (fields_ == nullptr) fields_ = new vector<UnknownField>;
  UnknownField field;
  field.number_ = number;
  field.type_ = UnknownField::TYPE_VARINT;
  field.varint_ = value;
  fields_->push_back(field);
}

void UnknownFieldSet::AddFixed32(int number, uint32 value) {
  if (fields_ == nullptr) fields_ = new vector<UnknownField>;
  UnknownField field;
  field.number_ = number;
  field.type_ = UnknownField::TYPE_FIXED32;
  field.fixed32_ = value;
  fields_->push_back(field);
}

void UnknownFieldSet::AddFixed64(int number, uint64 value) {
  if (fields_ == nullptr) fields_ = new vector<UnknownField>;
  UnknownField field;
  field.number_ = number;
  field.type_ = UnknownField::TYPE_FIXED64;
  field.fixed64_ = value;
  fields_->push_back(field);
}

string* UnknownFieldSet::AddLengthDelimited(int number) {
  if (fields_ == nullptr) fields_ = new vector<UnknownField>;
  UnknownField field;
  field.number_ = number;
  field.type_ = UnknownField::TYPE_LENGTH_DELIMITED;
  field.length_delimited_ = new string;
  fields_->push_back(field);
  return field.length_delimited_;
}

UnknownFieldSet* UnknownFieldSet::AddGroup(int number) {
  if (fields_ == nullptr) fields_ = new vector<UnknownField>;
  UnknownField field;
  field.number_ = number;
  field.type_ = UnknownField::TYPE_GROUP;
  field.group_ = new UnknownFieldSet;
  fields_->push_back(field);
  return field.group_;
}

void UnknownFieldSet::AddField(const UnknownField& field) {
  if (fields_ == nullptr) fields_ = new vector<UnknownField>;
  fields_->push_back(field);
  fields_->back().DeepCopy();
}

bool UnknownFieldSet::MergeFromCodedStream(io::CodedInputStream* input) {

  UnknownFieldSet other;
  if (internal::WireFormat::SkipMessage(input, &other) &&
                                  input->ConsumedEntireMessage()) {
    MergeFrom(other);
    return true;
  } else {
    return false;
  }
}

bool UnknownFieldSet::ParseFromCodedStream(io::CodedInputStream* input) {
  Clear();
  return MergeFromCodedStream(input);
}

bool UnknownFieldSet::ParseFromZeroCopyStream(io::ZeroCopyInputStream* input) {
  io::CodedInputStream coded_input(input);
  return ParseFromCodedStream(&coded_input) &&
    coded_input.ConsumedEntireMessage();
}

bool UnknownFieldSet::ParseFromArray(const void* data, int size) {
  io::ArrayInputStream input(data, size);
  return ParseFromZeroCopyStream(&input);
}

void UnknownField::Delete() {
  switch (type()) {
    case UnknownField::TYPE_LENGTH_DELIMITED:
      delete length_delimited_;
      break;
    case UnknownField::TYPE_GROUP:
      delete group_;
      break;
    default:
      break;
  }
}

void UnknownField::DeepCopy() {
  switch (type()) {
    case UnknownField::TYPE_LENGTH_DELIMITED:
      length_delimited_ = new string(*length_delimited_);
      break;
    case UnknownField::TYPE_GROUP: {
      UnknownFieldSet* group = new UnknownFieldSet;
      group->MergeFrom(*group_);
      group_ = group;
      break;
    }
    default:
      break;
  }
}

}  // namespace protobuf
}  // namespace google
