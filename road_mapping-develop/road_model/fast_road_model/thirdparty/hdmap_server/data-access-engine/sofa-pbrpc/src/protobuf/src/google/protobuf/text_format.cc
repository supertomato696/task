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

// Author: jschorr@google.com (Joseph Schorr)
//  Based on original Protocol Buffers design by
//  Sanjay Ghemawat, Jeff Dean, and others.

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stack>
#include <limits>
#include <vector>

#include <google/protobuf/text_format.h>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/unknown_field_set.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/io/tokenizer.h>
#include <google/protobuf/stubs/strutil.h>

namespace google {
namespace protobuf {

string Message::DebugString() const {
  string debug_string;

  TextFormat::PrintToString(*this, &debug_string);

  return debug_string;
}

string Message::ShortDebugString() const {
  string debug_string;

  TextFormat::Printer printer;
  printer.SetSingleLineMode(true);

  printer.PrintToString(*this, &debug_string);
  // Single line mode currently might have an extra space at the end.
  if (debug_string.size() > 0 &&
      debug_string[debug_string.size() - 1] == ' ') {
    debug_string.resize(debug_string.size() - 1);
  }

  return debug_string;
}

string Message::Utf8DebugString() const {
  string debug_string;

  TextFormat::Printer printer;
  printer.SetUseUtf8StringEscaping(true);

  printer.PrintToString(*this, &debug_string);

  return debug_string;
}

void Message::PrintDebugString() const {
  printf("%s", DebugString().c_str());
}


// ===========================================================================
// Internal class for parsing an ASCII representation of a Protocol Message.
// This class makes use of the Protocol Message compiler's tokenizer found
// in //google/protobuf/io/tokenizer.h. Note that class's Parse
// method is *not* thread-safe and should only be used in a single thread at
// a time.

// Makes code slightly more readable.  The meaning of "DO(foo)" is
// "Execute foo and fail if it fails.", where failure is indicated by
// returning false. Borrowed from parser.cc (Thanks Kenton!).
#define DO(STATEMENT) if (STATEMENT) {} else return false

class TextFormat::Parser::ParserImpl {
 public:

  // Determines if repeated values for a non-repeated field are
  // permitted, e.g., the string "foo: 1 foo: 2" for a
  // required/optional field named "foo".
  enum SingularOverwritePolicy {
    ALLOW_SINGULAR_OVERWRITES = 0,   // the last value is retained
    FORBID_SINGULAR_OVERWRITES = 1,  // an error is issued
  };

  ParserImpl(const Descriptor* root_message_type,
             io::ZeroCopyInputStream* input_stream,
             io::ErrorCollector* error_collector,
             TextFormat::Finder* finder,
             SingularOverwritePolicy singular_overwrite_policy)
    : error_collector_(error_collector),
      finder_(finder),
      tokenizer_error_collector_(this),
      tokenizer_(input_stream, &tokenizer_error_collector_),
      root_message_type_(root_message_type),
      singular_overwrite_policy_(singular_overwrite_policy),
      had_errors_(false) {
    // For backwards-compatibility with proto1, we need to allow the 'f' suffix
    // for floats.
    tokenizer_.set_allow_f_after_float(true);

    // '#' starts a comment.
    tokenizer_.set_comment_style(io::Tokenizer::SH_COMMENT_STYLE);

    // Consume the starting token.
    tokenizer_.Next();
  }
  ~ParserImpl() { }

  // Parses the ASCII representation specified in input and saves the
  // information into the output pointer (a Message). Returns
  // false if an error occurs (an error will also be logged to
  // GOOGLE_LOG(ERROR)).
  bool Parse(Message* output) {
    // Consume fields until we cannot do so anymore.
    while(true) {
      if (LookingAtType(io::Tokenizer::TYPE_END)) {
        return !had_errors_;
      }

      DO(ConsumeField(output));
    }
  }

  bool ParseField(const FieldDescriptor* field, Message* output) {
    bool suc;
    if (field->cpp_type() == FieldDescriptor::CPPTYPE_MESSAGE) {
      suc = ConsumeFieldMessage(output, output->GetReflection(), field);
    } else {
      suc = ConsumeFieldValue(output, output->GetReflection(), field);
    }
    return suc && LookingAtType(io::Tokenizer::TYPE_END);
  }

  void ReportError(int line, int col, const string& message) {
    had_errors_ = true;
    if (error_collector_ == nullptr) {
      if (line >= 0) {
        GOOGLE_LOG(ERROR) << "Error parsing text-format "
                   << root_message_type_->full_name()
                   << ": " << (line + 1) << ":"
                   << (col + 1) << ": " << message;
      } else {
        GOOGLE_LOG(ERROR) << "Error parsing text-format "
                   << root_message_type_->full_name()
                   << ": " << message;
      }
    } else {
      error_collector_->AddError(line, col, message);
    }
  }

  void ReportWarning(int line, int col, const string& message) {
    if (error_collector_ == nullptr) {
      if (line >= 0) {
        GOOGLE_LOG(WARNING) << "Warning parsing text-format "
                     << root_message_type_->full_name()
                     << ": " << (line + 1) << ":"
                     << (col + 1) << ": " << message;
      } else {
        GOOGLE_LOG(WARNING) << "Warning parsing text-format "
                     << root_message_type_->full_name()
                     << ": " << message;
      }
    } else {
      error_collector_->AddWarning(line, col, message);
    }
  }

 private:
  GOOGLE_DISALLOW_EVIL_CONSTRUCTORS(ParserImpl);

  // Reports an error with the given message with information indicating
  // the position (as derived from the current token).
  void ReportError(const string& message) {
    ReportError(tokenizer_.current().line, tokenizer_.current().column,
                message);
  }

  // Reports a warning with the given message with information indicating
  // the position (as derived from the current token).
  void ReportWarning(const string& message) {
    ReportWarning(tokenizer_.current().line, tokenizer_.current().column,
                  message);
  }

  // Consumes the specified message with the given starting delimeter.
  // This method checks to see that the end delimeter at the conclusion of
  // the consumption matches the starting delimeter passed in here.
  bool ConsumeMessage(Message* message, const string delimeter) {
    while (!LookingAt(">") &&  !LookingAt("}")) {
      DO(ConsumeField(message));
    }

    // Confirm that we have a valid ending delimeter.
    DO(Consume(delimeter));

    return true;
  }

  // Consumes the current field (as returned by the tokenizer) on the
  // passed in message.
  bool ConsumeField(Message* message) {
    const Reflection* reflection = message->GetReflection();
    const Descriptor* descriptor = message->GetDescriptor();

    string field_name;

    const FieldDescriptor* field = nullptr;

    if (TryConsume("[")) {
      // Extension.
      DO(ConsumeIdentifier(&field_name));
      while (TryConsume(".")) {
        string part;
        DO(ConsumeIdentifier(&part));
        field_name += ".";
        field_name += part;
      }
      DO(Consume("]"));

      field = (finder_ != nullptr
               ? finder_->FindExtension(message, field_name)
               : reflection->FindKnownExtensionByName(field_name));

      if (field == nullptr) {
        ReportError("Extension \"" + field_name + "\" is not defined or "
                    "is not an extension of \"" +
                    descriptor->full_name() + "\".");
        return false;
      }
    } else {
      DO(ConsumeIdentifier(&field_name));

      field = descriptor->FindFieldByName(field_name);
      // Group names are expected to be capitalized as they appear in the
      // .proto file, which actually matches their type names, not their field
      // names.
      if (field == nullptr) {
        string lower_field_name = field_name;
        LowerString(&lower_field_name);
        field = descriptor->FindFieldByName(lower_field_name);
        // If the case-insensitive match worked but the field is NOT a group,
        if (field != nullptr && field->type() != FieldDescriptor::TYPE_GROUP) {
          field = nullptr;
        }
      }
      // Again, special-case group names as described above.
      if (field != nullptr && field->type() == FieldDescriptor::TYPE_GROUP
          && field->message_type()->name() != field_name) {
        field = nullptr;
      }

      if (field == nullptr) {
        ReportError("Message type \"" + descriptor->full_name() +
                    "\" has no field named \"" + field_name + "\".");
        return false;
      }
    }

    // Fail if the field is not repeated and it has already been specified.
    if ((singular_overwrite_policy_ == FORBID_SINGULAR_OVERWRITES) &&
        !field->is_repeated() && reflection->HasField(*message, field)) {
      ReportError("Non-repeated field \"" + field_name +
                  "\" is specified multiple times.");
      return false;
    }

    // Perform special handling for embedded message types.
    if (field->cpp_type() == FieldDescriptor::CPPTYPE_MESSAGE) {
      // ':' is optional here.
      TryConsume(":");
      DO(ConsumeFieldMessage(message, reflection, field));
    } else {
      DO(Consume(":"));
      if (field->is_repeated() && TryConsume("[")) {
        // Short repeated format, e.g.  "foo: [1, 2, 3]"
        while (true) {
          DO(ConsumeFieldValue(message, reflection, field));
          if (TryConsume("]")) {
            break;
          }
          DO(Consume(","));
        }
      } else {
        DO(ConsumeFieldValue(message, reflection, field));
      }
    }

    // For historical reasons, fields may optionally be separated by commas or
    // semicolons.
    TryConsume(";") || TryConsume(",");

    if (field->options().deprecated()) {
      ReportWarning("text format contains deprecated field \""
                    + field_name + "\"");
    }

    return true;
  }

  bool ConsumeFieldMessage(Message* message,
                           const Reflection* reflection,
                           const FieldDescriptor* field) {
    string delimeter;
    if (TryConsume("<")) {
      delimeter = ">";
    } else {
      DO(Consume("{"));
      delimeter = "}";
    }

    if (field->is_repeated()) {
      DO(ConsumeMessage(reflection->AddMessage(message, field), delimeter));
    } else {
      DO(ConsumeMessage(reflection->MutableMessage(message, field),
                        delimeter));
    }
    return true;
  }

  bool ConsumeFieldValue(Message* message,
                         const Reflection* reflection,
                         const FieldDescriptor* field) {

// Define an easy to use macro for setting fields. This macro checks
// to see if the field is repeated (in which case we need to use the Add
// methods or not (in which case we need to use the Set methods).
#define SET_FIELD(CPPTYPE, VALUE)                                  \
        if (field->is_repeated()) {                                \
          reflection->Add##CPPTYPE(message, field, VALUE);         \
        } else {                                                   \
          reflection->Set##CPPTYPE(message, field, VALUE);         \
        }                                                          \

    switch(field->cpp_type()) {
      case FieldDescriptor::CPPTYPE_INT32: {
        int64 value;
        DO(ConsumeSignedInteger(&value, kint32max));
        SET_FIELD(Int32, static_cast<int32>(value));
        break;
      }

      case FieldDescriptor::CPPTYPE_UINT32: {
        uint64 value;
        DO(ConsumeUnsignedInteger(&value, kuint32max));
        SET_FIELD(UInt32, static_cast<uint32>(value));
        break;
      }

      case FieldDescriptor::CPPTYPE_INT64: {
        int64 value;
        DO(ConsumeSignedInteger(&value, kint64max));
        SET_FIELD(Int64, value);
        break;
      }

      case FieldDescriptor::CPPTYPE_UINT64: {
        uint64 value;
        DO(ConsumeUnsignedInteger(&value, kuint64max));
        SET_FIELD(UInt64, value);
        break;
      }

      case FieldDescriptor::CPPTYPE_FLOAT: {
        double value;
        DO(ConsumeDouble(&value));
        SET_FIELD(Float, static_cast<float>(value));
        break;
      }

      case FieldDescriptor::CPPTYPE_DOUBLE: {
        double value;
        DO(ConsumeDouble(&value));
        SET_FIELD(Double, value);
        break;
      }

      case FieldDescriptor::CPPTYPE_STRING: {
        string value;
        DO(ConsumeString(&value));
        SET_FIELD(String, value);
        break;
      }

      case FieldDescriptor::CPPTYPE_BOOL: {
        if (LookingAtType(io::Tokenizer::TYPE_INTEGER)) {
          uint64 value;
          DO(ConsumeUnsignedInteger(&value, 1));
          SET_FIELD(Bool, value);
        } else {
          string value;
          DO(ConsumeIdentifier(&value));
          if (value == "true" || value == "t") {
            SET_FIELD(Bool, true);
          } else if (value == "false" || value == "f") {
            SET_FIELD(Bool, false);
          } else {
            ReportError("Invalid value for boolean field \"" + field->name()
                        + "\". Value: \"" + value  + "\".");
            return false;
          }
        }
        break;
      }

      case FieldDescriptor::CPPTYPE_ENUM: {
        string value;
        const EnumDescriptor* enum_type = field->enum_type();
        const EnumValueDescriptor* enum_value = nullptr;

        if (LookingAtType(io::Tokenizer::TYPE_IDENTIFIER)) {
          DO(ConsumeIdentifier(&value));
          // Find the enumeration value.
          enum_value = enum_type->FindValueByName(value);

        } else if (LookingAt("-") ||
                   LookingAtType(io::Tokenizer::TYPE_INTEGER)) {
          int64 int_value;
          DO(ConsumeSignedInteger(&int_value, kint32max));
          value = SimpleItoa(int_value);        // for error reporting
          enum_value = enum_type->FindValueByNumber(int_value);
        } else {
          ReportError("Expected integer or identifier.");
          return false;
        }

        if (enum_value == nullptr) {
          ReportError("Unknown enumeration value of \"" + value  + "\" for "
                      "field \"" + field->name() + "\".");
          return false;
        }

        SET_FIELD(Enum, enum_value);
        break;
      }

      case FieldDescriptor::CPPTYPE_MESSAGE: {
        // We should never get here. Put here instead of a default
        // so that if new types are added, we get a nice compiler warning.
        GOOGLE_LOG(FATAL) << "Reached an unintended state: CPPTYPE_MESSAGE";
        break;
      }
    }
#undef SET_FIELD
    return true;
  }

  // Returns true if the current token's text is equal to that specified.
  bool LookingAt(const string& text) {
    return tokenizer_.current().text == text;
  }

  // Returns true if the current token's type is equal to that specified.
  bool LookingAtType(io::Tokenizer::TokenType token_type) {
    return tokenizer_.current().type == token_type;
  }

  // Consumes an identifier and saves its value in the identifier parameter.
  // Returns false if the token is not of type IDENTFIER.
  bool ConsumeIdentifier(string* identifier) {
    if (!LookingAtType(io::Tokenizer::TYPE_IDENTIFIER)) {
      ReportError("Expected identifier.");
      return false;
    }

    *identifier = tokenizer_.current().text;

    tokenizer_.Next();
    return true;
  }

  // Consumes a string and saves its value in the text parameter.
  // Returns false if the token is not of type STRING.
  bool ConsumeString(string* text) {
    if (!LookingAtType(io::Tokenizer::TYPE_STRING)) {
      ReportError("Expected string.");
      return false;
    }

    text->clear();
    while (LookingAtType(io::Tokenizer::TYPE_STRING)) {
      io::Tokenizer::ParseStringAppend(tokenizer_.current().text, text);

      tokenizer_.Next();
    }

    return true;
  }

  // Consumes a uint64 and saves its value in the value parameter.
  // Returns false if the token is not of type INTEGER.
  bool ConsumeUnsignedInteger(uint64* value, uint64 max_value) {
    if (!LookingAtType(io::Tokenizer::TYPE_INTEGER)) {
      ReportError("Expected integer.");
      return false;
    }

    if (!io::Tokenizer::ParseInteger(tokenizer_.current().text,
                                     max_value, value)) {
      ReportError("Integer out of range.");
      return false;
    }

    tokenizer_.Next();
    return true;
  }

  // Consumes an int64 and saves its value in the value parameter.
  // Note that since the tokenizer does not support negative numbers,
  // we actually may consume an additional token (for the minus sign) in this
  // method. Returns false if the token is not an integer
  // (signed or otherwise).
  bool ConsumeSignedInteger(int64* value, uint64 max_value) {
    bool negative = false;

    if (TryConsume("-")) {
      negative = true;
      // Two's complement always allows one more negative integer than
      // positive.
      ++max_value;
    }

    uint64 unsigned_value;

    DO(ConsumeUnsignedInteger(&unsigned_value, max_value));

    *value = static_cast<int64>(unsigned_value);

    if (negative) {
      *value = -*value;
    }

    return true;
  }

  // Consumes a double and saves its value in the value parameter.
  // Note that since the tokenizer does not support negative numbers,
  // we actually may consume an additional token (for the minus sign) in this
  // method. Returns false if the token is not a double
  // (signed or otherwise).
  bool ConsumeDouble(double* value) {
    bool negative = false;

    if (TryConsume("-")) {
      negative = true;
    }

    // A double can actually be an integer, according to the tokenizer.
    // Therefore, we must check both cases here.
    if (LookingAtType(io::Tokenizer::TYPE_INTEGER)) {
      // We have found an integer value for the double.
      uint64 integer_value;
      DO(ConsumeUnsignedInteger(&integer_value, kuint64max));

      *value = static_cast<double>(integer_value);
    } else if (LookingAtType(io::Tokenizer::TYPE_FLOAT)) {
      // We have found a float value for the double.
      *value = io::Tokenizer::ParseFloat(tokenizer_.current().text);

      // Mark the current token as consumed.
      tokenizer_.Next();
    } else if (LookingAtType(io::Tokenizer::TYPE_IDENTIFIER)) {
      string text = tokenizer_.current().text;
      LowerString(&text);
      if (text == "inf" || text == "infinity") {
        *value = std::numeric_limits<double>::infinity();
        tokenizer_.Next();
      } else if (text == "nan") {
        *value = std::numeric_limits<double>::quiet_NaN();
        tokenizer_.Next();
      } else {
        ReportError("Expected double.");
        return false;
      }
    } else {
      ReportError("Expected double.");
      return false;
    }

    if (negative) {
      *value = -*value;
    }

    return true;
  }

  // Consumes a token and confirms that it matches that specified in the
  // value parameter. Returns false if the token found does not match that
  // which was specified.
  bool Consume(const string& value) {
    const string& current_value = tokenizer_.current().text;

    if (current_value != value) {
      ReportError("Expected \"" + value + "\", found \"" + current_value
                  + "\".");
      return false;
    }

    tokenizer_.Next();

    return true;
  }

  // Attempts to consume the supplied value. Returns false if a the
  // token found does not match the value specified.
  bool TryConsume(const string& value) {
    if (tokenizer_.current().text == value) {
      tokenizer_.Next();
      return true;
    } else {
      return false;
    }
  }

  // An internal instance of the Tokenizer's error collector, used to
  // collect any base-level parse errors and feed them to the ParserImpl.
  class ParserErrorCollector : public io::ErrorCollector {
   public:
    explicit ParserErrorCollector(TextFormat::Parser::ParserImpl* parser) :
        parser_(parser) { }

    virtual ~ParserErrorCollector() { };

    virtual void AddError(int line, int column, const string& message) {
      parser_->ReportError(line, column, message);
    }

    virtual void AddWarning(int line, int column, const string& message) {
      parser_->ReportWarning(line, column, message);
    }

   private:
    GOOGLE_DISALLOW_EVIL_CONSTRUCTORS(ParserErrorCollector);
    TextFormat::Parser::ParserImpl* parser_;
  };

  io::ErrorCollector* error_collector_;
  TextFormat::Finder* finder_;
  ParserErrorCollector tokenizer_error_collector_;
  io::Tokenizer tokenizer_;
  const Descriptor* root_message_type_;
  SingularOverwritePolicy singular_overwrite_policy_;
  bool had_errors_;
};

#undef DO

// ===========================================================================
// Internal class for writing text to the io::ZeroCopyOutputStream. Adapted
// from the Printer found in //google/protobuf/io/printer.h
class TextFormat::Printer::TextGenerator {
 public:
  explicit TextGenerator(io::ZeroCopyOutputStream* output,
                         int initial_indent_level)
    : output_(output),
      buffer_(NULL),
      buffer_size_(0),
      at_start_of_line_(true),
      failed_(false),
      indent_(""),
      initial_indent_level_(initial_indent_level) {
    indent_.resize(initial_indent_level_ * 2, ' ');
  }

  ~TextGenerator() {
    // Only BackUp() if we're sure we've successfully called Next() at least
    // once.
    if (buffer_size_ > 0) {
      output_->BackUp(buffer_size_);
    }
  }

  // Indent text by two spaces.  After calling Indent(), two spaces will be
  // inserted at the beginning of each line of text.  Indent() may be called
  // multiple times to produce deeper indents.
  void Indent() {
    indent_ += "  ";
  }

  // Reduces the current indent level by two spaces, or crashes if the indent
  // level is zero.
  void Outdent() {
    if (indent_.empty() ||
        indent_.size() < initial_indent_level_ * 2) {
      GOOGLE_LOG(DFATAL) << " Outdent() without matching Indent().";
      return;
    }

    indent_.resize(indent_.size() - 2);
  }

  // Print text to the output stream.
  void Print(const string& str) {
    Print(str.data(), str.size());
  }

  // Print text to the output stream.
  void Print(const char* text) {
    Print(text, strlen(text));
  }

  // Print text to the output stream.
  void Print(const char* text, int size) {
    int pos = 0;  // The number of bytes we've written so far.

    for (int i = 0; i < size; i++) {
      if (text[i] == '\n') {
        // Saw newline.  If there is more text, we may need to insert an indent
        // here.  So, write what we have so far, including the '\n'.
        Write(text + pos, i - pos + 1);
        pos = i + 1;

        // Setting this true will cause the next Write() to insert an indent
        // first.
        at_start_of_line_ = true;
      }
    }

    // Write the rest.
    Write(text + pos, size - pos);
  }

  // True if any write to the underlying stream failed.  (We don't just
  // crash in this case because this is an I/O failure, not a programming
  // error.)
  bool failed() const { return failed_; }

 private:
  GOOGLE_DISALLOW_EVIL_CONSTRUCTORS(TextGenerator);

  void Write(const char* data, int size) {
    if (failed_) return;
    if (size == 0) return;

    if (at_start_of_line_) {
      // Insert an indent.
      at_start_of_line_ = false;
      Write(indent_.data(), indent_.size());
      if (failed_) return;
    }

    while (size > buffer_size_) {
      // Data exceeds space in the buffer.  Copy what we can and request a
      // new buffer.
      memcpy(buffer_, data, buffer_size_);
      data += buffer_size_;
      size -= buffer_size_;
      void* void_buffer;
      failed_ = !output_->Next(&void_buffer, &buffer_size_);
      if (failed_) return;
      buffer_ = reinterpret_cast<char*>(void_buffer);
    }

    // Buffer is big enough to receive the data; copy it.
    memcpy(buffer_, data, size);
    buffer_ += size;
    buffer_size_ -= size;
  }

  io::ZeroCopyOutputStream* const output_;
  char* buffer_;
  int buffer_size_;
  bool at_start_of_line_;
  bool failed_;

  string indent_;
  int initial_indent_level_;
};

// ===========================================================================

TextFormat::Finder::~Finder() {
}

TextFormat::Parser::Parser()
  : error_collector_(NULL),
    finder_(NULL),
    allow_partial_(false) {
}

TextFormat::Parser::~Parser() {}

bool TextFormat::Parser::Parse(io::ZeroCopyInputStream* input,
                               Message* output) {
  output->Clear();
  ParserImpl parser(output->GetDescriptor(), input, error_collector_,
                    finder_, ParserImpl::FORBID_SINGULAR_OVERWRITES);
  return MergeUsingImpl(input, output, &parser);
}

bool TextFormat::Parser::ParseFromString(const string& input,
                                         Message* output) {
  io::ArrayInputStream input_stream(input.data(), input.size());
  return Parse(&input_stream, output);
}

bool TextFormat::Parser::Merge(io::ZeroCopyInputStream* input,
                               Message* output) {
  ParserImpl parser(output->GetDescriptor(), input, error_collector_,
                    finder_, ParserImpl::ALLOW_SINGULAR_OVERWRITES);
  return MergeUsingImpl(input, output, &parser);
}

bool TextFormat::Parser::MergeFromString(const string& input,
                                         Message* output) {
  io::ArrayInputStream input_stream(input.data(), input.size());
  return Merge(&input_stream, output);
}

bool TextFormat::Parser::MergeUsingImpl(io::ZeroCopyInputStream* input,
                                        Message* output,
                                        ParserImpl* parser_impl) {
  if (!parser_impl->Parse(output)) return false;
  if (!allow_partial_ && !output->IsInitialized()) {
    vector<string> missing_fields;
    output->FindInitializationErrors(&missing_fields);
    parser_impl->ReportError(-1, 0, "Message missing required fields: " +
                                    JoinStrings(missing_fields, ", "));
    return false;
  }
  return true;
}

bool TextFormat::Parser::ParseFieldValueFromString(
    const string& input,
    const FieldDescriptor* field,
    Message* output) {
  io::ArrayInputStream input_stream(input.data(), input.size());
  ParserImpl parser(output->GetDescriptor(), &input_stream, error_collector_,
                    finder_, ParserImpl::ALLOW_SINGULAR_OVERWRITES);
  return parser.ParseField(field, output);
}

/* static */ bool TextFormat::Parse(io::ZeroCopyInputStream* input,
                                    Message* output) {
  return Parser().Parse(input, output);
}

/* static */ bool TextFormat::Merge(io::ZeroCopyInputStream* input,
                                    Message* output) {
  return Parser().Merge(input, output);
}

/* static */ bool TextFormat::ParseFromString(const string& input,
                                              Message* output) {
  return Parser().ParseFromString(input, output);
}

/* static */ bool TextFormat::MergeFromString(const string& input,
                                              Message* output) {
  return Parser().MergeFromString(input, output);
}

// ===========================================================================

TextFormat::Printer::Printer()
  : initial_indent_level_(0),
    single_line_mode_(false),
    use_short_repeated_primitives_(false),
    utf8_string_escaping_(false) {}

TextFormat::Printer::~Printer() {}

bool TextFormat::Printer::PrintToString(const Message& message,
                                        string* output) const {
  GOOGLE_DCHECK(output) << "output specified is NULL";

  output->clear();
  io::StringOutputStream output_stream(output);

  bool result = Print(message, &output_stream);

  return result;
}

bool TextFormat::Printer::PrintUnknownFieldsToString(
    const UnknownFieldSet& unknown_fields,
    string* output) const {
  GOOGLE_DCHECK(output) << "output specified is NULL";

  output->clear();
  io::StringOutputStream output_stream(output);
  return PrintUnknownFields(unknown_fields, &output_stream);
}

bool TextFormat::Printer::Print(const Message& message,
                                io::ZeroCopyOutputStream* output) const {
  TextGenerator generator(output, initial_indent_level_);

  Print(message, generator);

  // Output false if the generator failed internally.
  return !generator.failed();
}

bool TextFormat::Printer::PrintUnknownFields(
    const UnknownFieldSet& unknown_fields,
    io::ZeroCopyOutputStream* output) const {
  TextGenerator generator(output, initial_indent_level_);

  PrintUnknownFields(unknown_fields, generator);

  // Output false if the generator failed internally.
  return !generator.failed();
}

void TextFormat::Printer::Print(const Message& message,
                                TextGenerator& generator) const {
  const Reflection* reflection = message.GetReflection();
  vector<const FieldDescriptor*> fields;
  reflection->ListFields(message, &fields);
  for (int i = 0; i < fields.size(); i++) {
    PrintField(message, reflection, fields[i], generator);
  }
  PrintUnknownFields(reflection->GetUnknownFields(message), generator);
}

void TextFormat::Printer::PrintFieldValueToString(
    const Message& message,
    const FieldDescriptor* field,
    int index,
    string* output) const {

  GOOGLE_DCHECK(output) << "output specified is NULL";

  output->clear();
  io::StringOutputStream output_stream(output);
  TextGenerator generator(&output_stream, initial_indent_level_);

  PrintFieldValue(message, message.GetReflection(), field, index, generator);
}

void TextFormat::Printer::PrintField(const Message& message,
                                     const Reflection* reflection,
                                     const FieldDescriptor* field,
                                     TextGenerator& generator) const {
  if (use_short_repeated_primitives_ &&
      field->is_repeated() &&
      field->cpp_type() != FieldDescriptor::CPPTYPE_STRING &&
      field->cpp_type() != FieldDescriptor::CPPTYPE_MESSAGE) {
    PrintShortRepeatedField(message, reflection, field, generator);
    return;
  }

  int count = 0;

  if (field->is_repeated()) {
    count = reflection->FieldSize(message, field);
  } else if (reflection->HasField(message, field)) {
    count = 1;
  }

  for (int j = 0; j < count; ++j) {
    PrintFieldName(message, reflection, field, generator);

    if (field->cpp_type() == FieldDescriptor::CPPTYPE_MESSAGE) {
      if (single_line_mode_) {
        generator.Print(" { ");
      } else {
        generator.Print(" {\n");
        generator.Indent();
      }
    } else {
      generator.Print(": ");
    }

    // Write the field value.
    int field_index = j;
    if (!field->is_repeated()) {
      field_index = -1;
    }

    PrintFieldValue(message, reflection, field, field_index, generator);

    if (field->cpp_type() == FieldDescriptor::CPPTYPE_MESSAGE) {
      if (single_line_mode_) {
        generator.Print("} ");
      } else {
        generator.Outdent();
        generator.Print("}\n");
      }
    } else {
      if (single_line_mode_) {
        generator.Print(" ");
      } else {
        generator.Print("\n");
      }
    }
  }
}

void TextFormat::Printer::PrintShortRepeatedField(
    const Message& message,
    const Reflection* reflection,
    const FieldDescriptor* field,
    TextGenerator& generator) const {
  // Print primitive repeated field in short form.
  PrintFieldName(message, reflection, field, generator);

  int size = reflection->FieldSize(message, field);
  generator.Print(": [");
  for (int i = 0; i < size; i++) {
    if (i > 0) generator.Print(", ");
    PrintFieldValue(message, reflection, field, i, generator);
  }
  if (single_line_mode_) {
    generator.Print("] ");
  } else {
    generator.Print("]\n");
  }
}

void TextFormat::Printer::PrintFieldName(const Message& message,
                                         const Reflection* reflection,
                                         const FieldDescriptor* field,
                                         TextGenerator& generator) const {
  if (field->is_extension()) {
    generator.Print("[");
    // We special-case MessageSet elements for compatibility with proto1.
    if (field->containing_type()->options().message_set_wire_format()
        && field->type() == FieldDescriptor::TYPE_MESSAGE
        && field->is_optional()
        && field->extension_scope() == field->message_type()) {
      generator.Print(field->message_type()->full_name());
    } else {
      generator.Print(field->full_name());
    }
    generator.Print("]");
  } else {
    if (field->type() == FieldDescriptor::TYPE_GROUP) {
      // Groups must be serialized with their original capitalization.
      generator.Print(field->message_type()->name());
    } else {
      generator.Print(field->name());
    }
  }
}

void TextFormat::Printer::PrintFieldValue(
    const Message& message,
    const Reflection* reflection,
    const FieldDescriptor* field,
    int index,
    TextGenerator& generator) const {
  GOOGLE_DCHECK(field->is_repeated() || (index == -1))
      << "Index must be -1 for non-repeated fields";

  switch (field->cpp_type()) {
#define OUTPUT_FIELD(CPPTYPE, METHOD, TO_STRING)                             \
      case FieldDescriptor::CPPTYPE_##CPPTYPE:                               \
        generator.Print(TO_STRING(field->is_repeated() ?                     \
          reflection->GetRepeated##METHOD(message, field, index) :           \
          reflection->Get##METHOD(message, field)));                         \
        break;                                                               \

      OUTPUT_FIELD( INT32,  Int32, SimpleItoa);
      OUTPUT_FIELD( INT64,  Int64, SimpleItoa);
      OUTPUT_FIELD(UINT32, UInt32, SimpleItoa);
      OUTPUT_FIELD(UINT64, UInt64, SimpleItoa);
      OUTPUT_FIELD( FLOAT,  Float, SimpleFtoa);
      OUTPUT_FIELD(DOUBLE, Double, SimpleDtoa);
#undef OUTPUT_FIELD

      case FieldDescriptor::CPPTYPE_STRING: {
        string scratch;
        const string& value = field->is_repeated() ?
            reflection->GetRepeatedStringReference(
              message, field, index, &scratch) :
            reflection->GetStringReference(message, field, &scratch);

        generator.Print("\"");
        if (utf8_string_escaping_) {
          generator.Print(strings::Utf8SafeCEscape(value));
        } else {
          generator.Print(CEscape(value));
        }
        generator.Print("\"");

        break;
      }

      case FieldDescriptor::CPPTYPE_BOOL:
        if (field->is_repeated()) {
          generator.Print(reflection->GetRepeatedBool(message, field, index)
                          ? "true" : "false");
        } else {
          generator.Print(reflection->GetBool(message, field)
                          ? "true" : "false");
        }
        break;

      case FieldDescriptor::CPPTYPE_ENUM:
        generator.Print(field->is_repeated() ?
          reflection->GetRepeatedEnum(message, field, index)->name() :
          reflection->GetEnum(message, field)->name());
        break;

      case FieldDescriptor::CPPTYPE_MESSAGE:
        Print(field->is_repeated() ?
                reflection->GetRepeatedMessage(message, field, index) :
                reflection->GetMessage(message, field),
              generator);
        break;
  }
}

/* static */ bool TextFormat::Print(const Message& message,
                                    io::ZeroCopyOutputStream* output) {
  return Printer().Print(message, output);
}

/* static */ bool TextFormat::PrintUnknownFields(
    const UnknownFieldSet& unknown_fields,
    io::ZeroCopyOutputStream* output) {
  return Printer().PrintUnknownFields(unknown_fields, output);
}

/* static */ bool TextFormat::PrintToString(
    const Message& message, string* output) {
  return Printer().PrintToString(message, output);
}

/* static */ bool TextFormat::PrintUnknownFieldsToString(
    const UnknownFieldSet& unknown_fields, string* output) {
  return Printer().PrintUnknownFieldsToString(unknown_fields, output);
}

/* static */ void TextFormat::PrintFieldValueToString(
    const Message& message,
    const FieldDescriptor* field,
    int index,
    string* output) {
  return Printer().PrintFieldValueToString(message, field, index, output);
}

/* static */ bool TextFormat::ParseFieldValueFromString(
    const string& input,
    const FieldDescriptor* field,
    Message* message) {
  return Parser().ParseFieldValueFromString(input, field, message);
}

// Prints an integer as hex with a fixed number of digits dependent on the
// integer type.
template<typename IntType>
static string PaddedHex(IntType value) {
  string result;
  result.reserve(sizeof(value) * 2);
  for (int i = sizeof(value) * 2 - 1; i >= 0; i--) {
    result.push_back(int_to_hex_digit(value >> (i*4) & 0x0F));
  }
  return result;
}

void TextFormat::Printer::PrintUnknownFields(
    const UnknownFieldSet& unknown_fields, TextGenerator& generator) const {
  for (int i = 0; i < unknown_fields.field_count(); i++) {
    const UnknownField& field = unknown_fields.field(i);
    string field_number = SimpleItoa(field.number());

    switch (field.type()) {
      case UnknownField::TYPE_VARINT:
        generator.Print(field_number);
        generator.Print(": ");
        generator.Print(SimpleItoa(field.varint()));
        if (single_line_mode_) {
          generator.Print(" ");
        } else {
          generator.Print("\n");
        }
        break;
      case UnknownField::TYPE_FIXED32: {
        generator.Print(field_number);
        generator.Print(": 0x");
        char buffer[kFastToBufferSize];
        generator.Print(FastHex32ToBuffer(field.fixed32(), buffer));
        if (single_line_mode_) {
          generator.Print(" ");
        } else {
          generator.Print("\n");
        }
        break;
      }
      case UnknownField::TYPE_FIXED64: {
        generator.Print(field_number);
        generator.Print(": 0x");
        char buffer[kFastToBufferSize];
        generator.Print(FastHex64ToBuffer(field.fixed64(), buffer));
        if (single_line_mode_) {
          generator.Print(" ");
        } else {
          generator.Print("\n");
        }
        break;
      }
      case UnknownField::TYPE_LENGTH_DELIMITED: {
        generator.Print(field_number);
        const string& value = field.length_delimited();
        UnknownFieldSet embedded_unknown_fields;
        if (!value.empty() && embedded_unknown_fields.ParseFromString(value)) {
          // This field is parseable as a Message.
          // So it is probably an embedded message.
          if (single_line_mode_) {
            generator.Print(" { ");
          } else {
            generator.Print(" {\n");
            generator.Indent();
          }
          PrintUnknownFields(embedded_unknown_fields, generator);
          if (single_line_mode_) {
            generator.Print("} ");
          } else {
            generator.Outdent();
            generator.Print("}\n");
          }
        } else {
          // This field is not parseable as a Message.
          // So it is probably just a plain string.
          generator.Print(": \"");
          generator.Print(CEscape(value));
          generator.Print("\"");
          if (single_line_mode_) {
            generator.Print(" ");
          } else {
            generator.Print("\n");
          }
        }
        break;
      }
      case UnknownField::TYPE_GROUP:
        generator.Print(field_number);
        if (single_line_mode_) {
          generator.Print(" { ");
        } else {
          generator.Print(" {\n");
          generator.Indent();
        }
        PrintUnknownFields(field.group(), generator);
        if (single_line_mode_) {
          generator.Print("} ");
        } else {
          generator.Outdent();
          generator.Print("}\n");
        }
        break;
    }
  }
}

}  // namespace protobuf
}  // namespace google
