// This file is modified from `yinqiwen/pbjson':
//   https://github.com/yinqiwen/pbjson

/*
 * Copyright (c) 2013-2014, yinqiwen <yinqiwen@gmail.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Redis nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SOFA_PBRPC_PBJSON_H_
#define _SOFA_PBRPC_PBJSON_H_
#include <cstdint>
#include <string>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <rapidjson/document.h>

#define ERR_INVALID_ARG -1
#define ERR_INVALID_PB -2
#define ERR_UNKNOWN_FIELD -3
#define ERR_INVALID_JSON -4

namespace sofa {
namespace pbrpc {

void pb2json(const google::protobuf::Message* msg, std::string& str);
rapidjson::Value* pb2jsonobject(const google::protobuf::Message* msg);
rapidjson::Value* pb2jsonobject(const google::protobuf::Message* msg,
        rapidjson::Value::AllocatorType& allocator);
void json2string(const rapidjson::Value* json, std::string& str);

int json2pb(const std::string& json, google::protobuf::Message* msg, std::string& err);
int jsonobject2pb(const rapidjson::Value* json, google::protobuf::Message* msg, std::string& err);

} // namespace pbrpc
} // namespace sofa

#endif // _SOFA_PBRPC_PBJSON_H_

/* vim: set ts=4 sw=4 sts=4 tw=100 */
