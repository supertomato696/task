#!/usr/bin/env bash
set -e

TP=third_party
mkdir -p "$TP"

echo "1) asio ..."
git clone --depth 1 --branch asio-1-28-0 https://github.com/chriskohlhoff/asio.git \
    "$TP/asio"

echo "2) nlohmann/json ..."
mkdir -p "$TP/json"
curl -L https://github.com/nlohmann/json/releases/download/v3.11.2/json.hpp \
     -o "$TP/json/json.hpp"
# 也可: curl -L .../json-3.11.2.zip && unzip

echo "3) inja ..."
git clone --depth 1 --branch v3.4.0 https://github.com/pantor/inja.git \
    "$TP/inja"

echo "4) cpp-httplib ..."
git clone --depth 1 --branch v0.15.3 https://github.com/yhirose/cpp-httplib.git \
    "$TP/httplib"

echo "5) sqlite_modern_cpp ..."
git clone --depth 1 --branch v3.2 https://github.com/SqliteModernCpp/sqlite_modern_cpp.git \
    "$TP/sqlite_modern_cpp"

echo "6) json-schema-validator (optional) ..."
git clone --depth 1 --branch v2.1.0 https://github.com/pboettch/json-schema-validator.git \
    "$TP/json-schema-validator"

echo "7) SQLite amalgamation ..."
curl -L https://www.sqlite.org/2025/sqlite-autoconf-3460000.tar.gz | \
    tar -xz -C "$TP"
mv "$TP/sqlite-autoconf-3460000" "$TP/sqlite-amalgamation"

echo "8) OpenSSL 3.3.0 ..."
curl -L https://www.openssl.org/source/openssl-3.3.0.tar.gz | \
    tar -xz -C "$TP"

echo "done."
