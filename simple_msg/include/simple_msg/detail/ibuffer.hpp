#pragma once
#include <cassert>
#include <memory.h>

namespace simple::msg {

    class ibuffer {
    public:
        explicit ibuffer(uint32_t len = 0, uint8_t* buf = nullptr) {
            construct(buf, len);
        }
        
        ~ibuffer() {
            destruct();
        }
        
        ibuffer(const ibuffer& other) : ibuffer(other.len_, other.buf_) {
        }

        ibuffer(ibuffer&& other)
        : buf_(nullptr)
        , len_(0) {
            std::swap(buf_, other.buf_);
            std::swap(len_, other.len_);
        }

        ibuffer& operator=(ibuffer& other) {
            if (this != &other) {
                destruct();
                construct(other.buf_, other.len_);
            }
            
            return *this;
        }
        
        ibuffer& operator=(ibuffer&& other) {
            if (this != &other) {
                destruct();
                std::swap(buf_, other.buf_);
                std::swap(len_, other.len_);
            }
            
            return *this;
        }
        
        bool empty() const {
            return len_ == 0;
        }

        uint8_t* data() const {
            return buf_;
        }

        uint32_t size() const {
            return len_;
        }
        
    private:
        void destruct() {
            delete[] buf_;
            buf_ = nullptr;
            len_ = 0;
        }
        
        void construct(uint8_t* buf, uint32_t len) {
            if (len > 0) {
                buf_ = new uint8_t[len];
            } else {
                buf_ = nullptr;
            }

            len_ = len;

            if (buf_ != nullptr) {
                if (buf != nullptr) {
                    memcpy(buf_, buf, len_);
                } else {
                    memset(buf_, 0, len_);
                }
            }
        }
    private:
        uint8_t* buf_{};
        uint32_t len_{};
    };
}
