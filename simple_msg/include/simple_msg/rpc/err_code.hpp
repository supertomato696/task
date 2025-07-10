#pragma once
#include <cstdint>
#include "expected.hpp"

namespace simple::msg {
    template <typename T> using rpc_result = expected<T, int32_t>;
    using rpc_unexpected_result = unexpected<int32_t>;

    constexpr static int32_t err_network = 1;
    constexpr static int32_t err_deserialize = 3;
}
