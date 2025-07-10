#include <catch2/catch_test_macros.hpp>
#include <simple_msg/detail/ibuffer.hpp>

TEST_CASE( "construct buffer(default)") {
    simple::msg::ibuffer buf1{};
    REQUIRE(buf1.empty() == true);
    REQUIRE(buf1.data() == nullptr);
    REQUIRE(buf1.size() == 0);
}

TEST_CASE( "construct buffer(len == 0, buf == nullptr)") {
    simple::msg::ibuffer buf1{0, nullptr};
    REQUIRE(buf1.empty() == true);
    REQUIRE(buf1.data() == nullptr);
    REQUIRE(buf1.size() == 0);
}


TEST_CASE( "construct buffer(len = 0, buf != nullptr)") {
    uint8_t data = 'a';
    simple::msg::ibuffer buf1{0, &data};

    REQUIRE(buf1.empty() == true);
    REQUIRE(buf1.data() == nullptr);
    REQUIRE(buf1.size() == 0);
}


TEST_CASE( "construct buffer(len != 0, buf == nullptr)") {
    simple::msg::ibuffer buf1{1, nullptr};

    REQUIRE(buf1.empty() == false);
    REQUIRE(buf1.data() != nullptr);
    REQUIRE(buf1.size() != 0);
}

TEST_CASE( "construct buffer(len != 0, buf != nullptr)") {
    uint8_t data = 'a';
    simple::msg::ibuffer buf1{1, &data};

    REQUIRE(buf1.empty() == false);
    REQUIRE(buf1.data() != nullptr);
    REQUIRE(buf1.size() != 0);
    REQUIRE(memcmp(buf1.data(), &data, buf1.size()) == 0);
}

TEST_CASE("copy constructor(copy empty buffer)") {
    simple::msg::ibuffer buf2{};
    simple::msg::ibuffer buf1{buf2};

    REQUIRE(buf1.empty() == true);
    REQUIRE(buf1.data() == nullptr);
    REQUIRE(buf1.size() == 0);
}

TEST_CASE("copy constructor(copy not empty buffer)") {
    uint8_t data = 'a';
    simple::msg::ibuffer buf2{1, &data};
    simple::msg::ibuffer buf1{buf2};

    REQUIRE(buf1.empty() == false);
    REQUIRE(buf1.data() != nullptr);
    REQUIRE(buf1.data() != buf2.data());
    REQUIRE(buf1.size() == buf2.size());
    REQUIRE(buf2.size() == 1);
    REQUIRE(memcmp(buf1.data(), buf2.data(), buf1.size()) == 0);
}