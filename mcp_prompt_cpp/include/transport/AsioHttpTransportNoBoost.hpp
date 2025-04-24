#pragma once

#include <string>
#include <sstream>
#include <asio.hpp>
#include <nlohmann/json.hpp>
#include "McpClient.hpp"

namespace mcp::client {

using json = nlohmann::json;

/// HTTP Transport implementation using standalone Asio (no Boost.Beast)
class AsioHttpTransportNoBoost : public Transport {
public:
    /// Construct with IO context, target host, port, and path
    AsioHttpTransportNoBoost(asio::io_context& ioc,
                             const std::string& host,
                             const std::string& port = "80",
                             const std::string& target = "/")
        : io_context_(ioc),
          host_(host), port_(port), target_(target) {}

    /// Send JSON-RPC request over HTTP POST and return parsed JSON response
    json send(const json& request) override {
        // Serialize request JSON
        std::string body = request.dump();

        // Resolve the host
        asio::ip::tcp::resolver resolver(io_context_);
        auto endpoints = resolver.resolve(host_, port_);

        // Create and connect the socket
        asio::ip::tcp::socket socket(io_context_);
        asio::connect(socket, endpoints);

        // Build HTTP request
        std::ostringstream req_stream;
        req_stream << "POST " << target_ << " HTTP/1.1\r\n";
        req_stream << "Host: " << host_ << "\r\n";
        req_stream << "User-Agent: mcp-cpp-sdk\r\n";
        req_stream << "Content-Type: application/json\r\n";
        req_stream << "Content-Length: " << body.size() << "\r\n";
        req_stream << "Connection: close\r\n";
        req_stream << "\r\n";
        req_stream << body;
        std::string request_str = req_stream.str();

        // Send the request
        asio::write(socket, asio::buffer(request_str));

        // Read the response status line
        asio::streambuf response_buf;
        asio::read_until(socket, response_buf, "\r\n");

        std::istream response_stream(&response_buf);
        std::string http_version;
        unsigned int status_code;
        response_stream >> http_version >> status_code;
        std::string status_message;
        std::getline(response_stream, status_message);
        if (!response_stream || http_version.substr(0, 5) != "HTTP/") {
            throw std::runtime_error("Invalid HTTP response");
        }
        if (status_code != 200) {
            throw std::runtime_error("HTTP status code " + std::to_string(status_code));
        }

        // Read headers
        asio::read_until(socket, response_buf, "\r\n\r\n");
        std::size_t content_length = 0;
        std::string header;
        while (std::getline(response_stream, header) && header != "\r") {
            if (header.find("Content-Length:") == 0) {
                content_length = std::stoul(header.substr(15));
            }
        }

        // Read body
        std::string body_str;
        if (response_buf.size() > 0) {
            std::ostringstream ss;
            ss << &response_buf;
            body_str = ss.str();
        }
        if (body_str.size() < content_length) {
            std::vector<char> extra(content_length - body_str.size());
            asio::read(socket, asio::buffer(extra));
            body_str.append(extra.data(), extra.size());
        }

        // Close the socket
        asio::error_code ec;
        socket.shutdown(asio::ip::tcp::socket::shutdown_both, ec);
        // ignore not_connected

        // Parse and return JSON
        return json::parse(body_str);
    }

private:
    asio::io_context& io_context_;
    std::string host_;
    std::string port_;
    std::string target_;
};

} // namespace mcp::client