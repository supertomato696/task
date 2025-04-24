// Simple MCP JSON-RPC WebSocket Server Example
// Requires: WebSocket++ (asio_no_tls), nlohmann/json

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <thread>
#include <chrono>

using json = nlohmann::json;
typedef websocketpp::server<websocketpp::config::asio> server;

int main() {
    server ws_server;

    // Initialize Asio
    ws_server.init_asio();

    // Set message handler
    ws_server.set_message_handler([&](websocketpp::connection_hdl hdl, server::message_ptr msg) {
        try {
            // Parse request
            json req = json::parse(msg->get_payload());
            json resp;
            resp["jsonrpc"] = "2.0";
            resp["id"] = req.value("id", nullptr);

            std::string method = req.value("method", "");
            if (method == "prompts/list") {
                // Return static prompt list
                json result;
                result["prompts"] = json::array({
                    { {"name","greet"}, {"description","Greet a user"} },
                    { {"name","farewell"}, {"description","Bid farewell"} }
                });
                resp["result"] = result;

            } else if (method == "prompts/get") {
                auto params = req.at("params");
                std::string name = params.value("name", "");
                json result;
                // Provide messages based on name
                if (name == "greet") {
                    result["messages"] = json::array({
                        {
                            {"role","assistant"},
                            {"content", {
                                {"type","text"},
                                {"text","Hello, {{user_name}}! Welcome to our service."}
                            }}
                        }
                    });
                } else if (name == "farewell") {
                    result["messages"] = json::array({
                        {
                            {"role","assistant"},
                            {"content", {
                                {"type","text"},
                                {"text","Goodbye, {{user_name}}! Have a great day."}
                            }}
                        }
                    });
                } else {
                    // Unknown prompt
                    resp["error"] = { {"code", -32602}, {"message","Unknown prompt name"} };
                    ws_server.send(hdl, resp.dump(), msg->get_opcode());
                    return;
                }
                resp["result"] = result;

            } else {
                // Method not found
                resp["error"] = { {"code", -32601}, {"message","Method not found"} };
            }

            // Send response
            ws_server.send(hdl, resp.dump(), msg->get_opcode());
        } catch (const std::exception& e) {
            json err;
            err["jsonrpc"] = "2.0";
            err["error"] = { {"code", -32700}, {"message", e.what()} };
            ws_server.send(hdl, err.dump(), msg->get_opcode());
        }
    });

    // Listen on port 9002
    ws_server.listen(9002);
    ws_server.start_accept();

    std::cout << "MCP WebSocket server listening on ws://0.0.0.0:9002/" << std::endl;

    // Run the Asio loop
    ws_server.run();

    return 0;
}
