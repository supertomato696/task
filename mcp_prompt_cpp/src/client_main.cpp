#include "mcp/client/McpClient.hpp"
#include <iostream>

int main(){
    mcp::client::McpClient cli("127.0.0.1", 9275);

    auto list = cli.listPrompts();
    for(auto& p:list) std::cout << p.name << "\n";

    auto res = cli.getPrompt("echo_audio", {{"audio","file:///tmp/a.wav"}});
    for(auto& m: res.messages){
        std::cout << m.role << ": " << m.content.dump() << "\n";
    }
}