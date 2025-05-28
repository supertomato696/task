#include "LinuxAppProcessManager.hpp"

#include <atomic>
#include <thread>
#include <vector>
#include <asio/io_context.hpp>
#include "SignalHandler.hpp"
#define NUM_PROCS  5
#include <iostream>

int main(int argc, char* argv[])
    {
asio::io_context io;

  std::atomic<int> exitCounter{0};



  SignalHandler::instance().register_default_signals();
  SignalHandler::instance().register_handler(SIGINT, [&](int) {
      io.stop();
  });

  SignalHandler::instance().start_worker();

  LinuxAppProcessManager procMgr{io};
  procMgr.registerExitCallback([&](const ChildProcessInfo& info)
  {
                                 exitCounter++;
    std::cout << "Process " << info.instanceId << " exited with code " << info.exitCode << std::endl;
  });

  std::vector<LinuxAppInfo> apps;
  for (int i{0}; i < NUM_PROCS; ++i)
  {
    LinuxAppInfo app;
    app.instanceId = "sleep_" + std::to_string(i);
    app.execPath = "/bin/sleep";
    app.entrance = {  "2" };
    apps.push_back(app);

}

std::vector<std::thread> threads;
for (auto& app : apps) {
  threads.emplace_back([&]() {
    procMgr.start(app);
  });
  }



  // for (auto& app :apps) {
  //          procMgr.stop_linux_app(app);
  // };

  // while (exitCounter < NUM_PROCS) {
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }
  for (auto& t : threads) {
    if (t.joinable())
      t.join();
  }

  io.run();
  std::cout << "All processes exited" << std::endl;





return 0;

}

