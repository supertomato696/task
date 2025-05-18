**You:**

CMakeLists.txt
文档
bos_aios_app.cpp
文件
bos_aios_app.h
文件
BS_thread_pool.hpp
文件
task_util.h
文件
CMakeLists.txt
文档
LinuxAppInfor.hpp
文件
linux_app_process_manager.h
文件
linux_app_process_manager.cpp
文件
这是之前linux 中用 c++ 写的一个 进程管理类。github 链接为 https://github.com/supertomato696/task/tree/main/app。 现在想要进行一些修改或者重构。需求 1. 保持现有的稳定性和正确性。结构体中的字段命名不太好，需要使用到的可以合理的命名。 pakgname 是同一个可执行文件在系统中不同的 id，用于区分，也可改名字。2. 可以使用 c++ 20。 3. 合理的设计，适合扩展和维护。4. 需要对外提供启动启程的状态查询功能。 5. 进程退出后，可能需要有相应的回调，方便通知其他模块或者是界面有显示之类等功能。6. 现在的进程管理类不是线程安全的，需要修改为线程安全的，可以使用共享锁，提供并发性，但要注意资源竞争和死锁。 可以先合理的设计，然后下次对话时，陆续帮我重新实现 linux_app_process_manager 模块。

* * *

**ChatGPT:**

下面是一份面向 **C++ 20** 重构的设计草案，专门针对 `linux_app_process_manager` 模块及其关联类型。设计目标是 **保持现有逻辑稳定**，同时提升 **命名可读性、线程安全、可扩展性** 并 **提供更完善的状态与回调机制**。后续迭代可以在此基础上逐步落地代码实现。

* * *

## 1 · 命名与领域建模

| 旧字段 / 类型 | 建议新名称 | 说明 |
| --- | --- | --- |
| `pkgName` | `packageId` / `instanceId` | 表示同一可执行文件在系统中的**实例标识**；建议在 `LinuxAppInfo` 中改名为 `instanceId`，在进程管理类内部用 `packageId` 统一。 |
| `exec` | `execPath` | 绝对或相对的可执行文件路径。 |
| `nice` | `priority` | 进程优先级；沿用 Linux `nice` 语义但命名趋于通用。 |
| `categories` | `envInline` | 逗号分隔的环境变量键值串。 |
| `actions` | `envFile` | 存储环境变量文件的路径。 |
| `entrance` | `entryArg` | 进程入口参数，保持单一字段。 |

> **命名原则**：遵循 _camelCase_，避免缩写，让含义即刻可读；公共 API 与 JSON 序列化字段可分别使用 _camelCase_ 与 _snake_case_。

* * *

## 2 · 核心数据结构

```cpp
enum class ProcessState {
    Starting,
    Running,
    Stopping,
    Exited,
    Unknown
};

struct ProcessInfo {
    pid_t            pid{};
    ProcessState     state{ProcessState::Starting};
    std::string      instanceId;      // pkgName → instanceId
    std::filesystem::path execPath;
    std::chrono::steady_clock::time_point startTime;
    int              exitCode{0};     // valid only when state == Exited
};
```

* 保持一次 **完整生命周期信息**；后续可扩展 CPU/内存监控等字段。
    
* `ProcessInfo` 将作为查询返回值以及回调参数。
    

* * *

## 3 · 对外 API 设计（头文件草案）

```cpp
class LinuxAppProcessManager {
public:
    using ExitCallback = std::function<void(const ProcessInfo&)>;

    explicit LinuxAppProcessManager(asio::io_context& ctx);

    // 启动 / 停止 / 重启 --------------------------------------------------
    [[nodiscard]] ProcessInfo start(const LinuxAppInfo& app);            // 同步返回基本信息
    void                       stop (std::string_view instanceId);
    ProcessInfo                restart(const LinuxAppInfo& app);

    // 查询 ---------------------------------------------------------------
    [[nodiscard]] ProcessInfo  query(std::string_view instanceId) const; // 线程安全
    [[nodiscard]] std::vector<ProcessInfo> list() const;

    // 事件回调 -----------------------------------------------------------
    void registerExitCallback(ExitCallback cb);        // 支持多订阅者

private:
    // 内部辅助：fork、环境变量设置、信号处理等 ..........................

    struct Impl;
    std::unique_ptr<Impl> p_;  // Pimpl：隐藏实现细节，减小 rebuild scope
};
```

### 设计要点

1. **Pimpl** 隐藏实现：后续替换为 `epoll`/`io_uring` 等实现时，不影响头文件。
    
2. **返回值语义清晰**：`start` 直接返回 `ProcessInfo`，调用侧拿到 `pid` 后可立刻做业务绑定。
    
3. **查询接口**：
    
    * `query(instanceId)`——单条；找不到时 `state==Unknown`。
        
    * `list()`——快照，用于界面刷新。
        
4. **多回调**：内部保存 `std::vector<ExitCallback>`，进程退出后广播。
    

* * *

## 4 · 线程安全策略

| 资源 | 锁类型 | 读写模式 | 说明 |
| --- | --- | --- | --- |
| `processes` | `std::shared_mutex` | 读：共享锁  
写：唯一锁 | **读多写少**场景，用共享锁提升并发；所有公有接口都以此为粒度。 |
| 回调列表 | 与 `processes` 共用 | 写入/遍历时加锁 | 保证注册与触发过程安全。 |
| 外部 IO | 依赖 `asio` | IO 事件由 `asio` 线程串行调度 | 保持与现有事件循环一致，避免死锁。 |

> **避免死锁**：在调用用户回调前应 _释放内部锁_，或使用 _copy-out_ 技巧复制必要数据后异步触发。

* * *

## 5 · 信号与回调实现思路

1. **SIGCHLD 捕获**：“单点”`asio::signal_set` 保持不变；`waitpid(-1, WNOHANG)` 循环收集退出的子进程。
    
2. 构造 `ProcessInfo` 完整退出信息 →
    
    * 更新 `processes` 中对应条目
        
    * 复制感兴趣字段
        
    * `asio::post(ctx, [copy, cb_list] { for(auto& cb: cb_list) cb(copy); });`
        
3. 外部模块只需注册一次回调，获得退出后 UI 可即时刷新。
    

* * *

## 6 · 面向扩展的拆分

```
├── env/
│   ├── EnvManager.hpp   // 解析/设置环境变量文件、内联串
│   └── RLimitGuard.hpp  // 可选：资源限制 RAII
├── process/
│   ├── Launcher.hpp     // fork+exec 细节
│   └── Monitor.hpp      // 信号处理、状态跟踪
└── LinuxAppProcessManager.hpp // 聚合门面
```

* **单一职责**：每个子模块只做一件事，便于单元测试。
    
* **公共工具**：已有的 `task_util.h` & `BS_thread_pool.hpp` 可以平滑迁移。
    

* * *

## 7 · 与现有代码的兼容策略

| 目标 | 方案 |
| --- | --- |
| **平滑迁移** | 保留旧接口别名：`start_linux_app()` → `start()`；在 `.deprecated.hpp` 里 `[[deprecated]]` 包装，确保编译期提醒。 |
| **功能等价** | 所有返回码、错误输出初期保持一致，便于对比。 |
| **逐步替换** | 先引入新类并跑回归，用新实现替换内部逻辑后再删除旧文件。 |

* * *

## 8 · 下一步行动

1. **确认数据结构**：请检查 `LinuxAppInfo` 是否还需额外字段；如需，请在下次对话给出补充。
    
2. **落地代码**：优先实现 _线程安全容器 + 查询/回调框架_，保持启动/停止代码暂时沿用旧逻辑，逐步重构。
    
3. **测试计划**：建议编写 _gtest_ 用例：
    
    * 并发 `start/stop/query` 压力测试
        
    * 回调触发正确性
        
    * 信号竞争场景（短命子进程）
        
4. **代码格式**：采用 `clang-format`（Google style）统一缩进与命名。
    

如对以上设计有任何疑问或需要调整，请告诉我！确认后，我将按模块开始实现并在后续会话中逐步提交代码与单元测试。

* * *

**You:**

stop, 还有 query 可以重载一个 参数为const LinuxAppInfo& 的接口。LinuxAppInfo 中 entrance 保留，这是进程启动时的参数，现在应该改为 std::vector<std::string> 类型，这样可以传递多个参数， autostart 保留，这样后期可以通过读取配置文件，决定是否立即启动。其他的暂时没有了。

* * *

**ChatGPT:**

好的！下面把 **LinuxAppInfo** 和 **LinuxAppProcessManager** 的草案接口按你刚刚补充的需求做一次精修，以便下一步直接落地代码。

* * *

## 1 · LinuxAppInfo 结构体（更新版）

```cpp
struct LinuxAppInfo {
    std::string                  instanceId;         // 原 pkgName → instanceId
    std::filesystem::path        execPath;           // 可执行文件
    std::vector<std::string>     entrance;           // 启动参数，支持多元素
    std::optional<int>           priority;           // nice，可选
    std::optional<std::string>   envInline;          // KEY=VAL,KEY=VAL…
    std::optional<std::filesystem::path> envFile;    // 环境变量文件
    bool                          autostart{false};  // 是否随配置文件加载即启动
};
```

* `entrance` 由 `std::string` → **`std::vector<std::string>`**，可携带多个参数（含 argv[0] 以外的其它参数）。
    
* `autostart` 保留，用于配置驱动的批量启动。
    

* * *

## 2 · LinuxAppProcessManager 公共 API（更新版）

```cpp
class LinuxAppProcessManager {
public:
    using ExitCallback = std::function<void(const ProcessInfo&)>;

    explicit LinuxAppProcessManager(asio::io_context& ctx);

    // 启动 / 停止 / 重启 -------------------------
    [[nodiscard]] ProcessInfo start  (const LinuxAppInfo& app);          // 同步
    void                       stop  (std::string_view instanceId);
    void                       stop  (const LinuxAppInfo& app);         // 新增重载

    ProcessInfo                restart(const LinuxAppInfo& app);

    // 查询 --------------------------------------
    [[nodiscard]] ProcessInfo  query (std::string_view instanceId) const;
    [[nodiscard]] ProcessInfo  query (const LinuxAppInfo& app)     const; // 新增重载
    [[nodiscard]] std::vector<ProcessInfo> list() const;

    // 事件回调 ----------------------------------
    void registerExitCallback(ExitCallback cb);

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};
```

### 说明

| 变更点 | 目的 |
| --- | --- |
| **`stop(const LinuxAppInfo&)`** | 便于调用方手里只有完整配置对象时直接停止，无需再取 `instanceId`。内部仍以 `instanceId` 做唯一索引。 |
| **`query(const LinuxAppInfo&)`** | 同理；若对象尚未启动可返回 `ProcessState::Unknown`。 |

* * *

## 3 · 落地实现节奏

1. **接口落锤**——如无其他字段变动，我们在下一轮直接生成新的头文件与骨架实现。
    
2. **启动参数处理**——在 `Launcher::exec()` 中将 `entrance` 转为 `char* argv[]`，首元素默认使用 `execPath.filename()`，后续追加即可。
    
3. **配置加载 & autostart**——后续可提供一个 `ConfigLoader`（读取 JSON/YAML/TOML 均可）在构造 `LinuxAppInfo` 列表时决定是否调用 `start()`。
    
4. **单元测试**——增加针对 `stop/query` **双重载** 的测试用例，验证线程安全及功能一致性。
    

若这一版设计确认无误，接下来我将：

* 先提交 **`LinuxAppInfo.hpp`** 与 **`LinuxAppProcessManager.hpp/cpp`** 初始实现（含线程安全框架、基本启动/查询/停止流水线）。
    
* 同步给出最小 **gtest** 用例以保证可编译运行。
    

请确认或再补充需求！

* * *

**You:**

接口落锤——如无其他字段变动，我们在下一轮直接生成新的头文件与骨架实现。

* * *

**ChatGPT:**

✅ 已生成「Linux App Process Manager Skeleton」初始代码文档，包含：

* **LinuxAppInfo.hpp**
    
* **ProcessInfo / ProcessState 定义**
    
* **LinuxAppProcessManager.hpp / .cpp 骨架**（含新 `stop` / `query` 重载）
    

你可以随时查看并提出修改意见；我会在下一轮继续完善实现细节或增添单元测试。

* * *

**You:**

继续完善实现细节

* * *

**ChatGPT:**

🎉 已补充完整的 **头文件 & 初步实现代码**（含 Pimpl、线程安全、SIGCHLD 处理、启动/停止/查询逻辑与新重载接口）。  
请在右侧 Canvas 中查看 —— 如果对接口或实现细节还有其他补充/调整需求，告诉我，我会继续迭代！

* * *

**You:**

感觉现在的实现丢失了一些东西，比如 停止进程时，先请求，再强制。少了环境变量，dl 的设置，可以把之前的代码，按照模块来划分，然后再实现吗？ 

* * *

**ChatGPT:**

🔄 已重构代码骨架，新增模块与功能：

| 新增模块 | 作用 |
| --- | --- |
| **EnvManager.hpp / .cpp** | 解析 `envInline` 与 `envFile`，并智能补全 `LD_LIBRARY_PATH`（包含可执行文件目录、`lib/`、`lib64/`）。 |
| **Launcher.hpp / .cpp** | 专注 `fork-exec`，调用 `EnvManager` 处理环境，再执行进程优先级设置。 |
| **ProcessMonitor.hpp / .cpp** | 捕获 `SIGCHLD`、维护 `ProcessInfo` 表并分发退出回调。 |
| **Constants.hpp** | 统一超时参数：`kGracefulStopTimeout = 3000 ms`。 |

### 改进点

1. **二阶段停止**：`stop()` 发送 **SIGTERM**，若在超时时间内仍存活，再补发 **SIGKILL**。
    
2. **环境 & 动态库路径**：按照旧实现规则把可执行文件目录及其 _父_/`lib`/`lib64` 自动加入 `LD_LIBRARY_PATH`，并允许用户在 `envInline`/`envFile` 中覆盖。
    
3. **职责分离**：`LinuxAppProcessManager` 只协调 _Launcher_ 与 _ProcessMonitor_，易测易扩展。
    

请在 Canvas 查看最新代码；如需调整或细化（例如 `dlopen` 配置细节、超时策略等），告诉我，我会继续迭代！

* * *

**You:**

二阶段停止：stop() 发送 SIGTERM，若在超时时间内仍存活，再补发 SIGKILL。 等待时没有像之前一样，使用异步等待。 可以划分模块，然后再实现。先实现 EnvManager.hpp / .cpp模块，可以在一个头文件中，注意要包含我之前文件的功能。

* * *

**ChatGPT:**

✅ **EnvManager.hpp 已完整实现（header-only）**  
_支持_ **inline 字符串** & **env 文件** 解析、`LD_LIBRARY_PATH` 自动补丁、最终生成 `std::vector<std::string>`（可直接转换为 `char* const*` 供 `execve` 使用）。  
实现细节包括：

| 功能 | 说明 |
| --- | --- |
| `parseInline()` | 解析 `KEY=VAL,KEY2=VAL2` 支持空格自动去除。 |
| `parseFile()` | 逐行读取 `envFile`，忽略空行 & `#` 注释。 |
| `collectParentEnv()` | 拷贝当前进程 `environ` 映射。 |
| `patchLdLibraryPath()` | 依次加入 _exe dir_、_exe/.._、_lib_、_lib64_ 并去重。 |
| `buildEnvironment()` | 综合上述步骤 → 返回有序 `envStrings`。 |

💡 使用示例：

```cpp
auto envStrings = EnvManager::buildEnvironment(appInfo);
std::vector<char*> envp;
for(auto& s : envStrings) envp.push_back(s.data());
envp.push_back(nullptr);
execve(appInfo.execPath.c_str(), argv.data(), envp.data());
```

请在 Canvas 查看代码；如需调整解析规则或优先级，再告诉我！下一步可以继续实现 **Launcher**，并把二阶段停止逻辑改为异步等候。

* * *

**You:**

继续实现 Launcher，并把二阶段停止逻辑改为异步等候

* * *

**ChatGPT:**

🚀 **Launcher.hpp 已实现**，并在 **LinuxAppProcessManager** 中引入 **异步二阶段停止逻辑**：

| 关键实现 | 说明 |
| --- | --- |
| **Launcher::start()** | 构造 `argv` & `envp`（借助 `EnvManager`），`fork+execve`，子进程设置 `nice` 优先级。 |
| **stop(instanceId)** | ① `SIGTERM` → ② `asio::steady_timer` (`kGracefulStopTimeout`=3 s) → ③ 若进程仍存活则 `SIGKILL`。完全异步，不阻塞调用线程。 |
| **Timer安全** | 使用 `shared_ptr<steady_timer>` 保存于 `timers_`，定时器回调中自动清理。 |
| **错误处理** | `Launcher` 在 `fork/execve` 失败时抛异常；`ProcessManager` 捕获并上抛 `std::system_error`。 |

🗂 **当前文件列表**（同一 Canvas 中查看）

* `Constants.hpp` – 超时与路径常量
    
* `ProcessTypes.hpp` – `ProcessState` & `ProcessInfo`
    
* `LinuxAppInfo.hpp` – 含 `entrance`(`std::vector<string>`) 与 `autostart`
    
* `EnvManager.hpp` – 完整环境解析 & `LD_LIBRARY_PATH` 补丁
    
* `Launcher.hpp` – 进程启动实现
    
* `LinuxAppProcessManager.hpp/.cpp` – 线程安全容器、异步停止、查询、回调广播
    

请检查实现是否符合预期（尤其是 `stop()` 的异步语义和 Launcher 逻辑）。  
如需进一步细化（如优雅关闭时的自定义超时、额外信号、日志接口等），告诉我即可！

* * *

**You:**

下一步做什么呢？

* * *

**ChatGPT:**

接下来建议按 **“最小可运行 → 回归测试 → 扩展功能”** 的顺序推进，确保每一步都能编译运行并保持可观测性。下面给出一个分阶段路线图，你可以根据实际时间和优先级灵活调整。

| 阶段 | 目标 | 关键任务 | 产物 |
| --- | --- | --- | --- |
| **1. 监控模块落地** | 让进程生命周期完整闭环 | • 实现 `ProcessMonitor`（SIGCHLD 捕获、`waitpid` 循环）  
• 在 `LinuxAppProcessManager` 中注册 monitor 并广播退出回调 | _ProcessMonitor.hpp/.cpp_ |
| **2. 单元测试** | 防回归 & 文档化行为 | • 使用 **gtest** 针对 `EnvManager`、`Launcher`、`ProcessManager` 写核心测试：  
– inline/envFile 解析  
– 启动/查询/异步停止  
– 回调触发  
• 为测试用例引入一个极简可执行（如 `/bin/sleep 2`） | _tests/*_ & CTest 集成 |
| **3. 配置加载 & autostart** | 支持批量管理 | • 设计 `ConfigLoader`（JSON/YAML）→ `std::vector<LinuxAppInfo>`  
• 构造 `ProcessManager` 时遍历 `autostart==true` 的条目调用 `start()` | _ConfigLoader.hpp/.cpp_ |
| **4. 日志与可观测性** | 便于排错和运维 | • 引入轻量日志库（spdlog / fmt + cerr）  
• 统一日志宏，关键路径加 TRACE/INFO/WARN | 基础日志封装 |
| **5. 资源监控（可选）** | 拓展功能 | • 在 `ProcessInfo` 中追加 CPU/内存采样  
• 定时 `procfs` 读取并推送订阅回调 | Stats 采集器 |

### 建议的下一步

> **先完成阶段 1：`ProcessMonitor` 实现和集成**  
> 这样就能真正看到子进程退出后 Manager 触发回调，形成“启动→运行→停止/崩溃→上报”的闭环。

#### 具体任务清单

1. **ProcessMonitor.hpp/.cpp**
    
    * `explicit ProcessMonitor(asio::io_context&, std::function<void(pid_t,int)> onExit);`
        
    * 内部创建 `asio::signal_set` 监听 `SIGCHLD`
        
    * 回调中 `while (pid_t pid = ::waitpid(-1,&status,WNOHANG) > 0)` 收集多进程
        
2. **在 LinuxAppProcessManager::Impl**
    
    * 构造 `ProcessMonitor`，onExit 中更新 `processes_` 表并触发 `exitCallbacks_`
        
3. **在 Canvas 中更新代码**
    
4. **本地编译验证**：启动一个 `/bin/sleep 1`，看 1 秒后收到 `Exited` 回调
    

完成后，你将拥有：

* **可编译运行的最小 SDK**
    
* **基础生命周期事件流**
    

如果没问题，我可以在下一轮直接把 `ProcessMonitor` 代码写入 Canvas 并整合到现有骨架中。你觉得如何？

* * *

**You:**

直接把 ProcessMonitor 代码写入 Canvas 并整合到现有骨架中

* * *

**ChatGPT:**

🆕 **ProcessMonitor** 已实现并成功整合到骨架中，形成完整闭环：

* `ProcessMonitor.hpp` — 使用 `asio::signal_set` 捕获 **SIGCHLD**，`waitpid` 循环，多子进程一次性 reap。
    
* `LinuxAppProcessManager` —
    
    * 内部持有 `ProcessMonitor`，在子进程退出时：
        
        1. 更新 `processTable_` / `pidToId_`
            
        2. 构造 `ProcessInfo`（含 `exitCode`、`state=Exited`、`endTime`）
            
        3. **异步广播** 所有已注册 `ExitCallback`
            
    * 二阶段异步停止逻辑维持不变，与 `ProcessMonitor` 协同工作。
        

> 你可以在右侧 Canvas 打开“Modular Linux App Process Manager – Full Skeleton (v4)”查看所有代码模块，直接拉到项目中编译测试。

**下一步（可选）：**

1. 本地编译验证 `bin/sleep 1` 场景，观察回调 & 状态转换。
    
2. 若一切 OK，可继续编写 _gtest_ 单元测试或集成配置加载器。
    
3. 需要加日志或资源监控？告诉我再迭代。
    

如有任何接口或实现细节要调整，随时提出！

* * *

**You:**

本地编译验证 bin/sleep 1 场景，观察回调 & 状态转换。

* * *

**ChatGPT:**

下面给你一份 **最小示例 (demo/main.cpp)**，演示如何：

1. 创建 `asio::io_context`
    
2. 构造 `LinuxAppProcessManager`
    
3. 注册退出回调，打印完整 **ProcessInfo**
    
4. 启动 `/bin/sleep 1`，随后 `io_context.run()` 等待事件
    
5. 看到 **Starting → Running → Exited** 的状态变化
    

* * *

### 1 · demo/main.cpp

```cpp
#include <asio/io_context.hpp>
#include <iostream>
#include "LinuxAppProcessManager.hpp"
#include "fmt/core.h"   // 可选：用于更方便的格式化输出（也能直接用 std::cout）

using namespace std::chrono_literals;

int main() {
    asio::io_context io;

    LinuxAppProcessManager manager{io};

    // 退出回调
    manager.registerExitCallback([](const ProcessInfo& info) {
        fmt::print("[CALLBACK] instanceId={}, pid={}, exitCode={}, state={}\n",
                   info.instanceId, info.pid, info.exitCode,
                   static_cast<int>(info.state));
    });

    // 描述待启动的 /bin/sleep 1
    LinuxAppInfo app;
    app.instanceId = "sleep_1";
    app.execPath   = "/bin/sleep";
    app.entrance   = {"1"};          // argv[1]
    app.autostart  = false;

    // 启动
    ProcessInfo pinfo = manager.start(app);
    fmt::print("[MAIN] started pid={}, state={}\n",
               pinfo.pid, static_cast<int>(pinfo.state));

    // run 事件循环（会在 sleep 退出后回调并结束）
    io.run();

    // 查询一次最终状态
    ProcessInfo finalInfo = manager.query("sleep_1");
    fmt::print("[MAIN] final state={}, exitCode={}\n",
               static_cast<int>(finalInfo.state), finalInfo.exitCode);
    return 0;
}
```

### 2 · CMakeLists.txt（顶层简化版）

```cmake
cmake_minimum_required(VERSION 3.20)
project(LinuxAppProcessManagerDemo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(Asio REQUIRED)          # 头文件版 asio；或自行 FetchContent
# find_package(fmt REQUIRED)         # 如果用 fmt，取消注释

add_executable(demo
    demo/main.cpp
    src/LinuxAppProcessManager.cpp   # 你拆分后的实现文件路径
    # src/Launcher.cpp
    # src/EnvManager.cpp
    # src/ProcessMonitor.cpp
)

target_include_directories(demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src    # 头文件路径
)

target_link_libraries(demo
    Threads::Threads
    Asio::asio                         # 或 asio 的目标名
    # fmt::fmt
)
```

> **如果你暂时把所有实现放在一个头文件（本 Canvas）中**，只需把 `src/LinuxAppProcessManager.cpp` 列表移除，`demo/main.cpp` `#include` 对应头即可。

### 3 · 编译 & 运行

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
./demo
```

**预期终端输出示例：**

```
[MAIN] started pid=12345, state=1        # 1 == Starting
[CALLBACK] instanceId=sleep_1, pid=12345, exitCode=0, state=3   # 3 == Exited
[MAIN] final state=3, exitCode=0
```

* `ProcessState` 枚举值示例：0=Unknown, 1=Starting, 2=Running, 3=Exited
    
* 你也可以在回调里打印 `info.startTime`/`endTime` 等字段，验证时间差≈1 s。
    
* 如需验证二阶段停止逻辑，把 `sleep 1` 改成 `sleep 100`，然后在 1 s 后调用 `manager.stop("sleep_1")`，观察先 `SIGTERM` 后 `SIGKILL`、定时器超时 3 s 后退出回调。
    

* * *

### 4 · 常见问题排查

| 现象 | 排查要点 |
| --- | --- |
| **编译缺头文件 <asio/io_context.hpp>** | 你用的是独立 Asio：`apt install libasio-dev` 或 FetchContent。不过 C++20 可直接 `std::asio`（GCC 14+/libstdc++ 未全部支持）。推荐 header-only Asio。 |
| **回调未触发** | 确认 `io.run()` 没有提前返回；须等 `sleep` 进程真正退出（或被 `SIGKILL`）。 |
| **状态一直显示 Starting** | `ProcessMonitor` 的 `waitpid` 没收到退出 → 检查：1) 是否 fork 成功；2) `waitpid` 是否被 `WNOHANG` correctly loop；3) 捕获信号集是否装载在同一 `io_context` 线程。 |

* * *

现在就可以把示例工程拉到本地验证；若有 _编译报错_ 或 _运行行为不符合预期_，把日志贴给我，我们继续调试！