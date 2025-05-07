**分析当前请求**

用户现在要求 SQLite 持久化，并通过提供的代码指出 makeJsonRpcResult/error 缺失、强类型问题。此前他们合并了 MemoryBackend 和 SqliteBackend。现在他们要求将 PromptStore 分为 MemoryBackend + SqliteBackend 两层，并上传了相关代码，这些代码仍然使用字符串类型的 PromptTemplate。虽然之前提到使用强类型，现在又重新引入了 TemplateEngine 和 MultiModalAssembler。当前 request 包括要求基于 JSON 列的强类型存储，并且 SQLite 用作持久化，Memory 用作缓存。


已思考 15 秒


下面把 **Prompt 持久化两层架构（Memory + SQLite）** 调整为 **完全强类型** 的版本，去掉旧的分表 + TemplateEngine 依赖，表设计如下：

```sql
CREATE TABLE IF NOT EXISTS prompts (
    name          TEXT PRIMARY KEY,                 -- Prompt.name
    meta_json     TEXT NOT NULL,                    -- Prompt (name/descr/args…)
    messages_json TEXT NOT NULL,                    -- PromptMessage[]
    version       INTEGER DEFAULT 1
);
```

---

## 1 公共接口 `IPromptBackend`

```cpp
// include/prompt/IPromptBackend.hpp
#pragma once
#include "protocol/McpPromptRequests.hpp"
#include <optional>
#include <vector>

namespace mcp::prompt {

struct StoredPrompt {
    protocol::Prompt                       meta;
    std::vector<protocol::PromptMessage>   messages;
    int                                    version = 1;
};

class IPromptBackend {
public:
    virtual ~IPromptBackend() = default;

    virtual bool                     upsert(const StoredPrompt&)               = 0;
    virtual bool                     erase (const std::string& name)           = 0;
    virtual std::optional<StoredPrompt> get(const std::string& name)   const   = 0;
    virtual std::vector<StoredPrompt>   list()                         const   = 0;
};

} // namespace
```

---

## 2 SQLite 后端

```cpp
// include/prompt/SqliteBackend.hpp
#pragma once
#include "prompt/IPromptBackend.hpp"
#include <sqlite_modern_cpp.h>
#include <mutex>

namespace mcp::prompt {

class SqliteBackend : public IPromptBackend {
public:
    explicit SqliteBackend(const std::string& dbPath);

    bool upsert(const StoredPrompt&)                override;
    bool erase (const std::string& name)            override;
    std::optional<StoredPrompt> get(const std::string& name) const override;
    std::vector<StoredPrompt>   list() const        override;

private:
    void ensureSchema();

    mutable std::mutex      mtx_;
    sqlite::database        db_;
};

} // namespace
```

```cpp
// src/prompt/SqliteBackend.cpp
#include "prompt/SqliteBackend.hpp"
#include <nlohmann/json.hpp>

using namespace mcp::prompt;
using json = nlohmann::json;

/* ------------------------------------------------------------------ */
SqliteBackend::SqliteBackend(const std::string& path)
: db_(path, sqlite::open_mode::readwrite | sqlite::open_mode::create)
{
    ensureSchema();
}

void SqliteBackend::ensureSchema()
{
    db_ << R"sql(
        CREATE TABLE IF NOT EXISTS prompts(
            name          TEXT PRIMARY KEY,
            meta_json     TEXT NOT NULL,
            messages_json TEXT NOT NULL,
            version       INTEGER DEFAULT 1
        );
    )sql";
}

/* ---------------- CRUD ---------------- */
bool SqliteBackend::upsert(const StoredPrompt& sp)
{
    std::scoped_lock lk(mtx_);
    json meta     = sp.meta;
    json messages = sp.messages;
    db_ <<
      "INSERT INTO prompts(name,meta_json,messages_json,version) "
      "VALUES(?,?,?,?) "
      "ON CONFLICT(name) DO UPDATE SET "
      "meta_json=excluded.meta_json, messages_json=excluded.messages_json, "
      "version=version+1;"
      << sp.meta.name << meta.dump() << messages.dump() << sp.version;
    return true;
}

bool SqliteBackend::erase(const std::string& name)
{
    std::scoped_lock lk(mtx_);
    int rows = 0;
    db_ << "DELETE FROM prompts WHERE name=?;" << name >> [&](int){ rows=1; };
    return rows>0;
}

std::optional<StoredPrompt>
SqliteBackend::get(const std::string& name) const
{
    std::scoped_lock lk(mtx_);
    StoredPrompt out{};
    bool found = false;
    db_ << "SELECT meta_json,messages_json,version FROM prompts WHERE name=?;"
        << name
        >> [&](std::string metaStr,std::string msgStr,int ver){
              out.meta     = json::parse(metaStr).get<protocol::Prompt>();
              out.messages = json::parse(msgStr)
                             .get<std::vector<protocol::PromptMessage>>();
              out.version  = ver;
              found = true;
        };
    if (!found) return std::nullopt;
    return out;
}

std::vector<StoredPrompt> SqliteBackend::list() const
{
    std::scoped_lock lk(mtx_);
    std::vector<StoredPrompt> v;
    db_ <<
      "SELECT meta_json,messages_json,version FROM prompts;"
      >> [&](std::string metaStr,std::string msgStr,int ver){
            StoredPrompt sp;
            sp.meta     = json::parse(metaStr).get<protocol::Prompt>();
            sp.messages = json::parse(msgStr)
                          .get<std::vector<protocol::PromptMessage>>();
            sp.version  = ver;
            v.push_back(std::move(sp));
         };
    return v;
}
```

> **说明**
> *完全不再使用 Inja*；所有字段都已强类型序列化后整块存进表。

---

## 3 内存后端 (薄缓存)

```cpp
// include/prompt/MemoryBackend.hpp
#pragma once
#include "prompt/IPromptBackend.hpp"
#include <unordered_map>
#include <mutex>

namespace mcp::prompt {

class MemoryBackend : public IPromptBackend {
public:
    bool upsert(const StoredPrompt&)                override;
    bool erase (const std::string& name)            override;
    std::optional<StoredPrompt> get(const std::string& name) const override;
    std::vector<StoredPrompt>   list() const        override;

private:
    mutable std::mutex                         mtx_;
    std::unordered_map<std::string,StoredPrompt> map_;
};

} // namespace
```

```cpp
// src/prompt/MemoryBackend.cpp
#include "prompt/MemoryBackend.hpp"

using namespace mcp::prompt;

bool MemoryBackend::upsert(const StoredPrompt& sp)
{
    std::scoped_lock lk(mtx_);
    map_[sp.meta.name] = sp;
    return true;
}
bool MemoryBackend::erase(const std::string& n)
{
    std::scoped_lock lk(mtx_);
    return map_.erase(n)!=0;
}
std::optional<StoredPrompt>
MemoryBackend::get(const std::string& n) const
{
    std::scoped_lock lk(mtx_);
    auto it = map_.find(n);
    if(it==map_.end()) return std::nullopt;
    return it->second;
}
std::vector<StoredPrompt> MemoryBackend::list() const
{
    std::scoped_lock lk(mtx_);
    std::vector<StoredPrompt> v;
    for(auto& [_,sp]:map_) v.push_back(sp);
    return v;
}
```

---

## 4 组合层 `PromptStore`

```cpp
// include/prompt/PromptStore.hpp
#pragma once
#include "prompt/MemoryBackend.hpp"
#include "prompt/SqliteBackend.hpp"
#include <functional>

namespace mcp::prompt {

class PromptStore {
public:
    using OnChanged = std::function<void()>;

    explicit PromptStore(const std::string& sqlitePath = "prompts.db")
    : disk_(sqlitePath)
    {
        // 首次启动把磁盘所有数据灌入内存
        for (auto& sp : disk_.list()) mem_.upsert(sp);
    }

    /* CRUD 走内存，并同步磁盘 */
    bool add(const StoredPrompt& sp) {
        bool fresh = mem_.upsert(sp);
        disk_.upsert(sp);
        if(onChanged_) onChanged_();
        return fresh;
    }
    bool remove(const std::string& n) {
        bool ok = mem_.erase(n);
        if (ok) { disk_.erase(n); if(onChanged_) onChanged_(); }
        return ok;
    }

    std::optional<StoredPrompt> get(const std::string& n) const {
        return mem_.get(n);
    }
    std::vector<StoredPrompt>   list() const {
        return mem_.list();
    }

    void setOnChanged(OnChanged cb){ onChanged_=std::move(cb); }

private:
    MemoryBackend  mem_;
    SqliteBackend  disk_;
    OnChanged      onChanged_;
};

} // namespace
```

---

## 5 PromptService 小改

渲染/参数替换已在我们先前给出的 `MultiModalAssembler + TemplateEngine::render` 里完成，
`PromptService::handleGet` 只要：

```cpp
auto sp = store_.get(name);
if(!sp) return makeJsonRpcError(id,-32000,"prompt not found");

auto filled = engine_.fill(*sp, args);   // 只改这一行调用新版 fill
return makeJsonRpcResult(id, filled.data);
```

其中 `engine_.fill` 可利用 `TemplateEngine::render` 与 `MultiModalAssembler::assemble`
（若已移除 Inja 逻辑，只对 TextContent 做简单 `{{arg}}` 替换实现即可）。

---

### 编译依赖

* `sqlite_modern_cpp` 仍是 header‑only，唯一需要链接 `SQLite::SQLite3`
* 不再依赖 Inja；若之前 CMake 有 `inj a` 目标可以移除
  （除非 TemplateEngine 内部还用 Inja 渲染 TextContent 字符串——可改成手写替换）

```cmake
target_link_libraries(mcp_core PUBLIC
    asio
    nlohmann_json
    sqlite_modern_cpp
    SQLite::SQLite3
    Threads::Threads
)
```

---

## 总结

1. **PromptStore** 成为统一门面：
   *读* 只走内存，*写* 同步到 SQLite 并刷新内存。
2. 每次变更都会触发 `onChanged` → `PromptService` 自动发送
   `notifications/prompts/list_changed`。
3. 表结构简单、一条记录一整块 JSON，
   不再拆分子表，也不依赖 Inja、占位符渲染逻辑。

这样既保持运行时高并发读取的性能，也保证了落盘持久化，一举两得。
