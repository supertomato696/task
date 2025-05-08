#include "prompt/SqliteBackend.hpp"
#include <nlohmann/json.hpp>

using namespace mcp::prompt;
using json = nlohmann::json;

/* ------------------------------------------------------------------ */
//SqliteBackend::SqliteBackend(const std::string& path)
//: db_(path, sqlite::open_mode::readwrite | sqlite::open_mode::create)
//{
//    ensureSchema();
//}

//SqliteBackend::SqliteBackend(const std::string& path)
//: db_(path, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE)
//{
//    ensureSchema();
//}


SqliteBackend::SqliteBackend(const std::string& path)
: db_(path)
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