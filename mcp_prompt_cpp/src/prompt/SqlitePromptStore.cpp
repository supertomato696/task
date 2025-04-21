#include "prompt/SqlitePromptStore.hpp"
#include <inja/inja.hpp>    // 仅用于 json → string dump (messages)

using namespace mcp;

SqlitePromptStore::SqlitePromptStore(const std::string& path)
: db_(path)
{
    ensureSchema();
}

void SqlitePromptStore::ensureSchema(){
    db_ << "PRAGMA journal_mode=WAL;";
    db_ << "PRAGMA foreign_keys=ON;";
    db_ << R"sql(
        CREATE TABLE IF NOT EXISTS prompts(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT UNIQUE NOT NULL,
            description TEXT);
    )sql";
    db_ << R"sql(
        CREATE TABLE IF NOT EXISTS prompt_arguments(
            prompt_id INTEGER,
            arg_name TEXT,
            PRIMARY KEY(prompt_id,arg_name),
            FOREIGN KEY(prompt_id) REFERENCES prompts(id) ON DELETE CASCADE);
    )sql";
    db_ << R"sql(
        CREATE TABLE IF NOT EXISTS prompt_messages(
            prompt_id INTEGER,
            seq INTEGER,
            role TEXT,
            content_tpl TEXT,
            PRIMARY KEY(prompt_id,seq),
            FOREIGN KEY(prompt_id) REFERENCES prompts(id) ON DELETE CASCADE);
    )sql";
}

/* ---------------- add ---------------- */
void SqlitePromptStore::add(const PromptTemplate& t){
    std::scoped_lock lk(mtx_);
    db_ << "BEGIN;";
    try{
        db_ << "INSERT OR REPLACE INTO prompts(name,description) VALUES(?,?);"
            << t.name << t.description;
        long long pid = db_.last_insert_rowid();

        db_ << "DELETE FROM prompt_arguments WHERE prompt_id=?;" << pid;
        for(auto& a: t.arguments)
            db_ << "INSERT INTO prompt_arguments(prompt_id,arg_name) VALUES(?,?);"
                << pid << a;

        db_ << "DELETE FROM prompt_messages WHERE prompt_id=?;" << pid;
        int seq = 0;
        for(auto& m: t.messages){
            db_ <<
              "INSERT INTO prompt_messages(prompt_id,seq,role,content_tpl) VALUES(?,?,?,?);"
              << pid << seq++ << m.at("role").get<std::string>()
              << m.at("content").get<std::string>();
        }
        db_ << "COMMIT;";
        cache_.erase(t.name);          // 让下次读取时刷新
    }catch(...){ db_<<"ROLLBACK;"; throw; }
}

/* ---------------- find/all ---------------- */
void SqlitePromptStore::fillCache_locked() const{
    if(!cache_.empty()) return;

    db_ <<
      "SELECT id,name,description FROM prompts;"
      >> [&](long long id,std::string name,std::string desc){
            PromptTemplate pt;
            pt.name = std::move(name);
            pt.description = std::move(desc);

            // arguments
            db_ << "SELECT arg_name FROM prompt_arguments WHERE prompt_id=?;"
                << id >> [&](std::string a){ pt.arguments.push_back(a); };

            // messages
            db_ << "SELECT role,content_tpl FROM prompt_messages "
                   "WHERE prompt_id=? ORDER BY seq;" << id
                >> [&](std::string role,std::string tpl){
                    pt.messages.push_back({ {"role",role},{"content",tpl} });
                };

            cache_.emplace(pt.name, std::move(pt));
         };
}

const PromptTemplate* SqlitePromptStore::find(const std::string& n) const{
    std::scoped_lock lk(mtx_);
    fillCache_locked();
    auto it = cache_.find(n);
    return it==cache_.end()?nullptr:&it->second;
}

std::vector<PromptTemplate> SqlitePromptStore::all() const{
    std::scoped_lock lk(mtx_);
    fillCache_locked();
    std::vector<PromptTemplate> v;
    for(auto& [_,p]:cache_) v.push_back(p);
    return v;
}