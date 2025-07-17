#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <stdexcept>
#include <typeinfo>
#include <cstring>
#include <algorithm>
#include <cxxabi.h>
#include <cstdlib>
#include <filesystem>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace detail
{

    template <typename Target, typename Source, bool Same>
    class lexical_cast_t
    {
    public:
        static Target cast(const Source &arg)
        {
            Target ret;
            std::stringstream ss;
            if (!(ss << arg && ss >> ret && ss.eof()))
                throw std::bad_cast();

            return ret;
        }
    };

    template <typename Target, typename Source>
    class lexical_cast_t<Target, Source, true>
    {
    public:
        static Target cast(const Source &arg)
        {
            return arg;
        }
    };

    template <typename Source>
    class lexical_cast_t<std::string, Source, false>
    {
    public:
        static std::string cast(const Source &arg)
        {
            std::ostringstream ss;
            ss << arg;
            return ss.str();
        }
    };

    template <typename Target>
    class lexical_cast_t<Target, std::string, false>
    {
    public:
        static Target cast(const std::string &arg)
        {
            Target ret;
            std::istringstream ss(arg);
            if (!(ss >> ret && ss.eof()))
                throw std::bad_cast();
            return ret;
        }
    };

    template <typename T1, typename T2>
    struct is_same
    {
        static const bool value = false;
    };

    template <typename T>
    struct is_same<T, T>
    {
        static const bool value = true;
    };

    template <typename Target, typename Source>
    Target lexical_cast(const Source &arg)
    {
        return lexical_cast_t<Target, Source, detail::is_same<Target, Source>::value>::cast(arg);
    }

    static inline std::string demangle(const std::string &name)
    {
        int status = 0;
        char *p = abi::__cxa_demangle(name.c_str(), 0, 0, &status);
        std::string ret(p);
        free(p);
        return ret;
    }

    template <class T>
    std::string readable_typename()
    {
        return demangle(typeid(T).name());
    }

    template <class T>
    std::string default_value(T def)
    {
        return detail::lexical_cast<std::string>(def);
    }

    template <>
    inline std::string readable_typename<std::string>()
    {
        return "string";
    }

} // detail

//-----

class cmdline_error : public std::exception
{
public:
    cmdline_error(const std::string &msg) : msg(msg) {}

    ~cmdline_error() throw() {}

    const char *what() const throw() { return msg.c_str(); }

private:
    std::string msg;
};

template <class T>
struct default_reader
{
    T operator()(const std::string &str)
    {
        return detail::lexical_cast<T>(str);
    }
};

template <class T>
struct range_reader
{
    range_reader(const T &low, const T &high) : low(low), high(high) {}

    T operator()(const std::string &s) const
    {
        T ret = default_reader<T>()(s);
        if (!(ret >= low && ret <= high))
            throw cmdline_error("range_error");
        return ret;
    }

private:
    T low, high;
};

template <class T>
range_reader<T> range(const T &low, const T &high)
{
    return range_reader<T>(low, high);
}

template <class T>
struct oneof_reader
{
    T operator()(const std::string &s)
    {
        T ret = default_reader<T>()(s);
        if (std::find(alt.begin(), alt.end(), ret) == alt.end())
            throw cmdline_error("");
        return ret;
    }

    void add(const T &v) { alt.push_back(v); }

private:
    std::vector<T> alt;
};

template <class T>
oneof_reader<T> oneof(T a1)
{
    oneof_reader<T> ret;
    ret.add(a1);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3, T a4)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    ret.add(a4);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3, T a4, T a5)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    ret.add(a4);
    ret.add(a5);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3, T a4, T a5, T a6)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    ret.add(a4);
    ret.add(a5);
    ret.add(a6);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3, T a4, T a5, T a6, T a7)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    ret.add(a4);
    ret.add(a5);
    ret.add(a6);
    ret.add(a7);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3, T a4, T a5, T a6, T a7, T a8)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    ret.add(a4);
    ret.add(a5);
    ret.add(a6);
    ret.add(a7);
    ret.add(a8);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3, T a4, T a5, T a6, T a7, T a8, T a9)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    ret.add(a4);
    ret.add(a5);
    ret.add(a6);
    ret.add(a7);
    ret.add(a8);
    ret.add(a9);
    return ret;
}

template <class T>
oneof_reader<T> oneof(T a1, T a2, T a3, T a4, T a5, T a6, T a7, T a8, T a9, T a10)
{
    oneof_reader<T> ret;
    ret.add(a1);
    ret.add(a2);
    ret.add(a3);
    ret.add(a4);
    ret.add(a5);
    ret.add(a6);
    ret.add(a7);
    ret.add(a8);
    ret.add(a9);
    ret.add(a10);
    return ret;
}

//-----

class ArgParser
{
public:
    ArgParser()
    {
    }

    ~ArgParser()
    {
        for (std::map<std::string, option_base *>::iterator p = options.begin();
             p != options.end(); p++)
            delete p->second;
    }

    void add(const std::string &name,
             char short_name = 0,
             const std::string &desc = "")
    {
        if (options.count(name))
            throw cmdline_error("multiple definition: " + name);
        options[name] = new option_without_value(name, short_name, desc);
        ordered.push_back(options[name]);
    }

    template <class T>
    void add(const std::string &name,
             char short_name = 0,
             const std::string &desc = "",
             bool need = true,
             const T def = T())
    {
        add(name, short_name, desc, need, def, default_reader<T>());
    }

    template <class T, class F>
    void add(const std::string &name,
             char short_name = 0,
             const std::string &desc = "",
             bool need = true,
             const T def = T(),
             F reader = F())
    {
        if (options.count(name))
            throw cmdline_error("multiple definition: " + name);
        options[name] = new option_with_value_with_reader<T, F>(name, short_name, need, def, desc, reader);
        ordered.push_back(options[name]);
    }

    void footer(const std::string &f)
    {
        ftr = f;
    }

    void set_program_name(const std::string &name)
    {
        prog_name = name;
    }

    bool exist(const std::string &name) const
    {
        if (options.count(name) == 0)
            throw cmdline_error("there is no flag: --" + name);
        return options.find(name)->second->has_set();
    }

    template <class T>
    const T &get(const std::string &name) const
    {
        if (options.count(name) == 0)
            throw cmdline_error("there is no flag: --" + name);
        const option_with_value<T> *p = dynamic_cast<const option_with_value<T> *>(options.find(name)->second);
        if (p == NULL)
            throw cmdline_error("type mismatch flag '" + name + "'");
        return p->get();
    }

    const std::vector<std::string> &rest() const
    {
        return others;
    }

    bool parse(const std::string &arg)
    {
        std::vector<std::string> args;

        std::string buf;
        bool in_quote = false;
        for (std::string::size_type i = 0; i < arg.length(); i++)
        {
            if (arg[i] == '\"')
            {
                in_quote = !in_quote;
                continue;
            }

            if (arg[i] == ' ' && !in_quote)
            {
                args.push_back(buf);
                buf = "";
                continue;
            }

            if (arg[i] == '\\')
            {
                i++;
                if (i >= arg.length())
                {
                    errors.push_back("unexpected occurrence of '\\' at end of string");
                    return false;
                }
            }

            buf += arg[i];
        }

        if (in_quote)
        {
            errors.push_back("quote is not closed");
            return false;
        }

        if (buf.length() > 0)
            args.push_back(buf);

        for (size_t i = 0; i < args.size(); i++)
            std::cout << "\"" << args[i] << "\"" << std::endl;

        return parse(args);
    }

    bool parse(const std::vector<std::string> &args)
    {
        int argc = static_cast<int>(args.size());
        std::vector<const char *> argv(argc);

        for (int i = 0; i < argc; i++)
            argv[i] = args[i].c_str();

        return parse(argc, &argv[0]);
    }

    bool parse(int argc, const char *const argv[])
    {
        errors.clear();
        others.clear();

        if (argc < 1)
        {
            errors.push_back("argument number must be longer than 0");
            return false;
        }
        if (prog_name == "")
            prog_name = argv[0];

        std::map<char, std::string> lookup;
        for (std::map<std::string, option_base *>::iterator p = options.begin();
             p != options.end(); p++)
        {
            if (p->first.length() == 0)
                continue;
            char initial = p->second->short_name();
            if (initial)
            {
                if (lookup.count(initial) > 0)
                {
                    lookup[initial] = "";
                    errors.push_back(std::string("short option '") + initial + "' is ambiguous");
                    return false;
                }
                else
                    lookup[initial] = p->first;
            }
        }

        for (int i = 1; i < argc; i++)
        {
            if (strncmp(argv[i], "--", 2) == 0)
            {
                const char *p = strchr(argv[i] + 2, '=');
                if (p)
                {
                    std::string name(argv[i] + 2, p);
                    std::string val(p + 1);
                    set_option(name, val);
                }
                else
                {
                    std::string name(argv[i] + 2);
                    if (options.count(name) == 0)
                    {
                        errors.push_back("undefined option: --" + name);
                        continue;
                    }
                    if (options[name]->has_value())
                    {
                        if (i + 1 >= argc)
                        {
                            errors.push_back("option needs value: --" + name);
                            continue;
                        }
                        else
                        {
                            i++;
                            set_option(name, argv[i]);
                        }
                    }
                    else
                    {
                        set_option(name);
                    }
                }
            }
            else if (strncmp(argv[i], "-", 1) == 0)
            {
                if (!argv[i][1])
                    continue;
                char last = argv[i][1];
                for (int j = 2; argv[i][j]; j++)
                {
                    last = argv[i][j];
                    if (lookup.count(argv[i][j - 1]) == 0)
                    {
                        errors.push_back(std::string("undefined short option: -") + argv[i][j - 1]);
                        continue;
                    }
                    if (lookup[argv[i][j - 1]] == "")
                    {
                        errors.push_back(std::string("ambiguous short option: -") + argv[i][j - 1]);
                        continue;
                    }
                    set_option(lookup[argv[i][j - 1]]);
                }

                if (lookup.count(last) == 0)
                {
                    errors.push_back(std::string("undefined short option: -") + last);
                    continue;
                }
                if (lookup[last] == "")
                {
                    errors.push_back(std::string("ambiguous short option: -") + last);
                    continue;
                }

                if (i + 1 < argc && options[lookup[last]]->has_value())
                {
                    set_option(lookup[last], argv[i + 1]);
                    i++;
                }
                else
                {
                    set_option(lookup[last]);
                }
            }
            else
            {
                others.push_back(argv[i]);
            }
        }

        for (std::map<std::string, option_base *>::iterator p = options.begin();
             p != options.end(); p++)
            if (!p->second->valid())
                errors.push_back("need option: --" + std::string(p->first));

        return errors.size() == 0;
    }

    void parse_check(const std::string &arg)
    {
        if (!options.count("help"))
            add("help", '?', "print this message");
        check(0, parse(arg));
    }

    void parse_check(const std::vector<std::string> &args)
    {
        if (!options.count("help"))
            add("help", '?', "print this message");
        check(args.size(), parse(args));
    }

    void parse_check(int argc, char *argv[])
    {
        if (!options.count("help"))
            add("help", '?', "print this message");
        check(argc, parse(argc, argv));
    }

    std::string error() const
    {
        return errors.size() > 0 ? errors[0] : "";
    }

    std::string error_full() const
    {
        std::ostringstream oss;
        for (size_t i = 0; i < errors.size(); i++)
            oss << errors[i] << std::endl;
        return oss.str();
    }

    std::string usage() const
    {
        std::ostringstream oss;
        oss << "usage: " << prog_name << " ";
        for (size_t i = 0; i < ordered.size(); i++)
        {
            if (ordered[i]->must())
                oss << ordered[i]->short_description() << " ";
        }

        oss << "[options] ... " << ftr << std::endl;
        oss << "options:" << std::endl;

        size_t max_width = 0;
        for (size_t i = 0; i < ordered.size(); i++)
        {
            max_width = std::max(max_width, ordered[i]->name().length());
        }
        for (size_t i = 0; i < ordered.size(); i++)
        {
            if (ordered[i]->short_name())
            {
                oss << "  -" << ordered[i]->short_name() << ", ";
            }
            else
            {
                oss << "      ";
            }

            oss << "--" << ordered[i]->name();
            for (size_t j = ordered[i]->name().length(); j < max_width + 4; j++)
                oss << ' ';
            oss << ordered[i]->description() << std::endl;
        }
        return oss.str();
    }

private:
    void check(int argc, bool ok)
    {
        if ((argc == 1 && !ok) || exist("help"))
        {
            std::cerr << usage();
            exit(0);
        }

        if (!ok)
        {
            std::cerr << error() << std::endl
                      << usage();
            exit(1);
        }
    }

    void set_option(const std::string &name)
    {
        if (options.count(name) == 0)
        {
            errors.push_back("undefined option: --" + name);
            return;
        }
        if (!options[name]->set())
        {
            errors.push_back("option needs value: --" + name);
            return;
        }
    }

    void set_option(const std::string &name, const std::string &value)
    {
        if (options.count(name) == 0)
        {
            errors.push_back("undefined option: --" + name);
            return;
        }
        if (!options[name]->set(value))
        {
            errors.push_back("option value is invalid: --" + name + "=" + value);
            return;
        }
    }

    class option_base
    {
    public:
        virtual ~option_base() {}

        virtual bool has_value() const = 0;

        virtual bool set() = 0;

        virtual bool set(const std::string &value) = 0;

        virtual bool has_set() const = 0;

        virtual bool valid() const = 0;

        virtual bool must() const = 0;

        virtual const std::string &name() const = 0;

        virtual char short_name() const = 0;

        virtual const std::string &description() const = 0;

        virtual std::string short_description() const = 0;
    };

    class option_without_value : public option_base
    {
    public:
        option_without_value(const std::string &name,
                             char short_name,
                             const std::string &desc)
            : nam(name), snam(short_name), desc(desc), has(false)
        {
        }

        ~option_without_value() {}

        bool has_value() const { return false; }

        bool set()
        {
            has = true;
            return true;
        }

        bool set(const std::string &)
        {
            return false;
        }

        bool has_set() const
        {
            return has;
        }

        bool valid() const
        {
            return true;
        }

        bool must() const
        {
            return false;
        }

        const std::string &name() const
        {
            return nam;
        }

        char short_name() const
        {
            return snam;
        }

        const std::string &description() const
        {
            return desc;
        }

        std::string short_description() const
        {
            return "--" + nam;
        }

    private:
        std::string nam;
        char snam;
        std::string desc;
        bool has;
    };

    template <class T>
    class option_with_value : public option_base
    {
    public:
        option_with_value(const std::string &name,
                          char short_name,
                          bool need,
                          const T &def,
                          const std::string &desc)
            : nam(name), snam(short_name), need(need), has(false), def(def), actual(def)
        {
            this->desc = full_description(desc);
        }

        ~option_with_value() {}

        const T &get() const
        {
            return actual;
        }

        bool has_value() const { return true; }

        bool set()
        {
            return false;
        }

        bool set(const std::string &value)
        {
            try
            {
                actual = read(value);
                has = true;
            }
            catch (const std::exception &e)
            {
                return false;
            }
            return true;
        }

        bool has_set() const
        {
            return has;
        }

        bool valid() const
        {
            if (need && !has)
                return false;
            return true;
        }

        bool must() const
        {
            return need;
        }

        const std::string &name() const
        {
            return nam;
        }

        char short_name() const
        {
            return snam;
        }

        const std::string &description() const
        {
            return desc;
        }

        std::string short_description() const
        {
            return "--" + nam + "=" + detail::readable_typename<T>();
        }

    protected:
        std::string full_description(const std::string &desc)
        {
            return desc + " (" + detail::readable_typename<T>() +
                   (need ? "" : " [=" + detail::default_value<T>(def) + "]") + ")";
        }

        virtual T read(const std::string &s) = 0;

        std::string nam;
        char snam;
        bool need;
        std::string desc;

        bool has;
        T def;
        T actual;
    };

    template <class T, class F>
    class option_with_value_with_reader : public option_with_value<T>
    {
    public:
        option_with_value_with_reader(const std::string &name,
                                      char short_name,
                                      bool need,
                                      const T def,
                                      const std::string &desc,
                                      F reader)
            : option_with_value<T>(name, short_name, need, def, desc), reader(reader)
        {
        }

    private:
        T read(const std::string &s)
        {
            return reader(s);
        }

        F reader;
    };

    std::map<std::string, option_base *> options;
    std::vector<option_base *> ordered;
    std::string ftr;

    std::string prog_name;
    std::vector<std::string> others;

    std::vector<std::string> errors;
};

static std::string folder_path_add_file_name(const std::string &folder_path,
                                             const std::string &file_name)
{
    std::string folder_path_ = folder_path;
    if (folder_path_[folder_path_.length() - 1] != '/')
    {
        folder_path_.append("/");
    }
    return folder_path_ + file_name;
}

std::string get_folder_path_in_file_path(const std::string &file_path)
{
    size_t pos = file_path.find_last_of("\\/");
    return (std::string::npos == pos) ? "" : file_path.substr(0, pos);
}

std::string add_folder_if_not_exist(const std::string &folder_path)
{
    if (std::filesystem::exists(folder_path))
        return folder_path;
    std::filesystem::create_directories(folder_path);
    return folder_path;
}

static inline std::string _time_str_ms()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    auto timer = system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&timer);
    std::ostringstream oss;
    oss << std::put_time(&bt, "%d %H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

static inline std::string _time_str_s()
{
    time_t _t = time(0);
    char tmp[32] = {NULL};
    strftime(tmp, sizeof(tmp),
             "%Y-%m-%d_%H:%M:%S", localtime(&_t));
    std::string s(tmp);
    return s;
}

template <typename T>
inline void _print(T t)
{
    std::cout << t;
}

template <typename T, typename... Args>
inline void _print(T t, Args... args)
{
    std::cout << t << " ";
    _print(args...);
}

template <typename T, typename... Args>
inline void log(T t, Args... args)
{
    _print(_time_str_ms(), t, args..., "\033[0m\n");
}

template <typename T, typename... Args>
inline void log_info(T t, Args... args)
{
    _print(_time_str_ms(), t, args..., "\033[0m\n");
}

template <typename T, typename... Args>
inline void log_debug(T t, Args... args)
{
    std::cout << "\033[92m";
    _print(_time_str_ms(), t, args..., "\033[0m\n");
}

template <typename T, typename... Args>
inline void log_warning(T t, Args... args)
{
    std::cout << "\033[93m";
    _print(_time_str_ms(), t, args..., "\033[0m\n");
}

template <typename T, typename... Args>
inline void log_error(T t, Args... args)
{
    std::cout << "\033[91m";
    _print(_time_str_ms(), t, args..., "\033[0m\n");
}

static std::vector<std::string> str_split(const std::string &input,
                                          const char delimiter = ' ')
{
    std::vector<std::string> output;
    std::istringstream iss(input);
    std::string temp;
    while (std::getline(iss, temp, delimiter))
    {
        output.emplace_back(std::move(temp));
    }
    return output;
}

static std::vector<std::string> read_lines_from_file(const std::string &file_path)
{
    std::vector<std::string> out_lines;
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cout << "error 文件打开失败: " + file_path << std::endl;
        return out_lines;
    }
    std::string line;
    while (std::getline(file, line))
    {
        out_lines.push_back(line);
    }
    return out_lines;
}

static void write_lines_to_file_overwrite(const std::string &file_path,
                                          std::vector<std::string> &lines)
{
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        std::cout << "error 文件写入打开失败: " + file_path << std::endl;
        return;
    }
    for (auto &one_line : lines)
    {
        file << one_line << "\n";
    }
    file.close();
}

template <typename... Args>
inline std::string str_format(const char *format, Args... args)
{
    // demo: std::cout << format_string("adsfasf %d %s", 12342, "hello") << std::endl;
    //  %s 如对应的 std::string 需使用 .c_str() 转换成 C语言str
    constexpr size_t oldlen = BUFSIZ;
    char out_str[oldlen]; // 默认栈上的缓冲区
    size_t newlen = snprintf(&out_str[0], oldlen, format, args...);
    newlen++; // 算上终止符'\0'
    if (newlen > oldlen)
    {
        std::vector<char> newbuffer(newlen);
        snprintf(newbuffer.data(), newlen, format, args...);
        return std::string(newbuffer.data());
    }
    return out_str;
}

std::string to_string(uint64_t value)
{
    std::ostringstream os;
    os << value;
    return os.str();
}

static inline Eigen::Matrix4d qab_tab_to_Tab(const Eigen::Quaterniond &q,
                                             const Eigen::Vector3d &t)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(q);
    T.pretranslate(t);
    return T.matrix();
}

static inline Eigen::Matrix4d Rab_tab_to_Tab(const Eigen::Matrix3d &Rab,
                                             const Eigen::Vector3d &tab)
{
    Eigen::Matrix4d Tab;
    Tab.setIdentity();
    Tab.block(0, 0, 3, 3) = Rab;
    Tab.block(0, 3, 3, 1) = tab;
    return Tab;
}

inline std::string path_join(std::string t)
{
    return t;
}

template <typename... Args>
inline std::string path_join(std::string t, Args... args)
{
    if (t[t.length() - 1] != '/')
    {
        t.append("/");
    }
    return t + path_join(args...);
}

class ThreadPool
{
public:
    ThreadPool(size_t);

    template <class F, class... Args>
    auto enqueue(F &&f, Args &&...args)
        -> std::future<typename std::result_of<F(Args...)>::type>;

    ~ThreadPool();

private:
    // need to keep track of threads so we can join them
    std::vector<std::thread> workers;
    // the task queue
    std::queue<std::function<void()>> tasks;

    // synchronization
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

// the constructor just launches some amount of workers
inline ThreadPool::ThreadPool(size_t threads)
    : stop(false)
{
    for (size_t i = 0; i < threads; ++i)
        workers.emplace_back(
            [this]
            {
                for (;;)
                {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock,
                                             [this]
                                             { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty())
                            return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }

                    task();
                }
            });
}

// add new work item to the pool
template <class F, class... Args>
auto ThreadPool::enqueue(F &&f, Args &&...args)
    -> std::future<typename std::result_of<F(Args...)>::type>
{
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        // don't allow enqueueing after stopping the pool
        if (stop)
            throw std::runtime_error("enqueue on stopped ThreadPool");

        tasks.emplace([task]()
                      { (*task)(); });
    }
    condition.notify_one();
    return res;
}

// the destructor joins all threads
inline ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread &worker : workers)
        worker.join();
}

static inline bool file_exists(const std::string &file_path)
{
    std::ifstream f(file_path.c_str());
    return f.good();
}

void add_folder_in_file_path_if_not_exist(const std::string &file_path)
{
    size_t pos = file_path.find_last_of("\\/");
    std::string folder_path = (std::string::npos == pos) ? "" : file_path.substr(0, pos);
    if (std::filesystem::exists(folder_path))
        return;
    std::filesystem::create_directories(folder_path);
}

template <typename T>
void custom_voxel_filter(pcl::PointCloud<T> &input, float lx, float ly, float lz)
{
    auto output(new pcl::PointCloud<T>);
    if (input.size() == 0)
    {
        log_warning("[custom_voxel_filter] No input dataset given!");
        output->width = output->height = 0;
        output->clear();
        return;
    }

    output->height = 1;
    output->is_dense = true;
    Eigen::Vector4f min_p, max_p;
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
    Eigen::Vector4f leaf_size_;
    Eigen::Array4f inverse_leaf_size_;
    pcl::getMinMax3D(input, min_p, max_p);
    leaf_size_ << lx, ly, lz, 1;
    inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();

    unsigned int min_points_per_voxel_ = 1;
    std::vector<unsigned int> indices_;
    indices_.resize(input.points.size());
    for (size_t j = 0; j < input.points.size(); ++j)
    {
        indices_[j] = static_cast<unsigned int>(j);
    }

    min_b_[0] = static_cast<int>(std::floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int>(std::floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int>(std::floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int>(std::floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int>(std::floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int>(std::floor(max_p[2] * inverse_leaf_size_[2]));
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    struct cloud_point_index_idx
    {
        unsigned int idx;               // voxel index
        unsigned int cloud_point_index; // point index

        cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}
        bool operator<(const cloud_point_index_idx &p) const { return (idx < p.idx); }
    };
    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(indices_.size());

    for (const auto &index : indices_)
    {
        if (!input.is_dense)
            // Check if the point is invalid
            if (!pcl::isXYZFinite(input.points[index]))
                continue;

        int ijk0 = static_cast<int>(std::floor(input.points[index].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]));
        int ijk1 = static_cast<int>(std::floor(input.points[index].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]));
        int ijk2 = static_cast<int>(std::floor(input.points[index].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]));

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
        index_vector.emplace_back(static_cast<unsigned int>(idx), index);
    }
    std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

    unsigned int total = 0;
    unsigned int index = 0;
    std::vector<std::vector<int>> first_and_last_indices_vector;
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size())
    {
        std::vector<int> point_index_in_voxel;
        unsigned int i = index + 1;
        point_index_in_voxel.push_back(index_vector[index].cloud_point_index);
        while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
        {
            point_index_in_voxel.push_back(index_vector[i].cloud_point_index);
            ++i;
        }

        if (i - index >= min_points_per_voxel_)
        {
            ++total;
            first_and_last_indices_vector.push_back(point_index_in_voxel);
        }
        index = i;
    }
    output->resize(total);

    index = 0;
    for (const auto &cp : first_and_last_indices_vector)
    {
        output->points[index] = input.points[cp[0]];
        ++index;
    }
    output->width = output->size();
    input = *output;
}