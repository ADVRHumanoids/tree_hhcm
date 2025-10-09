#include <tree_hhcm/common/parsers.h>
#include <format>
#include <optional>
#include <set>

template<typename Numeric>
struct is_type
{
    static bool v(std::string s)
    {
        if(s.empty())
        {
            return false;
        }

        Numeric n;

        return((std::istringstream(s) >> n >> std::ws).eof());
    }
};

template<>
struct is_type<bool>
{
    static bool v(std::string s)
    {
        return s == "true" || s == "false";
    }
};

template<typename Numeric>
struct is_type<std::vector<Numeric>>
{
    static bool v(std::string s)
    {
        // is it a vector?
        if(s.length() < 3 || s[0] != '[' || s.back() != ']')
        {
            return false;
        }

        // remove brackets
        s = s.substr(1, s.length() - 2);

        // split
        auto tokens = BT::splitString(s, ',');

        // are all elems convertible to a numeric?
        for(auto t : tokens)
        {
            if(!is_type<Numeric>::v(t))
            {
                return false;
            }
        }

        return true;

    }
};

std::string tree::detect_yaml_type(YAML::Node yparam)
{
    if(yparam.IsScalar())
    {
        auto pstr = yparam.as<std::string>();

        if(is_type<bool>::v(pstr))
        {
            return "bool";
        }

        if(is_type<int>::v(pstr))
        {
            return "int";
        }

        if(is_type<double>::v(pstr))
        {
            return "double";
        }

        if(pstr.length() > 7 && pstr.substr(0, 7) == "file://")
        {
            return "file";
        }

        return "string";
    }
    else if(yparam.IsSequence())
    {
        std::set<std::string> types;

        YAML::Emitter em;
        em << yparam;

        for(auto s : yparam)
        {
            auto t = detect_yaml_type(s);

            types.insert(t);
        }

        if(types.count("string"))
        {
            return "vector<string>";
        }
        else if(types.size() > 1)
        {
            std::cerr << std::format("when detecting type for sequence '{}': mismatching types \n",
                     em.c_str());

            return "";
        }
        else if(types.size() == 0)
        {
            std::cerr << std::format("when detecting type for sequence '{}': empty sequence \n",
                em.c_str());

            return "";
        }

        return std::format("vector<{}>", *types.begin());
    }
    else if(yparam.IsMap())
    {
        YAML::Emitter em;
        em.SetMapFormat(YAML::Flow);
        em << yparam;

        std::optional<std::string> key_type, value_type;

        for(auto item : yparam)
        {
            auto k = detect_yaml_type(item.first);
            auto v = detect_yaml_type(item.second);

            if(k == "string")
            {
                key_type = k;
            }

            if(v == "string")
            {
                value_type = v;
            }

            if(key_type.has_value() && *key_type != k)
            {
                std::cerr << std::format("when detecting key type for map '{}': mismatching types \n",
                         em.c_str());

                return "";
            }

            if(value_type.has_value() && *value_type != v)
            {
                std::cerr << std::format("when detecting value type for map '{}': mismatching types \n",
                         em.c_str());

                return "";
            }

            key_type = k;
            value_type = v;
        }

        return std::format("map<{},{}>", *key_type, *value_type);
    }

    return "";
}