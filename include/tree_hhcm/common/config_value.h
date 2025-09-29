#ifndef TREE_HHCM_COMMON_CONFIG_VALUE_H
#define TREE_HHCM_COMMON_CONFIG_VALUE_H

#include <behaviortree_cpp/blackboard.h>
#include <tree_hhcm/common/common.h>

namespace tree
{

class ConfigValueBase
{
public:
    static BT::Blackboard::Ptr root_tree_blackboard;

protected:
};

template <typename T>
class ConfigValue : public ConfigValueBase
{
public:
    ConfigValue() {}

    const T &value() const { return _value; }
    void setValue(const T &value) { _value = value; }

    void setFromString(std::string str)
    {
        try
        {
            _setFromString(str);
        }
        catch (const std::exception &e)
        {
            std::cerr << "ConfigValue: error parsing string '" << str << "': " << e.what() << std::endl;
        }
    }

    void _setFromString(std::string str)
    {
        const std::string cache_key = str;

        if (str.size() > 2 && str.substr(0, 2) == "!!")
        {
            try
            {
                setValue(_cache.at(cache_key));
                return;
            }
            catch(const std::exception& e)
            {
                std::cout << "ConfigValue: cache miss for key '" << cache_key << "'" << std::endl;
            }

            // e.g. "!!{main_config}.poses.home"
            // remove leading "!!"
            str = str.substr(2);

            // split by '.'
            auto tokens = BT::splitString(str, '.');
            std::vector<std::string> yaml_keys(tokens.begin() + 1, tokens.end());

            // try to load yaml file from cache

            // find all entries of the form {key} in first token
            str = std::string(tokens[0]);

            if (str[0] == '{' && str[str.size() - 1] == '}')
            {
                // extract key
                std::string key = str.substr(1, str.size() - 2);

                // get value from blackboard
                auto bb = tree::ConfigValueBase::root_tree_blackboard;

                if (!bb)
                {
                    throw std::runtime_error("ConfigValue: root_tree_blackboard is not set");
                }

                str = bb->get<std::string>(key);
            }

            // parse with shell
            auto str_parsed = tree::Globals::instance().parse_shell(str);

            // load yaml
            auto cfg = YAML::LoadFile(str_parsed);

            for (auto k : yaml_keys)
            {
                // std::cout << "navigating to token: " << k << std::endl;
                cfg = cfg[k];
                // std::cout << cfg << std::endl;
            }

            // convert to T
            T value = cfg.as<T>();
            setValue(value);
            _cache[cache_key] = value;
            return;
        }

        setValue(BT::convertFromString<T>(str));
    }

private:
    T _value;
    static std::map<std::string,T> _cache;
};

template <typename T>
std::map<std::string, T> ConfigValue<T>::_cache;

}

#define TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(T)                       \
    namespace BT                                                      \
    {                                                                 \
        template <>                                                   \
        inline tree::ConfigValue<T> convertFromString(StringView str) \
        {                                                             \
            tree::ConfigValue<T> cfg;                                 \
            cfg.setFromString(std::string(str));                      \
            return cfg;                                               \
        }                                                             \
    }

TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(double)
TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(int)
TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(bool)
TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(std::string)
TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(std::vector<double>)
TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(std::vector<int>)
TREE_HHCM_REGISTER_CONFIG_VALUE_TYPE(std::vector<std::string>)

#endif