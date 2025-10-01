#ifndef TREE_HHCM_COMMON_PARSERS_H
#define TREE_HHCM_COMMON_PARSERS_H

#include <behaviortree_cpp/basic_types.h>
#include <xbot2_interface/xbotinterface2.h>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>

// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline std::map<std::string, double> convertFromString(StringView str)
    {
        // we expect a yaml map like "key1: val1, key2: val2"
        // so we add the curly braces to make it a valid map
        YAML::Node node = YAML::Load("{" + std::string(str) + "}");
        if (!node.IsMap())
        {
            throw RuntimeError("invalid input");
        }

        return node.as<std::map<std::string, double>>();
    }

    template <> inline std::unordered_map<std::string, double> convertFromString(StringView str)
    {
        // we expect a yaml map like "key1: val1, key2: val2"
        // so we add the curly braces to make it a valid map
        YAML::Node node = YAML::Load("{" + std::string(str) + "}");
        if (!node.IsMap())
        {
            throw RuntimeError("invalid input");
        }

        return node.as<std::unordered_map<std::string, double>>();
    }

    template <> inline std::vector<double> convertFromString(StringView str)
    {
        // we expect a yaml sequence like "[val1, val2, val3]"
        YAML::Node node = YAML::Load(std::string(str));
        if (!node.IsSequence())
        {
            throw RuntimeError("invalid input");
        }

        return node.as<std::vector<double>>();
    }

    template <> inline std::vector<std::string> convertFromString(StringView str)
    {
        // we expect a yaml sequence like "[val1, val2, val3]"
        YAML::Node node = YAML::Load(std::string(str));
        if (!node.IsSequence())
        {
            throw RuntimeError("invalid input");
        }

        return node.as<std::vector<std::string>>();
    }

    template <> inline Eigen::VectorXd convertFromString(StringView str)
    {
        // we expect a yaml sequence like "[val1, val2, val3]"
        YAML::Node node = YAML::Load(std::string(str));
        if (!node.IsSequence())
        {
            throw RuntimeError("invalid input");
        }

        Eigen::VectorXd output(node.size());
        for (std::size_t i = 0; i < node.size(); ++i)
        {
            output[i] = node[i].as<double>();
        }

        return output;
    }

    template <> inline Eigen::Vector6d convertFromString(StringView str)
    {
        YAML::Node node = YAML::Load(std::string(str));
        if (!node.IsSequence() || node.size() != 6)
        {
            throw RuntimeError("invalid input");
        }

        Eigen::Vector6d output;
        for (std::size_t i = 0; i < node.size(); ++i)
        {
            output[i] = node[i].as<double>();
        }

        return output;
    }

    template <> inline Eigen::Affine3d convertFromString(StringView str)
    {
        // We expect either a 3d or 7d array in yaml format
        YAML::Node node = YAML::Load(std::string(str));
        if (!node.IsSequence() || (node.size() != 3 && node.size() != 7))
        {
            throw RuntimeError("invalid input");
        }

        Eigen::Affine3d output = Eigen::Affine3d::Identity();
        output.translation().x() = node[0].as<double>();
        output.translation().y() = node[1].as<double>();
        output.translation().z() = node[2].as<double>();

        if(node.size() == 7)
        {
            output.linear() = Eigen::Quaterniond(node[6].as<double>(),
                                                 node[3].as<double>(),
                                                 node[4].as<double>(),
                                                 node[5].as<double>()).normalized().toRotationMatrix();
        }

        return output;
    }
} // end namespace BT


#endif