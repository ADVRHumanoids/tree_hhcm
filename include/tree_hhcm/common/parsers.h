#ifndef TREE_HHCM_COMMON_PARSERS_H
#define TREE_HHCM_COMMON_PARSERS_H

#include <behaviortree_cpp/basic_types.h>
#include <xbot2_interface/xbotinterface2.h>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>

namespace tree 
{
    std::string detect_yaml_type(YAML::Node node);
}

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

namespace YAML
{

template <>
struct convert<Eigen::Affine3d>
{
    static Node encode(const Eigen::Affine3d& d)
    {
        Node n;
        n.push_back(d.translation().x());
        n.push_back(d.translation().y());
        n.push_back(d.translation().z());
        Eigen::Quaterniond q(d.linear());
        n.push_back(q.x());
        n.push_back(q.y());
        n.push_back(q.z());
        n.push_back(q.w());
        return n;
    }

    static bool decode(const Node& node, Eigen::Affine3d& d)
    {
        d.setIdentity();
        
        // map format
        if(node.IsMap())
        {
            if(auto pos = node["pos"])
            {
                if(!pos.IsSequence() || pos.size() != 3)
                {
                    return false;
                }
                d.translation().x() = pos[0].as<double>();
                d.translation().y() = pos[1].as<double>();
                d.translation().z() = pos[2].as<double>();
            }
            else
            {
                return false;
            }

            if(auto rot = node["rot"])
            {
                if(!rot.IsSequence() || rot.size() != 4)
                {
                    return false;
                }
                d.linear() = Eigen::Quaterniond(rot[3].as<double>(),
                                                rot[0].as<double>(),
                                                rot[1].as<double>(),
                                                rot[2].as<double>()).normalized().toRotationMatrix();
            }

            return true;
        }

        // it must be an array!
        if (!node.IsSequence() || (node.size() != 3 && node.size() != 7))
        {
            return false;
        }

        d = Eigen::Affine3d::Identity();
        d.translation().x() = node[0].as<double>();
        d.translation().y() = node[1].as<double>();
        d.translation().z() = node[2].as<double>();

        if(node.size() == 7)
        {
            d.linear() = Eigen::Quaterniond(node[6].as<double>(),
                                            node[3].as<double>(),
                                            node[4].as<double>(),
                                            node[5].as<double>()).normalized().toRotationMatrix();
        }

        return true;
    }

};


}


#endif