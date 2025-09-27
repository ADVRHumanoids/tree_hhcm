#ifndef _BT_HHCM_GLOBALS_H
#define _BT_HHCM_GLOBALS_H

#include <string>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/rclcpp.hpp>

namespace tree {

class Printer {

public:

    Printer(const BT::TreeNode& n): _name(n.name().c_str()) {};

    void setHeader(std::string hdr)
    {
        _name = hdr;
    }

    std::ostream& cout() const
    {
        std::cout << "[" << _name << "] ";
        return std::cout;
    }

    std::ostream& cerr() const
    {
        std::cout << "[" << _name << "] ";
        return std::cout;
    }

private:

    std::string _name;

};

class Globals {

public:

    static Globals& instance();

    double tree_dt = 0.0;
    std::string tree_dirname;

private:

    Globals();

};

}

#endif // GLOBALS_H
