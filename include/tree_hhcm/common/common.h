#ifndef _tree_hhcm_GLOBALS_H
#define _tree_hhcm_GLOBALS_H

#include <string>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/rclcpp.hpp>

#include <tree_hhcm/common/parsers.h>

namespace tree {

class Printer {

public:

    Printer(const BT::TreeNode& n): _name(n.name().c_str()) {};

    Printer(std::string n): _name(n) {};

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

    std::string parse_shell(std::string str) const;

    std::string check_output(std::string cmd) const;
    
private:

    Globals();

};
}

#endif // GLOBALS_H
