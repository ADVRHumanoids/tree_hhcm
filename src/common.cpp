#include <tree_hhcm/common/common.h>
#include <tree_hhcm/common/parsers.h>

namespace tree {

Globals& Globals::instance()
{
    static Globals instance;
    return instance;
}

std::string Globals::parse_shell(std::string str) const
{
    std::string result;
    std::string cmd = "/bin/bash -c 'cd " + tree_dirname + " && echo " + str + "'";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe) {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        int ret = pclose(pipe);
        if(ret != 0) {
            std::stringstream ss;
            ss << "Error: Command '" << cmd << "' exited with code " << ret << std::endl;
            throw std::runtime_error(ss.str());
        }
        // Remove trailing newline if present
        if (!result.empty() && result.back() == '\n') {
            result.pop_back();
        }
    }
    return result;
}

std::string Globals::check_output(std::string cmd) const
{
    std::string result;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (pipe) {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        int ret = pclose(pipe);
        if(ret != 0) {
            std::stringstream ss;
            ss << "Error: Command '" << cmd << "' exited with code " << ret << std::endl;
            throw std::runtime_error(ss.str());
        }
        // Remove trailing newline if present
        if (!result.empty() && result.back() == '\n') {
            result.pop_back();
        }
    }
    return result;
}

Globals::Globals()
{
    // Initialize default values
    tree_dt = 0.0;
    tree_dirname = "";
}

} // namespace tree
