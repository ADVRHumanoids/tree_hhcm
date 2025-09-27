#include <bt_hhcm/common/common.h>

namespace tree {

Globals& Globals::instance()
{
    static Globals instance;
    return instance;
}

Globals::Globals()
{
    // Initialize default values
    tree_dt = 0.0;
    tree_dirname = "";
}

} // namespace tree
