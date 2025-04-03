#ifndef DATATYPES_H
#define DATATYPES_H

// Enums with the same name must be placed into their own namespace
namespace ReefAlignment {
    enum ReefAlignment {left, center, right};
}

namespace Auton {
    enum Auton {none, center, center_left, center_right, left, right, taxi};
}

namespace AutonLevel {
    enum AutonLevel {none, level_1, level_2, level_3, level_4};
}

#endif