#pragma once
#include "glm/glm.hpp"

namespace Lobo {

inline bool inRect(glm::vec4 rect, float x, float y) {
    if (x < std::max(rect[0], rect[2]) && x > std::min(rect[0], rect[2])) {
        if (y < std::max(rect[1], rect[3]) && y > std::min(rect[1], rect[3])) {
            return true;
        }
    }
    return false;
}

}  // namespace Lobo
