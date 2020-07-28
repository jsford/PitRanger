#pragma once
#include <iostream>
#include <string>

namespace pr {

inline void log_info(const std::string& msg) {
    std::cout << msg << std::endl;
}
inline void log_warn(const std::string& msg) {
    std::cout << msg << std::endl;
}
inline void log_error(const std::string& msg) {
    std::cout << msg << std::endl;
}

} // namespace pr
