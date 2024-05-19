//
// Created by Johan <job8197@thi.de> on 18.05.2024.
//

#ifndef THI_DRONE_WS_ERRORS_HPP
#define THI_DRONE_WS_ERRORS_HPP

#include <stdexcept>
#include <string>

namespace fcc_bridge {

class invalid_state_error : public std::runtime_error {
   public:
    invalid_state_error(const std::string &what) : std::runtime_error(what) {}
};

class unknown_enum_value_error : public std::runtime_error {
   public:
    unknown_enum_value_error(const std::string &what)
        : std::runtime_error(what) {}
};

}  // namespace fcc_bridge

#endif  // THI_DRONE_WS_ERRORS_HPP
