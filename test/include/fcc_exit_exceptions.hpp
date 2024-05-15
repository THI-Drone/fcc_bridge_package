//
// Created by Johan <job8197@thi.de> on 13.05.2024.
//

#ifndef THI_DRONE_WS_FCC_EXIT_EXCEPTIONS_HPP
#define THI_DRONE_WS_FCC_EXIT_EXCEPTIONS_HPP

#include <stdexcept>

namespace fcc_bridge::test {

/**
 * @brief Error to be thrown instead of exiting the process by
 * exit_process_on_error if FCCBridgeNode::internal_state is set to ERROR
 */
class normal_fcc_exit : std::runtime_error {
   public:
    /**
     * @brief Constructs an exception to signal exit_process_on_error was called
     * with FCCBridgeNode::internal_state set to ERROR
     *
     * @param func_name The name of the function were fcc_exit was thrown
     */
    explicit normal_fcc_exit(const std::string &func_name) noexcept
        : std::runtime_error(func_name + " was called") {}
};

/**
 * @brief Error to be thrown instead of exiting the process by
 * exit_process_on_error if FCCBridgeNode::internal_state is not set to ERROR
 */
class abnormal_fcc_exit : std::runtime_error {
   public:
    /**
     * @brief Constructs an exception to signal exit_process_on_error was called
     * without FCCBridgeNode::internal_state set to ERROR
     *
     * @param func_name The name of the function were fcc_exit was thrown
     */
    explicit abnormal_fcc_exit(const std::string &func_name) noexcept
        : std::runtime_error(func_name + " was called") {}
};

}  // namespace fcc_bridge::test

#endif  // THI_DRONE_WS_FCC_EXIT_EXCEPTIONS_HPP
