#ifndef CONTROLLIB_TOOLS_MSG_HPP
#define CONTROLLIB_TOOLS_MSG_HPP

#include <cstdlib>
#include <exception>
#include <iostream>

#ifndef CONTROLLIB_SHOW_WARNINGS
#define CONTROLLIB_SHOW_WARNINGS true
#endif

namespace control_lib {
    namespace tools {
        class Assertion : public std::exception {
        public:
            Assertion(const std::string& msg = "") : _msg(_make_message(msg)) {}

            const char* what() const throw()
            {
                return _msg.c_str();
            }

        private:
            std::string _msg;

            std::string _make_message(const std::string& msg) const
            {
                std::string message = "control-lib assertion failed";
                if (msg != "")
                    message += ": '" + msg + "'";
                return message;
            }
        };
    } // namespace tools
} // namespace control_lib

#define CONTROLLIB_WARNING(condition, message)                                    \
    if (CONTROLLIB_SHOW_WARNINGS && (condition)) {                                \
        std::cerr << "[control-lib WARNING]: \"" << message << "\"" << std::endl; \
    }

#define CONTROLLIB_ASSERT(condition, message, returnValue)                         \
    do {                                                                           \
        if (!(condition)) {                                                        \
            std::cerr << "control-lib assertion failed: " << message << std::endl; \
            return returnValue;                                                    \
        }                                                                          \
    } while (false)

#define CONTROLLIB_EXCEPTION_ASSERT(condition, message) \
    do {                                                \
        if (!(condition)) {                             \
            throw CONTROLLIB::Assertion(message);       \
        }                                               \
    } while (false)

#define CONTROLLIB_INTERNAL_ASSERT(condition)                                                                                 \
    do {                                                                                                                      \
        if (!(condition)) {                                                                                                   \
            std::cerr << "Assertion '" << #condition << "' failed in '" << __FILE__ << "' on line " << __LINE__ << std::endl; \
            std::abort();                                                                                                     \
        }                                                                                                                     \
    } while (false)

#define CONTROLLIB_EXCEPTION_INTERNAL_ASSERT(condition) \
    do {                                                \
        if (!(condition)) {                             \
            throw CONTROLLIB::Assertion(#condition);    \
        }                                               \
    } while (false)

#endif // CONTROLLIB_TOOLS_MSG_HPP