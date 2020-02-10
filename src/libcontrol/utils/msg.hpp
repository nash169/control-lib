#ifndef LIBCONTROL_UTILS_MSG_HPP
#define LIBCONTROL_UTILS_MSG_HPP

#include <cstdlib>
#include <exception>
#include <iostream>

#ifndef LIBCONTROL_SHOW_WARNINGS
#define LIBCONTROL_SHOW_WARNINGS true
#endif

namespace libcontrol {
    namespace utils {
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
                std::string message = "libcontrol assertion failed";
                if (msg != "")
                    message += ": '" + msg + "'";
                return message;
            }
        };
    } // namespace utils
} // namespace libcontrol

#define LIBCONTROL_WARNING(condition, message)                                   \
    if (LIBCONTROL_SHOW_WARNINGS && (condition)) {                               \
        std::cerr << "[libcontrol WARNING]: \"" << message << "\"" << std::endl; \
    }

#define LIBCONTROL_ASSERT(condition, message, returnValue)                        \
    do {                                                                          \
        if (!(condition)) {                                                       \
            std::cerr << "libcontrol assertion failed: " << message << std::endl; \
            return returnValue;                                                   \
        }                                                                         \
    } while (false)

#define LIBCONTROL_EXCEPTION_ASSERT(condition, message) \
    do {                                                \
        if (!(condition)) {                             \
            throw LIBCONTROL::Assertion(message);       \
        }                                               \
    } while (false)

#define LIBCONTROL_INTERNAL_ASSERT(condition)                                                                                 \
    do {                                                                                                                      \
        if (!(condition)) {                                                                                                   \
            std::cerr << "Assertion '" << #condition << "' failed in '" << __FILE__ << "' on line " << __LINE__ << std::endl; \
            std::abort();                                                                                                     \
        }                                                                                                                     \
    } while (false)

#define LIBCONTROL_EXCEPTION_INTERNAL_ASSERT(condition) \
    do {                                                \
        if (!(condition)) {                             \
            throw LIBCONTROL::Assertion(#condition);    \
        }                                               \
    } while (false)

#endif // LIBCONTROL_UTILS_MSG_HPP