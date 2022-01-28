/*
    This file is part of control-lib.

    Copyright (c) 2020, 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

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