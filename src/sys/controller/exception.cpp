/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "exception.h"

using namespace covise::controller;

Exception::Exception(const std::string &msg)
:m_msg(msg)
{
}

const char* Exception::what() const _GLIBCXX_USE_NOEXCEPT
{
    return m_msg.c_str();
}
