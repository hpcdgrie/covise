/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef CONTROLL_EXCECPTION_H
#define CONTROLL_EXCECPTION_H
#include <exception>
#include <string>
namespace covise{
namespace controller
{
struct Exception : std::exception
{
    Exception(const std::string &msg);
    const char* what() const _GLIBCXX_TXN_SAFE_DYN _GLIBCXX_USE_NOEXCEPT override;

private:
    const std::string &m_msg;
};

} // namespace controller
} // namespace covise

#endif // !CONTROLL_EXECPTION_H