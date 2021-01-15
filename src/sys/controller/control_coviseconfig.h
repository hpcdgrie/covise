/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef CTRL_COVISECONFIG_H_
#define CTRL_COVISECONFIG_H_

#include <covise/covise.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#endif
#include <config/CoviseConfig.h>

#define COVISE_MAXHOSTCONFIG 2000

#define COVISE_POSIX 5
#define COVISE_PROXIE 4
#define COVISE_NOSHM 3
#define COVISE_MMAP 2
#define COVISE_SHM 1

namespace covise
{

enum class ExecType
{
    Local, VRB, Manual, Script
};

template<typename T>
T &operator<<(T &stream, ExecType execType){
    stream << static_cast<int>(execType);
    return stream;
}

class ControlConfig
{
private:
    struct HostInfo
    {
        int shminfo;
        ExecType exectype;
        int timeout;
        char *display;
    };
    typedef std::map<std::string, HostInfo> HostMap;
    HostMap hostMap;

    HostMap::iterator getOrCreateHostInfo(const std::string &);
    void addhostinfo(const std::string &name, int s_mode, ExecType e_mode, int t);
    void addhostinfo_from_config(const std::string &name);

public:
    ControlConfig()
    {
    }
    ~ControlConfig()
    {
    }

    int getshminfo(const std::string &n);
    ExecType getexectype(const std::string &n);
	int gettimeout(const std::string &n);
	int gettimeout(const covise::Host &h);

	char *getDisplayIP(const covise::Host &h);

    int set_shminfo(const std::string &n, const char *shm_mode);
    int set_timeout(const std::string &n, const char *t);
    ExecType set_exectype(const std::string &n, const char *e);
    char *set_display(const std::string &n, const char *e);

};
}
#endif
