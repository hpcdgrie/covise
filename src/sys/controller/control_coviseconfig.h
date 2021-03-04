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



namespace covise
{
constexpr int DEFAULT_TIMEOUT = 30;


enum class ShmMode {
    Default = 1, //COVISE_SHM 
    MMap, //COVISE_MMAP,
    NoShm, //COVISE_NOSHM, 
    Proxie, //COVISE_PROXIE, 
    Posix //COVISE_POSIX, 
 };

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
        ShmMode shmMode = ShmMode::Default;
        ExecType exectype = ExecType::VRB;
        int timeout = DEFAULT_TIMEOUT;
        char *display = nullptr;
    };
    typedef std::map<std::string, HostInfo> HostMap;
    HostMap hostMap;

    HostMap::iterator getOrCreateHostInfo(const std::string &);
    void addhostinfo(const HostMap::iterator &host, ShmMode s_mode, ExecType e_mode, int t);
    void addhostinfo_from_config(const HostMap::iterator &host);

public:
    ControlConfig()
    {
    }
    ~ControlConfig()
    {
    }

    ShmMode getshmMode(const std::string &hostName);
    ExecType getexectype(const std::string &hostName);
	int gettimeout(const std::string &hostName);
	int gettimeout(const covise::Host &h);

	const char *getDisplayIP(const covise::Host &host);
	const char *getDisplayIP(const char* hostName);

    ShmMode set_shminfo(const std::string &n, const char *shm_mode);
    int set_timeout(const std::string &n, const char *t);
    ExecType set_exectype(const std::string &n, const char *e);
    char *set_display(const std::string &n, const char *e);

};
}
#endif
