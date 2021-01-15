/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#ifdef _WIN32
#include <ws2tcpip.h>
#endif
#include <covise/covise.h>
#include <util/unixcompat.h>

#ifdef _WIN32
#include <io.h>
#include <direct.h>
#else
#include <arpa/inet.h>
#include <dirent.h>
#endif

#include <net/covise_host.h>
#include "control_coviseconfig.h"
#include "control_process.h"
#include "covise_module.h"

using namespace covise;

#define DEFAULT_TIMEOUT 30

//----------------------------------------------------------------------------
void ControlConfig::addhostinfo(const std::string &name, int s_mode, ExecType e_mode, int t)
//----------------------------------------------------------------------------
{
    //std::cerr << "adding host info for " << name << std::endl;
    if (!name.empty())
    {
        auto &h = hostMap[name];
        h.exectype = e_mode;
        h.shminfo = s_mode;
        h.display = NULL;
        h.timeout = t;
    }
}

//----------------------------------------------------------------------------
char *ControlConfig::getDisplayIP(const covise::Host &h)
//----------------------------------------------------------------------------
{
    getOrCreateHostInfo(h.getName());

    return hostMap[h.getName()].display;
}


ControlConfig::HostMap::iterator ControlConfig::getOrCreateHostInfo(const std::string &name)
{
    HostMap::iterator it = hostMap.find(name);
    if (it != hostMap.end())
    {
        return it;
    }
    it = hostMap.insert(HostMap::value_type{name, HostMap::value_type::second_type{}}).first;
    it->second.timeout = DEFAULT_TIMEOUT;
    it->second.exectype = ExecType::VRB;
    it->second.display = NULL;
    it->second.shminfo = COVISE_SHM;

    addhostinfo_from_config(name);

    return it;
}


//----------------------------------------------------------------------------
int ControlConfig::set_shminfo(const std::string &n, const char *shm_info)
//----------------------------------------------------------------------------
{
    HostMap::iterator it = getOrCreateHostInfo(n);

    int s_info = COVISE_SHM;
    int retval = sscanf(shm_info, "%d", &s_info);
    if (retval != 1)
    {
        std::cerr << "ControlConfig::set_shminfo_ip: sscanf failed" << std::endl;
    }

    it->second.shminfo = s_info;

    return (s_info);
}

//----------------------------------------------------------------------------
int ControlConfig::set_timeout(const std::string &n, const char *t)
//----------------------------------------------------------------------------
{
    HostMap::iterator it = getOrCreateHostInfo(n);

    int ti = DEFAULT_TIMEOUT;
    int retval = sscanf(t, "%d", &ti);
    if (retval != 1)
    {
        std::cerr << "ControlConfig::set_timeout: sscanf failed" << std::endl;
    }

    it->second.timeout = ti;

    return (ti);
}

//----------------------------------------------------------------------------
ExecType ControlConfig::set_exectype(const std::string &n, const char *exec_mode)
//----------------------------------------------------------------------------
{
    HostMap::iterator it = getOrCreateHostInfo(n);

    int e_mode = static_cast<int>(ExecType::VRB);
    int retval = sscanf(exec_mode, "%d", &e_mode);
    if (retval != 1)
    {
        std::cerr << "ControlConfig::set_exectype_ip: sscanf failed" << std::endl;
    }
    it->second.exectype = static_cast<ExecType>(e_mode);

    return (it->second.exectype);
}

//----------------------------------------------------------------------------
char *ControlConfig::set_display(const std::string &n, const char *dp)
//----------------------------------------------------------------------------
{
    HostMap::iterator it = getOrCreateHostInfo(n);

    char *dsp = NULL;
    if (dp && strlen(dp) > 1)
    {
        dsp = new char[strlen(dp) + 1];
        strcpy(dsp, dp);
    }

    it->second.display = dsp;

    return (dsp);
}

//----------------------------------------------------------------------------
ExecType ControlConfig::getexectype(const std::string &n)
//----------------------------------------------------------------------------
{
	return getOrCreateHostInfo(n)->second.exectype;
}

//----------------------------------------------------------------------------
int ControlConfig::gettimeout(const std::string &n)
//----------------------------------------------------------------------------
{
	return getOrCreateHostInfo(n)->second.timeout;
}

int covise::ControlConfig::gettimeout(const covise::Host & h)
{
    return gettimeout(std::string{h.getName()});
}

//----------------------------------------------------------------------------
int ControlConfig::getshminfo(const std::string &n)
//----------------------------------------------------------------------------
{
    return getOrCreateHostInfo(n)->second.shminfo;
}

void ControlConfig::addhostinfo_from_config(const std::string &name)
{
	std::string shm_mode, exec_mode;
    int s_mode, tim;
    ExecType e_mode;

    /// default values

    char key[1024];
    snprintf(key, sizeof(key), "System.HostConfig.Host:%s", name.c_str());
    for (int i = (int)strlen("System.HostConfig.Host:"); key[i]; i++)
    {
        if (key[i] == '.')
            key[i] = '_';
    }
	tim = coCoviseConfig::getInt("timeout", key, DEFAULT_TIMEOUT);
	shm_mode = coCoviseConfig::getEntry("memory", key, "shm");
	exec_mode = coCoviseConfig::getEntry("method", key, "ssh");

    s_mode = COVISE_SHM;
    if (strcasecmp(shm_mode.c_str(), "shm") == 0 || strcasecmp(shm_mode.c_str(), "sysv") == 0)
    {
        s_mode = COVISE_SHM;
    }
    else if (strcasecmp(shm_mode.c_str(), "posix") == 0)
    {
        s_mode = COVISE_POSIX;
    }
    else if (strcasecmp(shm_mode.c_str(), "mmap") == 0)
    {
        s_mode = COVISE_MMAP;
    }
    else if (strcasecmp(shm_mode.c_str(), "none") == 0)
    {
        s_mode = COVISE_NOSHM;
    }
    else if (strcasecmp(shm_mode.c_str(), "noshm") == 0)
    {
        s_mode = COVISE_NOSHM;
    }
    else if (strcasecmp(shm_mode.c_str(), "cray") == 0)
    {
        s_mode = COVISE_NOSHM;
    }
    else if (strcasecmp(shm_mode.c_str(), "proxie") == 0)
    {
        s_mode = COVISE_PROXIE;
    }
    else
    {
        print_error(__LINE__, __FILE__, "Wrong memory mode %s for %s, should be shm, mmap, or none (covise.config)! Using default shm", shm_mode.c_str(), name.c_str());
        fflush(stderr);
    }
    e_mode = ExecType::VRB;
    if (strcasecmp(exec_mode.c_str(), "rexec") == 0 ||
        strcasecmp(exec_mode.c_str(), "rsh") == 0 ||
        strcasecmp(exec_mode.c_str(), "ssh") == 0 ||
        strcasecmp(exec_mode.c_str(), "accessGrid") == 0 ||
        strcasecmp(exec_mode.c_str(), "SSLDaemon") == 0 ||
        strcasecmp(exec_mode.c_str(), "nqs") == 0 ||
        strcasecmp(exec_mode.c_str(), "remoteDaemon") == 0 ||
        strcasecmp(exec_mode.c_str(), "globus_gram") == 0)
    {
        std::cerr << exec_mode << "is no longer supported" << std::endl;
    }
    else if (strcasecmp(exec_mode.c_str(), "manual") == 0)
    {
        e_mode = ExecType::Manual;
    }
    else if (strcasecmp(exec_mode.c_str(), "vrb") == 0)
    {
        e_mode = ExecType::VRB;
    }
    else
    {
        print_error(__LINE__, __FILE__, "Wrong exec mode %s for %s, should be vrb, manual or script (covise.config)! Using default vrb", shm_mode.c_str(), name.c_str());
        fflush(stderr);
    }
    if (tim == 0)
        tim = DEFAULT_TIMEOUT;

    addhostinfo(name, s_mode, e_mode, tim);
}
