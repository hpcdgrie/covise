#ifndef CONTROL_MODULE_MANAGER_H
#define CONTROL_MODULE_MANAGER_H

#include "control_remoteHost.h"
namespace covise
{
namespace controller
{

class ModuleManager
{
public:
    ModuleManager();



private:
    HostManager m_hosts;
};
} // namespace controller
} // namespace covise

#endif