/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef CONTROL_REQUEST_BROKER_H
#define CONTROL_REQUEST_BROKER_H

#include "process.h"
#include <net/message_types.h>


namespace covise
{
namespace controller
{
struct RemoteHost;

class CRBModule : public Module{
public:
    static const Module::Type moduleType = Module::Type::Crb;
    static StaticModuleInfo crbModuleInfo;
    static StaticModuleInfo crbProxyModuleInfo;
    CRBModule(const RemoteHost &host, bool proxy);
    virtual ~CRBModule();
    Message initMessage, interfaceMessage;
    std::string covisePath;
    bool init();

private:
    bool checkCoviseVersion(const Message &versionMessage, const std::string &hostname);
    void sendMaster(const Message &msg);
    bool tryReceiveMessage(Message &msg);
    void prepareInitMessageForUIs();
    void queryDataPath();
    bool connectOtherCRB(const Module &crb);
};

} // namespace controller
    
} // namespace covise



#endif // !CONTROLREQUEST_BROKER_H