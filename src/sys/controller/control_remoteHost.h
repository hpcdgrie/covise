#ifndef CONTROLLER_REMOTE_HOST_H
#define CONTROLLER_REMOTE_HOST_H

#include "covise_module.h"
#include <vrb/RemoteClient.h>
#include <vrb/client/VRBClient.h>
#include <comsg/coviseLaunchOptions.h>

#include <thread>
#include <vector>
#include <map>
#include <mutex>
namespace covise
{
    class NEW_UI_HandlePartners;

namespace controller{


enum class ExecType
{
    Local,
    VRB,
    Manual,
    Script
};

struct RemoteHost : vrb::RemoteClient
{
    using vrb::RemoteClient::RemoteClient;
    void handleAction(covise::LaunchStyle action);
    covise::LaunchStyle state() const;
    void setTimeout(int seconds);
    void launch(const std::string& ipAdress,int port, int moduleCount, covise::MessageSenderInterface &sender);
    bool startCrb();
    bool startUI(const UIOptions &options, const RemoteHost& crb);
    const Module *getModule(sender_type type) const;
    std::vector<Module>::const_iterator begin() const;
    std::vector<Module>::const_iterator end() const;

private:
    void addPartner();
    void sendLaunchRequest(const std::string &ipAdress, int port, int moduleCount,
                           const vrb::VrbCredentials &vrbCredentials, covise::MessageSenderInterface &sender);
    void launchScipt(const std::string &ipAdress, int port, int moduleCount);
    void launchManual(const std::string& ipAdress,int port, int moduleCount);

    bool RemoteHost::startMapeditor(const UIOptions &options,const RemoteHost& crb);
    bool RemoteHost::startWebService(const UIOptions &options,const RemoteHost& crb);
    bool RemoteHost::startPythonInterface(const UIOptions &options,const RemoteHost& crb);
    void setUIParams(userinterface &localUi);
    covise::LaunchStyle m_state = covise::LaunchStyle::Disconnect;
    controller::ExecType m_exectype = controller::ExecType::VRB;
    std::vector<Module> m_modules;
    int m_shmID;
    int m_timeout = 30;
};

class HostManager
{
public:
    HostManager();
    ~HostManager();
    covise::Message uiUpdate();
    void handleAction(const covise::NEW_UI_HandlePartners &msg);
    void setOnConnectCallBack(std::function<void(void)> cb);
    int vrbClientID() const;
    RemoteHost &getLocalHost();
    RemoteHost *getHost(int clientID);


private:
    vrb::VRBClient m_vrb;
    RemoteHost m_localHost;
    std::map<int, RemoteHost> m_hosts;
    std::thread m_thread;
    std::mutex m_mutex;
    std::function<void(void)> m_onConnectCallBack;
    std::atomic_int m_moduleCount{0};
    covise::ConnectionList m_connList;
    covise::ServerConnection *m_crbConn = nullptr;
    void handleVrb();
    void startLocalCrb();
    bool confirmConnection(covise::ServerConnection *conn, int timeout);

};
} //namespace controller
} //namespace covise
#endif