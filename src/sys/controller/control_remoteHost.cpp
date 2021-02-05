#include "control_remoteHost.h"
#include "CTRLHandler.h"
#include "control_coviseconfig.h"

#include <comsg/NEW_UI.h>
#include <net/message_types.h>
#include <vrb/VrbSetUserInfoMessage.h>
#include <vrb/client/LaunchRequest.h>
#include <util/coSpawnProgram.h>
#include <net/covise_host.h>

#include <chrono>
#include <algorithm>

using namespace covise;
using namespace covise::controller;


const Module *RemoteHost::getModule(sender_type type) const{
    auto it = std::find_if(m_modules.begin(), m_modules.end(), [type](const Module &m) {
        return m.type == type;
    });
    if (it != m_modules.end())
    {
        return &*it;
    }
    return nullptr;
}

std::vector<Module>::const_iterator RemoteHost::begin() const{
    return m_modules.begin();
}

std::vector<Module>::const_iterator RemoteHost::end() const{
    return m_modules.end();
}


bool RemoteHost::startCrb(){
    
    std::string execName = CTRLHandler::instance()->Config->getshmMode(userInfo().name) == ShmMode::Proxie ? "crbProxy" : "crb";
    Module m{*this, sender_type::CRB, execName};

    int port = 0;
    std::unique_ptr<ServerConnection> conn(new ServerConnection(&port, m.id, CONTROLLER));
    conn->listen();
    if (!conn->is_connected())
        return false;
    std::vector<std::string> args;
    args.push_back(std::to_string(port));
    args.push_back(covise::Host::getHostaddress());
    args.push_back(std::to_string(m.id));
    const char *covisedir = getenv("COVISEDIR");
    const char *archsuffix = getenv("ARCHSUFFIX");
    if (covisedir && archsuffix)
    {
        std::stringstream crb;
        crb << covisedir << "/" << archsuffix << "/bin/" << vrb::programNames[vrb::Program::crb];
        covise::spawnProgram(crb.str(), args);
    }
    if (conn->acceptOne(m_timeout) < 0)
    {
        cerr << "* timelimit in accept for crb exceeded!!" << endl;
        return false;
    }
    CTRLGlobal::getInstance()->controller->get_shared_memory(&*conn);

    m.setConn(std::move(conn));
    m_modules.push_back(m);
    return true;
}

bool RemoteHost::startUI(const UIOptions &options, const RemoteHost& crb){
    cerr << "* Starting user interface....                                                 *" << endl;
    switch (options.type)
    {
    case UIOptions::python:
        startPythonInterface(options, crb);
        break;
    case UIOptions::gui:
        startMapeditor(options, crb);
#ifdef HAVE_GSOAP
        startWebService(options, crb);
#endif
        break;

    default:
        break;
    }
}



bool RemoteHost::startMapeditor(const UIOptions &options,const RemoteHost& host)
{
    UIMapEditor local{*this};
    setUIParams(local);
    auto crb = host.getModule(CRB);
    if (crb && local.start(*crb, false))
    {
        m_modules.push_back(local);
        if (options.iconify)
        {
            Message msg{COVISE_MESSAGE_UI, "ICONIFY"};
            local.send(&msg);
        }

        if (options.maximize)
        {
#ifndef _AIRBUS
            Message msg{COVISE_MESSAGE_UI, "MAXIMIZE"};
#else
            Message msg{COVISE_MESSAGE_UI, "PREPARE_CSCW"};
#endif
            local.send(&msg);
        }
        return true;
    }
    return false;
}

bool RemoteHost::startWebService(const UIOptions &options,const RemoteHost& host)
{
    UISoap local{*this};
    setUIParams(local);
    auto crb = host.getModule(CRB);
    if (crb && local.start(*crb, false))
    {
        m_modules.push_back(local);
        return true;
    }
    return false;
}

// test by RM
bool RemoteHost::startPythonInterface(const UIOptions &options,const RemoteHost& host)
{
    UIMapEditor ui{*this};
    if (host.ID() == this->ID())
    {
        ui.set_status("MASTER");
    }
    
    auto crb = host.getModule(CRB);
    if (ui.xstart(options.pyFile))
    {
        m_modules.push_back(ui);
        return true;
    }
    return false;
}

void RemoteHost::setUIParams(userinterface &ui)
{
    ui.set_host(userInfo().hostName);
    ui.set_userid(userInfo().name);
    ui.set_status("MASTER");
}



covise::LaunchStyle RemoteHost::state() const
{
    return m_state;
}

void RemoteHost::setTimeout(int seconds)
{
    m_timeout = seconds;
}

void RemoteHost::launch(const std::string &ipAdress, int port, int moduleCount, covise::MessageSenderInterface &sender)
{
    switch (m_exectype)
    {
    case controller::ExecType::VRB:
        sendLaunchRequest(ipAdress, port, moduleCount, , sender);
        break;
    case controller::ExecType::Manual:
        launchManual(ipAdress, port, moduleCount);
        break;
    case controller::ExecType::Script:
        launchScipt(ipAdress, port, moduleCount);
        break;
    default:
        break;
    }
}

void RemoteHost::launchScipt(const std::string &ipAdress, int port, int moduleCount)
{
    std::stringstream start_string;
    std::string script_name;
    const char *del = " ";
    start_string << script_name << del << vrb::programNames[userInfo().userType] << del << port
                 << del << ipAdress << del << moduleCount - 1;
    int retval;
    retval = system(start_string.str().c_str());
    if (retval == -1)
    {
        std::cerr << "Controller::start_datamanager: system failed" << std::endl;
    }
}

void RemoteHost::launchManual(const std::string &ipAdress, int port, int moduleCount)
{
    std::stringstream text;
    text << "please start " << vrb::programNames[userInfo().userType] << " " << port
         << " " << ipAdress << " " << moduleCount << " on " << userInfo().hostName;
    Message msg{COVISE_MESSAGE_COVISE_ERROR, text.str()};
    CTRLGlobal::getInstance()->userinterfaceList->send_master(&msg);
    std::cerr << text.str() << std::endl;
}

void RemoteHost::handleAction(covise::LaunchStyle action)
{

    switch (action)
    {
    case covise::LaunchStyle::Partner:
        addPartner();
        break;

    default:
        break;
    }
}

void RemoteHost::addPartner()
{
}

void RemoteHost::sendLaunchRequest(const std::string &ipAdress, int port, int moduleCount,
                                   const vrb::VrbCredentials &vrbCredentials, covise::MessageSenderInterface &sender)
{

    std::vector<std::string> args;
    //    args.emplace_back(std::to_string(port));
    args.emplace_back(ipAdress);
    args.emplace_back(std::to_string(moduleCount));
    args.emplace_back(vrbCredentials.ipAddress);
    args.emplace_back(std::to_string(vrbCredentials.tcpPort));
    args.emplace_back(std::to_string(vrbCredentials.udpPort));

    vrb::sendLaunchRequestToRemoteLaunchers(vrb::VRB_MESSAGE{userInfo().userType, ID(), args}, &sender);
}

HostManager::HostManager()
    : m_localHost(vrb::Program::covise)
    , m_vrb(vrb::Program::covise)
    , m_thread([this]() {
          handleVrb();
      })
{
    startLocalCrb();
}

HostManager::~HostManager()
{
    m_vrb.shutdown();
    m_thread.join();
}

covise::Message HostManager::uiUpdate()
{
    std::lock_guard<std::mutex> g{m_mutex};
    std::cerr << "vrb remote launchers requested:" << std::endl;
    ClientList cll{m_hosts.size()};
    std::transform(m_hosts.begin(), m_hosts.end(), cll.begin(),
                   [](const std::pair<const int, RemoteHost> &host) {
                       return ClientList::value_type{host.second.ID(), host.second.userInfo().hostName};
                   });
    NEW_UI_AvailablePartners p{cll};
    auto msg = p.createMessage();
    CTRLGlobal::getInstance()->userinterfaceList->send_all(&msg);
}

void HostManager::handleAction(const covise::NEW_UI_HandlePartners &msg)
{
    std::lock_guard<std::mutex> g{m_mutex};
    for (auto clID : msg.clients)
    {
        auto hostIt = m_hosts.find(clID);
        if (hostIt != m_hosts.end())
        {
            hostIt->second.handleAction(msg.launchStyle);
            hostIt->second.setTimeout(msg.timeout);
        }
    }
}

void HostManager::setOnConnectCallBack(std::function<void(void)> cb)
{
    std::lock_guard<std::mutex> g{m_mutex};
    m_onConnectCallBack = cb;
}

int HostManager::vrbClientID() const
{
    std::lock_guard<std::mutex> g{m_mutex};
    return m_vrb.ID();
}

RemoteHost &HostManager::getLocalHost(){
    return m_localHost;
}

RemoteHost *HostManager::getHost(int clientID){
    auto h = m_hosts.find(clientID);
    if (h != m_hosts.end())
    {
        return &h->second;
    }
    return nullptr;
}

void HostManager::handleVrb()
{
    using namespace covise;
    m_vrb.connectToServer();
    while (true)
    {
        while (!m_vrb.isConnected())
        {
            std::this_thread::sleep_for(std::chrono::duration<int>{1});
        }
        covise::Message msg;
        m_vrb.wait(&msg);

        switch (msg.type)
        {
        case COVISE_MESSAGE_VRB_SET_USERINFO:
        {

            std::lock_guard<std::mutex> g{m_mutex};
            vrb::UserInfoMessage uim(&msg);
            if (uim.hasMyInfo)
            {
                m_vrb.setID(uim.myClientID);
                m_vrb.setSession(uim.mySession);
                if (m_onConnectCallBack)
                {
                    m_onConnectCallBack();
                }
            }
            for (auto &cl : uim.otherClients)
            {
                if (cl.userInfo().userType == vrb::Program::VrbRemoteLauncher)
                {
                    m_hosts.insert(std::pair<int, RemoteHost>{cl.ID(), std::move(cl)});
                }
            }
        }
        break;
        case COVISE_MESSAGE_VRB_QUIT:
        {
            TokenBuffer tb{&msg};
            int id;
            tb >> id;
            if (id != m_vrb.ID())
            {
                std::lock_guard<std::mutex> g{m_mutex};
                auto clIt = m_hosts.find(id);
                if (clIt != m_hosts.end())
                {
                    if (clIt->second.state() == LaunchStyle::Disconnect)
                    {
                        m_hosts.erase(clIt);
                    }
                    else
                    {
                        //
                    }
                }
            }
        }
        break;
        case COVISE_MESSAGE_SOCKET_CLOSED:
        case COVISE_MESSAGE_CLOSE_SOCKET:
        case COVISE_MESSAGE_VRB_CLOSE_VRB_CONNECTION:
        {
            std::cerr << "lost connection to vrb" << std::endl;
            m_localHost.connectToServer();
        }
        break;
        default:
            break;
        }
    }
}

void HostManager::startLocalCrb()
{

    int port = 0;

    m_moduleCount++;
    ServerConnection *conn = new ServerConnection(&port, m_moduleCount, CONTROLLER);
    conn->listen();
    if (!conn->is_connected())
        return;
    std::vector<std::string> args;
    args.push_back(std::to_string(port));
    args.push_back(covise::Host::getHostaddress());
    args.push_back(std::to_string(m_moduleCount));
    const char *covisedir = getenv("COVISEDIR");
    const char *archsuffix = getenv("ARCHSUFFIX");
    if (covisedir && archsuffix)
    {
        std::stringstream crb;
        crb << covisedir << "/" << archsuffix << "/bin/" << vrb::programNames[vrb::Program::crb];
        covise::spawnProgram(crb.str(), args);
    }
    

    if (confirmConnection(conn, 5))
    {
        m_crbConn = conn;
        CTRLGlobal::getInstance()->controller->get_shared_memory(conn);
    }
    else
    {
        m_crbConn = nullptr;
    }
}

bool HostManager::confirmConnection(covise::ServerConnection *conn, int timeout)
{
    if (conn->acceptOne(timeout) < 0)
    {
        delete conn;
        cerr << "* timelimit in accept for crb exceeded!!" << endl;
        return false;
    }
    m_connList.add(conn);
    return true;
}