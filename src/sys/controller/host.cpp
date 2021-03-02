/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "host.h"
#include "handler.h"
#include "util.h"
#include "exception.h"
#include "module.h"
#include "userinterface.h"
#include "crb.h"
#include "renderModule.h"
#include "global.h"

#include <comsg/CRB_EXEC.h>
#include <comsg/NEW_UI.h>
#include <net/covise_host.h>
#include <net/message_types.h>
#include <util/coSpawnProgram.h>
#include <vrb/VrbSetUserInfoMessage.h>
#include <vrb/client/LaunchRequest.h>
#include <vrb/client/VRBClient.h>
#include <util/covise_version.h>

#include <chrono>
#include <algorithm>
#include <cassert>

using namespace covise;
using namespace covise::controller;

const Module &RemoteHost::getModule(sender_type type) const
{
    return const_cast<RemoteHost *>(this)->getModule(type);
}

Module &RemoteHost::getModule(sender_type type)
{
    auto it = std::find_if(m_modules.begin(), m_modules.end(), [type](const ProcessList::value_type &m) {
        return m->type == type;
    });
    if (it != m_modules.end())
    {
        return *it->get();
    }
    throw Exception{"RemoteHost did not find module of type " + std::to_string(type)};
}

const Application &RemoteHost::getApplication(const std::string &name, int instance) const
{
    return const_cast<RemoteHost *>(this)->getApplication(name, instance);
}

Application &RemoteHost::getApplication(const std::string &name, int instance)
{
    auto app = std::find_if(begin(), end(), [&name, &instance](const std::unique_ptr<Module> &mod) {
        if (const auto app = dynamic_cast<const Application *>(&*mod))
        {
            return app->info().name == name && app->instance() == instance;
        }
    });
    if (app != end())
    {
        return *dynamic_cast<Application *>(&**app);
    }
    throw Exception{"RemoteHost did not find application module " + name + "_" + std::to_string(instance)};
}

void RemoteHost::removeApplication(Application &app, int alreadyDead)
{
    app.setDeadFlag(alreadyDead);
    m_modules.erase(std::remove_if(m_modules.begin(), m_modules.end(), [&app](const std::unique_ptr<Module> &mod) {
                        return &*mod == &app;
                    }),
                    m_modules.end());
}

RemoteHost::ProcessList::const_iterator RemoteHost::begin() const
{
    return m_modules.begin();
}

RemoteHost::ProcessList::iterator RemoteHost::begin()
{
    return m_modules.begin();
}

RemoteHost::ProcessList::const_iterator RemoteHost::end() const
{
    return m_modules.end();
}

RemoteHost::ProcessList::iterator RemoteHost::end()
{
    return m_modules.end();
}

bool RemoteHost::startCrb(ShmMode shmMode)
{
    try
    {
        auto execName = shmMode == ShmMode::Proxie ? vrb::Program::crbProxy : vrb::Program::crb;
        auto m = m_modules.emplace(m_modules.end(), new CRBModule{*this, shmMode == ShmMode::Proxie});
        auto crbModule = m->get()->as<CRBModule>();
        if (!crbModule->setupConn([this, &crbModule, execName](int port) {
                std::vector<std::string> args;
                args.push_back(std::to_string(port));
                args.push_back(covise::Host::getHostaddress());
                args.push_back(std::to_string(crbModule->id));
                launchCrb(execName, args);
                return true;
            }))
        {
            std::cerr << "startCrb failed to spawn CRB connection" << std::endl;
            return false;
        }
        connectShm(*crbModule);
        if (!crbModule->init())
        {
            return false;
        }
        determineAvailableModules(*crbModule);
        Message msg = crbModule->initMessage;
        msg.type = COVISE_MESSAGE_UI;
        hostManager.sendAll<Userinterface>(msg);

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        std::cerr << "startCrb called on host" << userInfo().hostName << " but crb is already running!" << std::endl;
        return false;
    }
}

void RemoteHost::connectShm(const CRBModule &crbModule)
{
    //only needed for localhost
}

void RemoteHost::determineAvailableModules(const CRBModule &crb)
{
    // get number of new modules from message
    //LIST      0
    //HOST      1
    //USER      2
    //NUMBER    3
    auto list = splitString(crb.initMessage.data.data(), "\n");
    int mod_count = std::stoi(list[3]);
    int iel = 4;
    for (int i = 0; i < mod_count; i++)
    {
        m_availableModules.emplace_back(&hostManager.registerModuleInfo(list[iel], list[iel + 1]));
        iel = iel + 2;
    }
}
namespace covise
{
    namespace controller
    {
        namespace detail
        {

#ifdef _WIN32
            const char *PythonInterfaceExecutable = "..\\..\\Python\\scriptInterface.bat ";
#else
            const char *PythonInterfaceExecutable = "scriptInterface ";
#endif
        }
    }
}

bool RemoteHost::startUI(const UIOptions &options, const RemoteHost &master)
{
    cerr << "* Starting user interface....                                                 *" << endl;
    std::unique_ptr<Userinterface> ui;
    switch (options.type)
    {
    case UIOptions::python:
    {
        m_pythonUiInfo.reset(new StaticModuleInfo{detail::PythonInterfaceExecutable + options.pyFile, "python based command line interface"});
        auto modInfo = *m_availableModules.emplace(m_availableModules.end(), &*m_pythonUiInfo);
        ui.reset(new PythonInterface{*this, *modInfo});
    }
    break;
    case UIOptions::gui:
    {
        ui.reset(new MapEditor{*this});
    }
    break;

    default:
#ifdef HAVE_GSOAP
        ui.reset(new WsInterface{*this});
#else
        return false;
#endif
        break;
    }
    return startUI(std::move(ui), options, master);
}

bool RemoteHost::startUI(std::unique_ptr<Userinterface> &&ui, const UIOptions &options, const RemoteHost &master)
{
    if (master.ID() == this->ID())
    {
        ui->setStatus(Userinterface::Master);
    }
    try
    {
        if (ui->start(options, dynamic_cast<const CRBModule&>(master.getModule(CRB)), false))
        {
            m_modules.push_back(std::move(ui));
            return true;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        std::cerr << "startUI failed: no CRB running on master" << std::endl;
        return false;
    }
}

void RemoteHost::launchProcess(const CRB_EXEC &exec) const
{
    try
    {
        auto &crb = getModule(sender_type::CRB);
        sendCoviseMessage(exec, crb);
    }
    catch (const Exception &e)
    {
        std::cerr << e.what() << '\n';
        std::cerr << "failed to start module " << exec.name << " on host " << exec.moduleHostName << ": no crb running on that host" << std::endl;
    }
}

bool RemoteHost::isModuleAvailable(const std::string &moduleName) const
{
    auto it = std::find_if(m_availableModules.begin(), m_availableModules.end(), [&moduleName](const StaticModuleInfo *info) { return info->name == moduleName; });
    return it != m_availableModules.end();
}

Application &RemoteHost::startApplicationModule(const string &name, const string &instanz,
                                                      int posx, int posy, int copy, ExecFlag flags, Application *mirror)
{
    // check the Category of the Module
    auto moduleInfo = std::find_if(m_availableModules.begin(), m_availableModules.end(), [&name](const StaticModuleInfo *info) {
        return info->name == name;
    });
    if (moduleInfo == m_availableModules.end())
    {
        throw Exception{"failed to start " + name + "on " + userInfo().hostName + ": module not available!"};
    }
    int nr = std::stoi(instanz);
    Module *module = nullptr;
    if ((*moduleInfo)->category == "Renderer")
    {
        module = &**m_modules.emplace(m_modules.end(), new Renderer{*this, **moduleInfo, nr});
    }
    else
    {
        module = &**m_modules.emplace(m_modules.end(), new Application{*this, **moduleInfo, nr});
    }
    // set initial values
    auto &app = dynamic_cast<Application &>(*module);
    app.init({posx, posy}, copy, flags, mirror);
    return app;
}

bool RemoteHost::get_mark() const
{
    return m_saveInfo;
}

void RemoteHost::reset_mark()
{
    m_saveInfo = false;
}

void RemoteHost::mark_save()
{
    m_saveInfo = true;
}

void RemoteHost::launchCrb(vrb::Program exec, const std::vector<std::string> &cmdArgs)
{
    switch (m_exectype)
    {
    case controller::ExecType::VRB:
        vrb::sendLaunchRequestToRemoteLaunchers(vrb::VRB_MESSAGE{exec, ID(), cmdArgs}, &hostManager.getVrbClient());
        break;
    case controller::ExecType::Manual:
        launchManual(exec, cmdArgs);
        break;
    case controller::ExecType::Script:
        launchScipt(exec, cmdArgs);
        break;
    default:
        break;
    }
}

void LocalHost::launchCrb(vrb::Program exec, const std::vector<std::string> &cmdArgs)
{
    auto execPath = coviseBinDir() + vrb::programNames[exec];
    spawnProgram(execPath, cmdArgs);
}

void LocalHost::connectShm(const CRBModule &crbModule)
{
    CTRLGlobal::getInstance()->controller->get_shared_memory(crbModule.conn());
}

covise::LaunchStyle RemoteHost::state() const
{
    return m_state;
}

void RemoteHost::setTimeout(int seconds)
{
    m_timeout = seconds;
}

void RemoteHost::launchScipt(vrb::Program exec, const std::vector<std::string> &cmdArgs)
{
    std::stringstream start_string;
    std::string script_name;
    start_string << script_name << " " << vrb::programNames[exec];
    for (const auto arg : cmdArgs)
        start_string << " " << arg;
    start_string << " " << userInfo().hostName;
    int retval;
    retval = system(start_string.str().c_str());
    if (retval == -1)
    {
        std::cerr << "Controller::start_datamanager: system failed" << std::endl;
    }
}

void RemoteHost::launchManual(vrb::Program exec, const std::vector<std::string> &cmdArgs)
{
    std::stringstream text;
    text << "please start \"" << vrb::programNames[exec];
    for (const auto arg : cmdArgs)
        text << " " << arg;
    text << "\" on " << userInfo().hostName;
    Message msg{COVISE_MESSAGE_COVISE_ERROR, text.str()};
    hostManager.getMasterUi().send(&msg);
    std::cerr << text.str() << std::endl;
}

RemoteHost::RemoteHost(const HostManager &manager, vrb::Program type, const std::string &sessionName)
    : RemoteClient(type, sessionName), hostManager(manager)
{
}

RemoteHost::RemoteHost(const HostManager &manager, vrb::RemoteClient &&base)
    : RemoteClient(std::move(base)), hostManager(manager)
{
}

void RemoteHost::handleAction(covise::LaunchStyle action)
{
    m_state = action;
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
    startCrb(CTRLHandler::instance()->Config->getshmMode(userInfo().hostName));
    startUI(CTRLHandler::instance()->uiOptions(), hostManager.getMasterUi().host);
}

HostManager::HostManager()
    : m_localHost(m_hosts.insert(
                             HostMap::value_type(0, std::unique_ptr<RemoteHost>{new LocalHost{*this, vrb::Program::covise}}))
                      .first),
      m_vrb(vrb::Program::covise), m_thread([this]() {
          handleVrb();
      })
{
    m_localHost->second->startCrb(CTRLHandler::instance()->Config->getshmMode(m_localHost->second->userInfo().hostName));
}

HostManager::~HostManager()
{
    m_vrb.shutdown();
    m_thread.join();
}

covise::Message HostManager::sendPartnerList()
{
    std::lock_guard<std::mutex> g{m_mutex};
    std::cerr << "vrb remote launchers requested:" << std::endl;
    ClientList cll{m_hosts.size()};
    std::transform(m_hosts.begin(), m_hosts.end(), cll.begin(),
                   [](const HostMap::value_type &host) {
                       return ClientList::value_type{host.second->ID(), host.second->userInfo().hostName};
                   });
    NEW_UI_AvailablePartners p{cll};
    auto msg = p.createMessage();
    sendAll<Userinterface>(msg);
}

void HostManager::handleAction(const covise::NEW_UI_HandlePartners &msg)
{
    std::lock_guard<std::mutex> g{m_mutex};
    for (auto clID : msg.clients)
    {
        auto hostIt = m_hosts.find(clID);
        if (hostIt != m_hosts.end())
        {
            hostIt->second->handleAction(msg.launchStyle);
            hostIt->second->setTimeout(msg.timeout);
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

const vrb::VRBClient &HostManager::getVrbClient() const
{
    return m_vrb;
}

LocalHost &HostManager::getLocalHost()
{
    assert(dynamic_cast<LocalHost *>(m_localHost->second.get()));
    return *dynamic_cast<LocalHost *>(m_localHost->second.get());
}

const LocalHost &HostManager::getLocalHost() const
{
    return const_cast<HostManager *>(this)->getLocalHost();
}

RemoteHost *HostManager::getHost(int clientID)
{
    auto h = m_hosts.find(clientID);
    if (h != m_hosts.end())
    {
        return h->second.get();
    }
    return nullptr;
}

const RemoteHost *HostManager::getHost(int clientID) const
{
    return const_cast<HostManager *>(this)->getHost(clientID);
}

RemoteHost &HostManager::findHost(const std::string &hostName)
{
    auto h = std::find_if(m_hosts.begin(), m_hosts.end(), [&hostName](HostMap::value_type &host) {
        return host.second->userInfo().hostName == hostName;
    });
    if (h != m_hosts.end())
    {
        return *h->second.get();
    }
    throw Exception{"HostManager could not find host " + hostName};
}

const RemoteHost &HostManager::findHost(const std::string &hostName) const
{
    return const_cast<HostManager *>(this)->findHost(hostName);
}

std::vector<const Module *> HostManager::getAllModules(sender_type type) const
{
    std::vector<const Module *> modules;
    for (const auto &host : m_hosts)
    {
        for (const auto &module : *host.second)
        {
            if (type == sender_type::ANY || module->type == type)
            {
                modules.push_back(&*module);
            }
        }
    }
    return modules;
}

HostManager::HostMap::const_iterator HostManager::begin() const
{
    return m_hosts.begin();
}

HostManager::HostMap::iterator HostManager::begin() 
{
    return m_hosts.begin();
}

HostManager::HostMap::const_iterator HostManager::end() const
{
    return m_hosts.end();
}

HostManager::HostMap::iterator HostManager::end()
{
    return m_hosts.end();
}

Module *HostManager::findModule(int peerID)
{
    for (auto &host : m_hosts)
    {
        for (auto &module : *host.second)
        {
            if (auto render = module->as<Renderer>())
            {
                if (render->getDisplay(peerID) != render->end())
                {
                    return render;
                }
            }
            else
            {
                if (module->id == peerID)
                {
                    return &*module;
                }
            }
        }
    }
    return nullptr;
}

bool HostManager::slaveUpdate()
{
    if (!m_slaveUpdate)
    {
        return false;
    }
    for (const Renderer *renderer : getAllModules<Renderer>())
    {
        auto &masterUi = getMasterUi();
        auto display = std::find_if(renderer->begin(), renderer->end(), [&masterUi](const Renderer::DisplayList::value_type &disp) {
            return &disp->host == &masterUi.host;
        });
        if (display != renderer->end())
        {
            ostringstream os;
            os << "UPDATE\n"
               << renderer->info().name << "\n"
               << renderer->instance() << "\n"
               << masterUi.host.userInfo().hostName << "\n";
            Message msg{COVISE_MESSAGE_RENDER, os.str()};
            display->get()->send(&msg);
        }
    }
    m_slaveUpdate = false;
    return true;
}

const Userinterface &HostManager::getMasterUi() const
{
    return const_cast<HostManager *>(this)->getMasterUi();
}

Userinterface &HostManager::getMasterUi()
{
    auto uis = getAllModules<Userinterface>();
    auto masterUi = std::find_if(uis.begin(), uis.end(), [](const Userinterface *ui) {
        return ui->status() == Userinterface::Status::Master;
    });
    assert(masterUi != uis.end());
    return **masterUi;
}

std::string HostManager::getHostsInfo() const
{

    std::stringstream buffer;
    buffer << m_hosts.size() << "\n";
    for (const auto &h : *this)
    {
        auto &host = *h.second;
        if (host.get_mark())
        {
            if (&host == &getLocalHost())
            {
                buffer << "LOCAL\nLUSTER";
            }
            else
            {
                buffer << host.userInfo().hostName << "\n"
                       << host.userInfo().userName;
                try
                {
                    host.getModule(sender_type::USERINTERFACE);
                    buffer << " Partner";
                }
                catch (const Exception &e)
                {
                }
            }
            buffer << "\n";
        }
    }
    return buffer.str();
}

const StaticModuleInfo &HostManager::registerModuleInfo(const std::string &name, const std::string &category) const
{
    return *m_availableModules.insert(StaticModuleInfo{name, category}).first;
}

void HostManager::resetModuleInstances()
{
    for (const auto &info : m_availableModules)
        info.count = 0;
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
                auto old = m_localHost;
                m_localHost = m_hosts.insert(HostMap::value_type{uim.myClientID, std::move(old->second)}).first;
                m_localHost->second->setID(uim.myClientID);
                m_localHost->second->setSession(uim.mySession);

                m_hosts.erase(old);
                if (m_onConnectCallBack)
                {
                    m_onConnectCallBack();
                }
            }
            for (auto &cl : uim.otherClients)
            {
                if (cl.userInfo().userType == vrb::Program::VrbRemoteLauncher)
                {
                    m_hosts.insert(HostMap::value_type{cl.ID(), std::unique_ptr<RemoteHost>{new RemoteHost{*this, std::move(cl)}}});
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
                    if (clIt->second->state() == LaunchStyle::Disconnect)
                    {
                        m_hosts.erase(clIt);
                    }
                    else
                    {
                        //disconnect partner
                    }
                }
            }
        }
        break;
        case COVISE_MESSAGE_SOCKET_CLOSED:
        case COVISE_MESSAGE_CLOSE_SOCKET:
        case COVISE_MESSAGE_VRB_CLOSE_VRB_CONNECTION:
        {
            auto old = m_localHost;
            std::unique_ptr<RemoteHost> local{std::move(m_localHost->second)};
            m_hosts.clear();
            m_localHost = m_hosts.insert(HostMap::value_type{0, std::move(local)}).first;
            m_localHost->second->setID(0);
            std::cerr << "lost connection to vrb" << std::endl;
            m_vrb.connectToServer();
        }
        break;
        default:
            break;
        }
    }
}
