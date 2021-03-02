/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "handler.h"
#include "host.h"
#include "util.h"
#include "module.h"
#include "exception.h"
#include "global.h"

#include <comsg/CRB_EXEC.h>
#include <covise/covise.h>
#include <covise/covise_msg.h>
#include <covise/covise_process.h>
#include <net/covise_host.h>
#include <net/covise_socket.h>

const int SIZEOF_IEEE_INT = 4;

using namespace covise;
using namespace covise::controller;

void ModuleNetConnectivity::addInterface(const string &name, const string &type, controller::Direction direction, const string &text, const string &demand)
{
    auto inter = interfaces.emplace(interfaces.end());
    inter->set_name(name);
    inter->set_type(type);
    inter->set_direction(direction);
    inter->set_text(text);
    inter->set_demand(demand);
}

void ModuleNetConnectivity::addParameter(const string &name, const string &type, const string &text, const string &value, const string ext, Direction dir)
{
    parameter *param;
    if (dir == Direction::Input)
        param = &*inputParams.emplace(inputParams.end());
    else if (dir == Direction::Output)
        param = &*outputParams.emplace(outputParams.end());

    param->set_name(name);
    param->set_type(type);
    param->set_text(text);
    param->set_extension(ext);
    param->set_value_list(value);
}

const parameter &ModuleNetConnectivity::getParam(const std::string &paramName) const
{
    return const_cast<ModuleNetConnectivity *>(this)->getParam(paramName);
}

parameter &ModuleNetConnectivity::getParam(const std::string &paramName)
{
    auto param = getParameter(inputParams, paramName);
    if (!param)
    {
        param = getParameter(outputParams, paramName);
    }
    if (!param)
    {
        throw Exception{"ModuleNetConnectivity did not find parameter " + paramName};
    }
    return *param;
}

const net_interface &ModuleNetConnectivity::getInterface(const std::string &interfaceName) const
{
    return const_cast<ModuleNetConnectivity *>(this)->getInterface(interfaceName);
}

net_interface &ModuleNetConnectivity::getInterface(const std::string &interfaceName)
{
    auto it = std::find_if(interfaces.begin(), interfaces.end(), [&interfaceName](const C_interface &inter) {
        return inter.get_name() == interfaceName;
    });
    if (it != interfaces.end())
    {
        if (auto netInter = dynamic_cast<net_interface *>(&*it))
            return *netInter;
    }
    throw Exception{"ModuleNetConnectivity did not find interface " + interfaceName};
}

void ModuleNetConnectivity::forAllNetInterfaces(const std::function<void(net_interface &)> &func)
{
    for (auto &inter : interfaces)
    {
        if (auto netInter = dynamic_cast<net_interface *>(&inter))
            func(*netInter);
    }
}

void ModuleNetConnectivity::forAllNetInterfaces(const std::function<void(const net_interface &)> &func) const
{
    for (auto &inter : interfaces)
    {
        if (auto netInter = dynamic_cast<const net_interface *>(&inter))
            func(*netInter);
    }
}

const parameter *controller::getParameter(const std::vector<parameter> &params, const std::string &parameterName)
{
    auto it = std::find_if(params.begin(), params.end(), [&parameterName](const parameter &param) {
        return param.get_name() == parameterName;
    });
    if (it != params.end())
    {
        return &*it;
    }
    return nullptr;
}

parameter *controller::getParameter(std::vector<parameter> &params, const std::string &parameterName)
{
    return const_cast<parameter *>(getParameter(const_cast<const std::vector<parameter> &>(params), parameterName));
}

StaticModuleInfo::StaticModuleInfo(const std::string &name, const std::string &category)
    : name(name), category(category) {}

const ModuleNetConnectivity StaticModuleInfo::connectivity() const
{
    return m_connectivity;
}

const std::string &StaticModuleInfo::description() const
{
    return m_description;
}

void StaticModuleInfo::readConnectivity(const char *data)
{
    m_connectivity.interfaces.clear();
    m_connectivity.inputParams.clear();
    m_connectivity.outputParams.clear();
    auto list = splitString(data, "\n");
    int iel = 3;
    m_description = list[iel++];
    std::array<int, 4> interfaceAndParamCounts; //in_interface, out_interface, in_param, out_param
    for (size_t i = 0; i < 4; i++)
    {
        interfaceAndParamCounts[i] = std::stoi(list[iel++]);
    }
    for (size_t i = 0; i < interfaceAndParamCounts[0]; i++) // read the input -interfaces
    {
        m_connectivity.addInterface(list[iel], list[iel + 1], Direction::Input, list[iel + 2], list[iel + 3]);
        iel += 4;
    }
    for (size_t i = 0; i < interfaceAndParamCounts[1]; i++) // read the output -interfaces
    {
        m_connectivity.addInterface(list[iel], list[iel + 1], Direction::Output, list[iel + 2], list[iel + 3]);
        iel += 4;
    }
    for (size_t i = 0; i < interfaceAndParamCounts[2]; i++) // read the output -interfaces
    {
        m_connectivity.addParameter(list[iel], list[iel + 1], list[iel + 2], list[iel + 3], list[iel + 4], Direction::Input);
        iel += 5;
    }
    for (size_t i = 0; i < interfaceAndParamCounts[3]; i++) // read the output -interfaces
    {
        m_connectivity.addParameter(list[iel], list[iel + 1], list[iel + 2], list[iel + 3], list[iel + 4], Direction::Output);
        iel += 5;
    }
}

bool StaticModuleInfo::operator==(const StaticModuleInfo &other) const
{
    return name ==other.name;
}

bool StaticModuleInfo::operator<(const StaticModuleInfo &other) const
{
    return name <other.name;
}


size_t controller::Module::moduleCount = 0;

Module::Module(Type t, const RemoteHost &h, sender_type type, const StaticModuleInfo &moduleInfo)
    : host(h), type(type), m_info(moduleInfo), id(moduleCount++), m_type(t)
{
}

Module::~Module()
{
    CTRLGlobal::getInstance()->controller->getConnectionList()->remove(m_conn);
}

void Module::resetId(){
    moduleCount = 0;
}

const Connection *Module::conn() const
{
    return &*m_conn;
}

void Module::recv_msg(Message *msg) const
{
    if (m_conn)
    {
        m_conn->recv_msg(msg);
    }
};

bool Module::sendMessage(const Message *msg) const
{
    if (m_conn)
        return m_conn->sendMessage(msg);
    return false;
}

bool Module::sendMessage(const UdpMessage *msg) const
{
    return false;
}

bool Module::connect(const Module &crb)
{
    return connect(crb, COVISE_MESSAGE_APP_CONTACT_DM);
}

bool Module::connect(const Module &crb, covise_msg_type type)
{
    if (&crb == this)
    {
        std::cerr << "can not connect module to itself" << std::endl;
        return false;
    }
    // Tell CRB to open a socket for the module
    Message msg{COVISE_MESSAGE_PREPARE_CONTACT, DataHandle{}};
    print_comment(__LINE__, __FILE__, "before PREPARE_CONTACT send");
    if (!crb.send(&msg))
        return false;

    // Wait for CRB to deliver the opened port number
    do
    {
        std::unique_ptr<Message> portmsg{new Message{}};
        crb.recv_msg(&*portmsg);
        print_comment(__LINE__, __FILE__, "PORT received");
        if (portmsg->type == COVISE_MESSAGE_PORT)
        {
            // copy port number to message buffer:
            // can be binary, since CRB and Module are on same machine
            DataHandle msg_data{80};
            memcpy(msg_data.accessData(), &portmsg->data.data()[0], sizeof(int));

            // copy adress of CRB host in dot notation into message
            strncpy(&msg_data.accessData()[sizeof(int)], crb.host.userInfo().ipAdress.c_str(), 76);

            // send to module
            msg_data.setLength(sizeof(int) + (int)strlen(&msg_data.data()[sizeof(int)] + 1));
            msg = Message{type, msg_data};
            print_comment(__LINE__, __FILE__, "vor APP_CONTACT_DM send");
            send(&msg);
            return true;
        }
        else
        {
            CTRLHandler::instance()->handleMsg(portmsg); // handle all other messages
        }

    } while (true);
}

bool Module::setupConn(std::function<bool(int)> sendConnMessage)
{
    int port = 0;
    auto conn = createListeningConn<ServerConnection>(&port, id, (int)CONTROLLER);
    if (sendConnMessage(port))
    {
        int timeout = 0; // do not timeout
        if (conn->acceptOne(timeout) < 0)
        {
            cerr << "* timelimit in accept for module " << info().name << " exceeded!!" << endl;
            return false;
        }
        conn->set_peer(id, type);
        m_conn = covise::Process::this_process->getConnectionList()->add(std::move(conn));
        return true;
    }
    return false;
}

bool Module::start(const char *instance)
{
    setupConn([this, instance](int port) {
        auto &controllerHost = host.hostManager.getLocalHost();
        CRB_EXEC crbExec{covise::ExecFlag::Normal, info().name.c_str(), port, controllerHost.userInfo().ipAdress.c_str(), static_cast<int>(id), instance,
                         host.userInfo().ipAdress.c_str(), host.userInfo().userName.c_str(),
                         nullptr, host.ID(), vrb::VrbCredentials{}, std::vector<std::string>{}};

        host.launchProcess(crbExec);
        return true;
    });
}

const StaticModuleInfo &Module::info() const
{
    return m_info;
}
