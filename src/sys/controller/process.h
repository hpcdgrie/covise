/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef CONTROLLER_PROCESS_H
#define CONTROLLER_PROCESS_H

#include "port.h"

#include <covise/covise.h>
#include <covise/covise_msg.h>
#include <net/covise_connect.h>
#include <net/message_sender_interface.h>
#include <functional>
namespace covise
{
namespace controller{

struct RemoteHost;

struct ModuleNetConnectivity
{
    std::vector<C_interface> interfaces;
    std::vector<parameter> inputParams, outputParams;
    void addInterface(const string &name, const string &type, controller::Direction direction, const string &text, const string &demand);
    void addParameter(const string &name, const string &type, const string &text, const string &value, const string ext, Direction dir);

    const parameter &getParam(const std::string& paramName) const;
    parameter &getParam(const std::string& paramName);

    const net_interface &getInterface(const std::string &interfaceName) const;
    net_interface &getInterface(const std::string &interfaceName);

    void forAllNetInterfaces(const std::function < void(net_interface &)> &func);
    void forAllNetInterfaces(const std::function < void(const net_interface &)> &func) const;
};

const parameter *getParameter(const std::vector<parameter> &params, const std::string &parameterName);
parameter *getParameter(std::vector<parameter> &params, const std::string &parameterName);

//name, category and connectivity info of a module class
struct StaticModuleInfo
{
    StaticModuleInfo(const std::string &name, const std::string &category);
    const std::string name, category;
    mutable size_t count = 0;
    const ModuleNetConnectivity connectivity() const;
    const std::string &description() const;
    void readConnectivity(const char *buff);
    bool operator==(const StaticModuleInfo &other) const;
    bool operator<(const StaticModuleInfo &other) const;


private:
    std::string m_description;
    ModuleNetConnectivity m_connectivity;
};

//representation of a connected covice process (crbs, uis and modules)
//each host has exacly one CRBModule (if it is connected) and up to one Userinterface
struct Module : MessageSenderInterface
{
    enum class Type
    {
        Crb,
        UI,
        App,
        Display
    };
    Module(Type t, const RemoteHost&host, sender_type type, const StaticModuleInfo &moduleInfo);
    virtual ~Module();
    Module(const Module &) = delete;
    Module(Module &&) = default;
    Module &operator=(const Module &) = delete;
    Module &operator=(Module &&) = default;


    static void resetId(); //reset global module id
    const size_t id; //global id, incremented for every module created -> moduleCount
    const sender_type type;
    const RemoteHost &host;
    template <typename T>
    const T *as() const
    {
        return const_cast<Module *>(this)->as<T>();
    }

    template <typename T>
    T *as()
    {
        if (m_type == T::moduleType)
        {
            return dynamic_cast<T*>(this);
        }
        return nullptr;
    }

    bool setupConn(std::function<bool(int)> sendConnMessage);

    const Connection *conn() const;
    void recv_msg(Message *msg) const;
    bool start(const char* instance);
    const StaticModuleInfo &info() const;
    bool connect(const Module &crb);
    
protected:
    virtual bool sendMessage(const Message *msg) const override;
    virtual bool sendMessage(const UdpMessage *msg) const override;
    bool connect(const Module &crb, covise_msg_type type);
    const Connection *m_conn; // connection to this other module managed by process::list_of_connections
    StaticModuleInfo m_info;
private:
    const Type m_type;
    static size_t moduleCount;
    int port = 0;
};
} // namespace controller
} //covise
#endif

