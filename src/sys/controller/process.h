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

//representation of a connected covice process (crbs, uis and modules)
//each host has exacly one CRBModule (if it is connected) and up to one Userinterface
struct SubProcess : MessageSenderInterface
{
    enum class Type
    {
        Crb,
        UI,
        App,
        Display
    };
    SubProcess(Type t, const RemoteHost&host, sender_type type, const std::string &executableName);
    virtual ~SubProcess();
    SubProcess(const SubProcess &) = delete;
    SubProcess(SubProcess &&) = default;
    SubProcess &operator=(const SubProcess &) = delete;
    SubProcess &operator=(SubProcess &&) = default;


    static void resetId(); //reset global module id
    const size_t id; //global id, incremented for every module created -> moduleCount
    const sender_type type;
    const RemoteHost &host;
    template <typename T>
    const T *as() const
    {
        return const_cast<SubProcess *>(this)->as<T>();
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
    bool start(const char* instance, const char* category = nullptr);
    bool connect(const SubProcess &crb);
    
protected:
    virtual bool sendMessage(const Message *msg) const override;
    virtual bool sendMessage(const UdpMessage *msg) const override;
    bool connect(const SubProcess &crb, covise_msg_type type);
    const Connection *m_conn = nullptr; // connection to this other module managed by process::list_of_connections
private:
    const Type m_type; //type use to safely upcast
    const std::string m_executableName;
    static size_t moduleCount; //global number of SubProcesses
    int port = 0;
};
} // namespace controller
} // namespace covise
#endif

