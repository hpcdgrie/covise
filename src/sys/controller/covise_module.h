/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef EC_MODULE_H
#define EC_MODULE_H

#include <covise/covise.h>
#include <covise/covise_msg.h>
#include <net/covise_connect.h>
#include <net/message_sender_interface.h>
namespace covise
{
namespace controller{

struct RemoteHost;

struct Module : MessageSenderInterface{
    Module(const RemoteHost&host, sender_type t, const std::string &name);
    virtual ~Module() = default;
    const size_t id; 
    const sender_type type;
    const std::string name;
    const RemoteHost &host;
    template <typename T>
    const T *as()
    {
        if (type == T::type)
        {
            return dynamic_cast<T>(this);
        }
        return nullptr;
    }
    void setConn(std::unique_ptr<Connection> &&conn);
    const Connection *conn();
    void recv_msg(Message *msg);
    static ConnectionList connectionList;

protected:
    virtual bool sendMessage(const Message *msg) override;
    virtual bool sendMessage(const UdpMessage *msg) override;
private:
    std::unique_ptr<Connection> m_conn; // connection to this other module
    static size_t moduleCount;
};
} // namespace controller
class AppModule : public MessageSenderInterface// class that represents other modules or processes
{
private:
    Connection *conn; // connection to this other module
    Host *host; // machine where this other module runs
    string name; // name of this other module
    int id; // id of this other module
    int hostid; // id of host

public:
    string covise_path; // COVISE_PATH on host

    // complete initialization
    AppModule(Connection *c, int i, const string &n, Host *h)
    {
        conn = c;
        id = i;
        name = n;
        host = h;
        hostid = 0;
    }

    // complete initialization
    AppModule(Connection *c, int i, const string &n, Host *h, int hid)
    {
        conn = c;
        id = i;
        name = n;
        hostid = hid;
        conn->set_hostid(hid);
        host = h;
    }

    ~AppModule() // destructor
    {
        if (conn)
            delete conn;
    };

    // set peer id - @@@@@:
    void set_peer(int peer_id, sender_type peer_type);

    Connection *get_conn()
    {
        return conn;
    }
private:
    // forward message to the connection:
    bool sendMessage(const Message *msg) override;
    bool sendMessage(const UdpMessage *msg) override;

public:
    // get message from the connection
    int recv_msg(Message *msg);

    // establishes connection between 2 processes
    int connect(AppModule *m);

    // same as above, but the process can wait for the answer (PORT) itselve
    int prepare_connect(AppModule *m);

    int do_connect(AppModule *m, Message *portmsg);

    int connect_datamanager(AppModule *m);

    // get host
    const Host *get_host() const
    {
        return host;
    };

    // get module id
    int get_id() const
    {
        return id;
    };

    // get module name
    const string &get_name() const
    {
        return name;
    };

    // get host id
    int get_hostid() const
    {
        return hostid;
    };

    // set host id
    void set_hostid(int hid)
    {
        conn->set_hostid(hid);
    };

    int get_socket_id(void (*remove_func)(int))
    { // give socket id
        return conn->get_id(remove_func);
    };
};
}
#endif

// establishes connection between 2 datamanagers
