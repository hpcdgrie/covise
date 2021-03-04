/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef CONTROLLER_RENDER_MODULE_H
#define CONTROLLER_RENDER_MODULE_H

#include "module.h"
#include "userinterface.h"
namespace covise
{
namespace controller
{

class Display : public SubProcess
{
    static const SubProcess::Type moduleType = SubProcess::Type::Display;
    controller::Userinterface::Status excovise_status = Userinterface::Status::Init;
    bool DISPLAY_READY = false;
    bool NEXT_DEL = false;
    string DO_name;
    int m_helper = 0;
    const Renderer &m_renderer;

public:
    Display(controller::Renderer &renderer, const controller::RemoteHost &ui);
    using SubProcess::connect;
    using SubProcess::m_info;
    void set_helper(int hlp)
    {
        m_helper = hlp;
    };
    int is_helper(void)
    {
        return m_helper;
    };

    void set_execstat(Userinterface::Status status);
    controller::Userinterface::Status get_execstat() const
    {
        return excovise_status;
    };

    void set_DISPLAY(bool bvar);
    bool get_DISPLAY();
    bool get_NEXT_DEL();

    void quit();

    void send_add(const string &DO_name);
    void send_add();
    void send_del(const string &DO_name, const string &DO_new_name);

    void send_status(const string &info_str);
    void send_message(Message *msg);
};

struct Renderer : Application
{
    Renderer(const RemoteHost &host, const ModuleInfo &moduleInfo, int instance);
    virtual void exec(NumRunning &numRunning) override;
    virtual std::string serializeInputInterface(const net_interface &interface) const override;
    virtual void init(const MapPosition &pos, int copy, ExecFlag flag, Application *mirror) override;
    ModuleInfo &getInfo();
    typedef std::vector<std::unique_ptr<Display>> DisplayList;
    DisplayList::iterator begin();
    DisplayList::const_iterator begin() const;

    DisplayList::iterator end();
    DisplayList::const_iterator end() const;

    DisplayList::iterator getDisplay(int moduleID);
    DisplayList::const_iterator getDisplay(int moduleID) const;

    void removeDisplay(DisplayList::iterator display);
    size_t numDisplays() const;
    DisplayList::iterator addDisplayAndHandleConnections(const Userinterface &ui);
    void send_add(const object &obj, obj_conn &connection);
    void send_add_obj(const string &name);

    bool update(int moduleID, NumRunning &numRunning);
    bool isMirrorOf(int ModuleID) const;
    void setSenderStatus();
    void send_del(const std::string &name);
    virtual void execute(NumRunning &numRunning) override;

private:
    DisplayList::iterator addDisplay(const Userinterface &ui);
    DisplayList m_displays;
    mutable int m_ready = 0;
    bool initDisplays(int copy);
};

} // namespace controller
} // namespace covise

#endif // !CONTROLLER_RENDER_MODULE_H