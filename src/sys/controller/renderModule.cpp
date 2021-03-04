
/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "renderModule.h"
#include "userinterface.h"
#include "handler.h"
#include "exception.h"

using namespace covise::controller;

//**********************************************************************
//
// 			DISPLAY
//
//**********************************************************************

Display::Display(Renderer &renderer, const controller::RemoteHost &host)
    : SubProcess(moduleType, host, renderer.type, renderer.info().name), m_renderer(renderer)
{
}

void Display::set_execstat(Userinterface::Status status)
{
    excovise_status = status;
}

void Display::set_DISPLAY(bool bvar)
{
    DISPLAY_READY = bvar;
}

bool Display::get_DISPLAY()
{
    return DISPLAY_READY;
}

#ifdef NEW_RENDER_MSG

void display::send_status(const string &info_str)
{
    string text = "STATUS\n" + excovise_status + "\n" + info_str + "\n";
    Message *msg = new Message(COVISE_MESSAGE_RENDER, text);
    applmod->send_msg(msg);

#ifdef DEBUG
//	fprintf(msg_prot, "---------------------------------------------------\n");
//	fprintf(msg_prot, "send displaystatus\n%i %i \n %s \n", msg->sender, msg->type, msg->data);
//	fflush(msg_prot);
#endif

    delete msg;
}

#else

void Display::send_status(const string &info_str)
{
    string text = excovise_status + "\n" + info_str + "\n";
    Message msg{COVISE_MESSAGE_RENDER, text};
    send(&msg);
}

#endif

void Display::quit()
{
#ifdef QUITMOD
    Message msg{COVISE_MESSAGE_QUIT, ""};
    send(&msg);
#endif
}

bool Display::get_NEXT_DEL()
{
    return NEXT_DEL;
}

void Display::send_add(const string &DO_name)
{
    if (DO_name.empty())
        return;

    CTRLHandler::instance()->numRunning().apps++;
    CTRLHandler::instance()->numRunning().renderer++;

    NEXT_DEL = true;

#ifdef NEW_RENDER_MSG
    string tmp = "ADD\n" + DO_name + "\n";
    Message msg{COVISE_MESSAGE_RENDER, tmp};
#else
    string tmp = DO_name;
    Message msg{COVISE_MESSAGE_ADD_OBJECT, tmp};
#endif

    if (!is_helper())
        m_renderer.send(&msg);
}

void Display::send_add()
{
    send_add(DO_name);
}

void Display::send_del(const string &DO_old_name, const string &DO_new_name)
{
    CTRLHandler::instance()->numRunning().apps++;
    CTRLHandler::instance()->numRunning().renderer++;

    NEXT_DEL = true; // DEL or REPLACE possible after REPLACE

    // test, if new-name is an empty string -> only DELETE
    if (DO_new_name.empty())
    {
        NEXT_DEL = true;
        DO_name = DO_new_name;

        if (!DO_old_name.empty())
        {

#ifdef NEW_RENDER_MSG
            string text = "DEL\n" + DO_old_name + "\n";
            Message msg{COVISE_MESSAGE_RENDER, text};
#else
            Message msg{COVISE_MESSAGE_DELETE_OBJECT, DO_old_name};
#endif
            if (!is_helper())
                m_renderer.send(&msg);
        }

#ifdef DEBUG
        fprintf(msg_prot, "---------------------------------------------------\n");
        fprintf(msg_prot, "send DEL_Obj to \n%i %s \n", this->get_mod_id(), msg.data);
        fflush(msg_prot);
#endif
    }

    else
    {
        // copy new-name to DO_name
        DO_name = DO_new_name;

        // create Msg-string
        string text = DO_old_name + "\n" + DO_new_name + "\n";
        Message msg{COVISE_MESSAGE_REPLACE_OBJECT, text};

        if (!is_helper())
            m_renderer.send(&msg);

#ifdef DEBUG
        fprintf(msg_prot, "---------------------------------------------------\n");
        fprintf(msg_prot, "send DEL_Obj to \n%i %s \n", this->get_mod_id(), msg.data);
        fflush(msg_prot);
#endif
    }

    // else REPLACE
}

void Display::send_message(Message *msg)
{
    if (!is_helper())
        m_renderer.send(msg);
}

Renderer::Renderer(const RemoteHost &host, const ModuleInfo &moduleInfo, int instance)
    : NetModule(host, moduleInfo, instance)
{
    assert(moduleInfo.category == "Renderer");
}

void Renderer::exec(NumRunning &numrunning)
{
    (void)numrunning;
    Message err_msg{COVISE_MESSAGE_COVISE_ERROR, "Sorry, can't execute a Renderer!\n"};
    host.hostManager.sendAll<Userinterface>(err_msg);
}

std::string Renderer::serializeInputInterface(const net_interface &interface) const
{
    std::stringstream buff;
    buff << interface.get_name() << "\n";
    if (interface.get_conn_state())
    {
        buff << dynamic_cast<const render_interface &>(interface).get_objlist() << "\n";
    }
    else
        buff << "NOT CONNECTED>\n";
    return buff.str();
}

void Renderer::init(const MapPosition &pos, int copy, ExecFlag flag, NetModule *mirror)
{
    m_position = pos;

    //create_netlink
    if (copy == 4 && mirror)
    {
        this->mirror(mirror);
        try
        {
            host.getModule(sender_type::USERINTERFACE);
            auto &displays = dynamic_cast<Renderer *>(mirror)->m_displays;
            auto display = std::find_if(displays.begin(), displays.end(), [this](const DisplayList::value_type &d) {
                return &d->host == &host;
            });
            display->get()->quit();
            displays.erase(display);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    initDisplays(copy);
    initConnectivity();
}

bool containsDisplay(const std::vector<std::unique_ptr<Display>> &displays, const RemoteHost &host)
{
    return std::find_if(displays.begin(), displays.end(), [&host](const std::unique_ptr<Display> &dis) {
               return &dis->host == &host;
           }) != displays.end();
}

bool Renderer::initDisplays(int copy)
{
    const SubProcess *localUi = nullptr;
    try
    {
        localUi = &host.getModule(sender_type::USERINTERFACE);
    }
    catch (const Exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    if (localUi && copy != 4) // !MIRROR
    {
        for (const Userinterface *ui : host.hostManager.getAllModules<Userinterface>())
        {
            if (containsDisplay(m_displays, ui->host))
                continue;

            auto &newDisplay = *addDisplay(*ui);

            // status dem Rendermodule mitteilen
            Message msg;
            newDisplay->recv_msg(&msg);
            m_info.readConnectivity(msg.data.data());
            std::string info_str = info().name + "\n" + std::to_string(instance()) + "\n" + ui->host.userInfo().hostName;
            newDisplay->send_status(info_str);
        }
    }                               // end !MIRROR
    else if (!localUi || copy == 4) // no ui on this host or it's a mirror node
    {
        if (containsDisplay(m_displays, host))
            return true;

        Display &newDisplay = **m_displays.emplace(m_displays.end(), new Display{*this, host});
        newDisplay.set_execstat(Userinterface::Mirror);
        std::string info_str = info().name + "\n" + std::to_string(instance()) + "\n" + host.userInfo().hostName;
        if (!newDisplay.start(std::to_string(instance()).c_str(), info().category.c_str()))
        {
            return false;
        }
    }
    return 1;
}

Renderer::DisplayList::iterator Renderer::begin()
{
    return m_displays.begin();
}

Renderer::DisplayList::const_iterator Renderer::begin() const
{
    return m_displays.begin();
}

Renderer::DisplayList::iterator Renderer::end()
{
    return m_displays.end();
}

Renderer::DisplayList::const_iterator Renderer::end() const
{
    return m_displays.end();
}

Renderer::DisplayList::iterator Renderer::getDisplay(int moduleID)
{
    return std::find_if(m_displays.begin(), m_displays.end(), [moduleID](const Renderer::DisplayList::value_type &dp) {
        return dp->id == moduleID;
    });
}

Renderer::DisplayList::const_iterator Renderer::getDisplay(int moduleID) const
{
    return const_cast<Renderer *>(this)->getDisplay(moduleID);
}

void Renderer::removeDisplay(Renderer::DisplayList::iterator display)
{
    m_displays.erase(display);
}

size_t Renderer::numDisplays() const
{
    return m_displays.size();
}

Renderer::DisplayList::iterator Renderer::addDisplay(const Userinterface &ui)
{
    auto newDisplay = m_displays.emplace(m_displays.end(), new Display{*this, ui.host});
    newDisplay->get()->set_execstat(ui.status());

    auto &crb = ui.host.getModule(sender_type::CRB);
    newDisplay->get()->start(std::to_string(instance()).c_str(), info().category.c_str());
    newDisplay->get()->connect(host.getModule(sender_type::CRB));

    return newDisplay;
}

Renderer::DisplayList::iterator Renderer::addDisplayAndHandleConnections(const Userinterface &ui)
{
    auto displ = addDisplay(ui);
    bool exec = false;
    for (auto &interface : connectivity().interfaces)
    {
        if (auto renderInterface = dynamic_cast<render_interface *>(&interface))
        {
            auto connList = renderInterface->get_connects();
            connList->reset();
            while (auto conn = connList->next())
            {
                if (auto obj = conn->get_obj())
                {
                    if (!obj->isEmpty())
                    {
                        auto dataObjectName = obj->get_current_name();
                        --m_ready;
                        setExecuting(true);
                        ++m_numRunning;
                        displ->get()->send_add(dataObjectName);
                        exec = true;
                    }
                }
            }
        }
    }
    if (exec)
    {
        auto content = get_inparaobj();
        if (!content.empty())
        {
            Message msg{COVISE_MESSAGE_START, content};
            host.hostManager.sendAll<Userinterface>(msg);
        }
    }
    host.hostManager.m_slaveUpdate = exec;

    return displ;
}

bool Renderer::update(int moduleID, NumRunning &numRunning)
{
    auto display = getDisplay(moduleID);
    if (display != end())
    {
        display->get()->set_DISPLAY(true);
        ++m_ready;
        if (m_ready < m_displays.size())
        {
            m_ready = m_displays.size();
        }

        --numRunning.apps;
        --numRunning.renderer;

        if (m_ready == 0)
        {
            setExecuting(false);
            for (auto &dp : m_displays)
            {
                dp->set_DISPLAY(false);
            }
            std::string content = createBasicModuleDescription() + "0\n0\n";
            Message msg(COVISE_MESSAGE_FINISHED, content);
            host.hostManager.sendAll<Userinterface>(msg);
            return true;
        }
    }
    return false;
}

bool Renderer::isMirrorOf(int moduleID) const
{
    if (id == moduleID)
    {
        return false;
    }

    switch (m_mirror)
    {
    case NOT_MIRR:
        return false;

    case ORG_MIRR:
    {
        auto m = std::find_if(m_mirrors.begin(), m_mirrors.end(), [moduleID](const NetModule *app) {
            return app->id == moduleID;
        });
        return m != m_mirrors.end();
    }
    case CPY_MIRR:
    {
        const Renderer *org = dynamic_cast<Renderer *>(m_mirrors[0]);
        if (org->id == moduleID)
            return true;
        auto m = std::find_if(org->m_mirrors.begin(), org->m_mirrors.end(), [moduleID](const NetModule *app) {
            return app->id == moduleID;
        });
        return m != m_mirrors.end();
    }
    default:
    {
        return false;
    }
    }
}

void Renderer::setSenderStatus()
{
    for (const Userinterface *ui : host.hostManager.getAllModules<Userinterface>())
    {
        auto display = std::find_if(m_displays.begin(), m_displays.end(), [&ui](const DisplayList::value_type &disp) {
            return &disp->host == &ui->host;
        });
        if (display != m_displays.end())
        {
            display->get()->set_execstat(ui->status());
            ostringstream info_str;
            info_str << info().name << "\n"
                     << instance() << "\n"
                     << ui->host.userInfo().hostName << "\n";
            display->get()->send_status(info_str.str());
        }
    }
}

void Renderer::send_del(const std::string &name)
{
    for (auto &display : m_displays)
    {
        display->send_del(name, "");
    }
    m_ready -= m_displays.size();
}

void Renderer::execute(NumRunning &numRunning)
{
    m_status = Status::executing;
    m_errorsSentByModule.clear();
    // sende first data object with New to the renderers
    connect_obj *connection = nullptr;
    render_interface *renderInterface = nullptr;
    for (C_interface &interface : connectivity().interfaces)
    {
        if (renderInterface = dynamic_cast<render_interface *>(&interface))
        {
            if (renderInterface->get_direction() == controller::Direction::Input && renderInterface->get_state(this) == S_NEW)
            {
                connection = renderInterface->get_first_NEW(this);
                break;
            }
        }
    }
    assert(renderInterface);
    // get new Dataobject-name
    object *tmp_obj = connection->get_obj();

    // Namen des alten DO lesen
    string old_name = connection->get_oldname();

    // Namen des neuen DO lesen
    string DO_name = tmp_obj->get_current_name();

    if (!old_name.empty()) /// existent and not empty
        std::for_each(m_displays.begin(), m_displays.end(), [&old_name, &DO_name](DisplayList::value_type &d) { d->send_del(old_name, DO_name); });
    else // send name to the displays
        std::for_each(m_displays.begin(), m_displays.end(), [&DO_name](DisplayList::value_type &d) { d->send_add(DO_name); });

    // neuen Namen als alten schreiben
    connection->set_oldname(DO_name);

    // decrement wait-counter
    renderInterface->decr_wait();

    // setze DO auf OLD
    tmp_obj->change_NEW_to_OLD(this, renderInterface->get_name());

    // setze DISPLAY_READY in den Renderen auf false
    std::for_each(m_displays.begin(), m_displays.end(), [](DisplayList::value_type &d) { d->set_DISPLAY(false); });
    m_ready -= m_displays.size();

    string content = get_inparaobj();
    if (!content.empty())
    {
        Message msg{COVISE_MESSAGE_START, content};
        host.hostManager.sendAll<Userinterface>(msg);
    }
}

bool Renderer::sendMessage(const covise::Message *msg) const
{
    bool err = false;
    for (const auto &display : m_displays)
    {
        if (!display->send(msg))
        {
            err = true;
        }
    }
    return !err;
}

void Renderer::send_add(const object &obj, obj_conn &connection)
{
    m_ready -= m_displays.size();

    m_status = Status::executing;
    m_numRunning++;
    auto oldName = connection.get_old_name();
    if (!connection.get_old_name().empty())
    {
        for (auto &display : m_displays)
        {
            display->send_del(connection.get_old_name(), obj.get_current_name());
        }
    }
    else
    {
        for (auto &display : m_displays)
        {
            display->send_add(obj.get_current_name());
        }
    }
    connection.set_old_name(obj.get_current_name());
    auto content = get_inparaobj();
    if (!content.empty())
    {
        Message msg{COVISE_MESSAGE_START, content};
        host.hostManager.sendAll<Userinterface>(msg);
    }
}

void Renderer::send_add_obj(const string &name)
{
    for (auto &display : m_displays)
    {
        display->send_add(name);
    }
    m_ready -= m_displays.size();
}
