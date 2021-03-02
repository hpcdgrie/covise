/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "userinterface.h"
#include "host.h"
#include "global.h"
#include "handler.h"
#include "crb.h"

#include <config/CoviseConfig.h>
#include <config/coConfig.h>

using namespace covise::controller;

Userinterface::Userinterface(const RemoteHost &h, const StaticModuleInfo &info)
    : Module(moduleType, h, sender_type::USERINTERFACE, info)
    , m_status(Slave)
{
}

Userinterface::~Userinterface()
{
    Message ui_msg{COVISE_MESSAGE_QUIT, ""};
    send(&ui_msg);
}

const char *Userinterface::getStatusName(Status s)
{
    return statusNames[static_cast<int>(s)];
}

bool Userinterface::restart(const UIOptions &options)
{
    if (!m_crb)
    {
        std::cerr << "failed to restart ui: ui has not been started yet" << endl;
        return false;
    }

    start(options, *m_crb, true);

    // send current net to UIF
    // send current controller information to ui
    Message msg{COVISE_MESSAGE_UI, "START_READING\n"};
    send(&msg);

    for(const Application *mod : host.hostManager.getAllModules<Application>())
    {
        cerr << mod->info().name << endl;
        ostringstream mybuf;
        mybuf << "INIT\n"
                << mod->createBasicModuleDescription()
                << mod->pos().x << "\n"
                << mod->pos().y << "\n";
        msg = Message{COVISE_MESSAGE_UI, mybuf.str()};
        send(&msg);

        ostringstream os;
        os << "DESC\n";
        os << mod->createDescription();
        msg = Message{COVISE_MESSAGE_UI, os.str()};
        send(&msg);

        ostringstream oss;
        oss << "MODULE_TITLE\n"
            << mod->createBasicModuleDescription()
            << mod->title() << "\n";
        msg = Message{COVISE_MESSAGE_UI, oss.str()};
        send(&msg);

        // send current parameter
        // only input parameter



        // loop over all input parameters
        for(const parameter& param: mod->connectivity().inputParams)
        {
            std::string value = param.get_val_list();
            if (param.get_type() == "Browser")
            {
                CTRLHandler::instance()->handleBrowserPath(mod->info().name, std::to_string(mod->instance()), mod->host.userInfo().hostName,
                                                           mod->host.userInfo().hostName, param.get_name(), value);
            }
            std::stringstream ss;
            ss << "PARAM_RESTART\n"
               << mod->createBasicModuleDescription() << param.get_name() << "\n"
               << param.get_type() << "\n"
               << value;
            msg = Message{COVISE_MESSAGE_UI, ss.str()};
            send(&msg);
            ss = std::stringstream{};
            ss << "ADD_PANEL\n"
               << mod->createBasicModuleDescription() << param.get_name() << "\n"
               << param.get_type() << "\n"
               << param.get_addvalue();
            msg = Message{COVISE_MESSAGE_UI, ss.str()};
            host.hostManager.sendAll<Userinterface>(msg);
        }
    }

    // send connection informations
    object *tmp_obj;

    CTRLGlobal::getInstance()->objectList->reset();
    while ((tmp_obj = CTRLGlobal::getInstance()->objectList->next()) != NULL)
    {
        int i = 0;
        string buffer = tmp_obj->get_simple_connection(&i);
        if (!buffer.empty() || i != 0)
        {
            ostringstream mybuf2;
            mybuf2 << "OBJCONN2\n"
                    << i << "\n"
                    << buffer;
            msg = Message{COVISE_MESSAGE_UI, mybuf2.str()};
            send(&msg);
        }
    }

    // send end message to UIF
    msg = Message{COVISE_MESSAGE_UI, "END_READING\ntrue"};
    send(&msg);

    return true;
}


void Userinterface::setStatus(Status status)
{
    m_status = status;
}

Userinterface::Status Userinterface::status() const
{
    return m_status;
};

void Userinterface::changeStatus(Status status)
{
    setStatus(status);
    Message msg{COVISE_MESSAGE_UI, statusNames[m_status]};
    send(&msg);
}

void Userinterface::changeMaster(const RemoteHost &master)
{
    string text = "MASTERREQ\n" + master.userInfo().userName + "\n" + master.userInfo().ipAdress + "\n\n";
    Message msg{COVISE_MESSAGE_UI, text};
    send(&msg);
}


bool Userinterface::updateUI(){
for (const CRBModule *remoteCrb : host.hostManager.getAllModules<CRBModule>())
{
    if (remoteCrb->host.state() != LaunchStyle::Disconnect)
    {
        Message ui_msg{remoteCrb->initMessage};
        ui_msg.type = COVISE_MESSAGE_UI;
        send(&ui_msg);
    }
}
}

StaticModuleInfo MapEditor::uiMapEditorInfo{"mapeditor", "graphical userinterface with map editor"};


MapEditor::MapEditor(const RemoteHost &h)
    : Userinterface(h, uiMapEditorInfo)
{
}

bool MapEditor::start(const UIOptions &options, const CRBModule &crb, bool restart) // if restart is true a restart was done
{
    m_crb = &crb;
    if (!Module::start("001"))
        return false;
    if (!connect(crb))
        return false;

    // send status-Message/
    string tmp = statusNames[m_status];
    if (restart)
        tmp.append("_RESTART");

    if (options.type == UIOptions::Type::miniGui)
        tmp.append("\nMINI_GUI");

    Message msg{COVISE_MESSAGE_UI, tmp};
    send(&msg);

    updateUI();

    // wait for OK from Mapeditor
    msg = Message{};
    recv_msg(&msg);

    if (msg.type == COVISE_MESSAGE_MSG_OK && msg.data.data())
    {
        msg.type = COVISE_MESSAGE_UI;
        crb.send(&msg); //message for CRB that an embedded renderer is possible
        return 1;
    }
    if (options.iconify)
    {
        Message msg{COVISE_MESSAGE_UI, "ICONIFY"};
        send(&msg);
    }

    if (options.maximize)
    {
#ifndef _AIRBUS
        Message msg{COVISE_MESSAGE_UI, "MAXIMIZE"};
#else
        Message msg{COVISE_MESSAGE_UI, "PREPARE_CSCW"};
#endif
        send(&msg);
    }
    return 0;
}

StaticModuleInfo WsInterface::wsInterfaceInfo{"wsinterface", "web-service interface"};


WsInterface::WsInterface(const RemoteHost& host)
    : Userinterface(host, wsInterfaceInfo)
{
}

bool WsInterface::start(const UIOptions &options, const CRBModule &crb, bool restart)
{
    m_crb = &crb;
    //Determine from config whether to use WSInterface
    bool ws_enabled = covise::coConfig::getInstance()->getBool("System.WSInterface", true);
    if (ws_enabled)
    {
        const char *instance = "ws001";
        Module::start(instance);
    }
    else
    {
        return false;
    }

    if (!connect(crb))
        return 0;

    // send status-Message
    string tmp = statusNames[m_status];
    tmp.append("\n");
    Message msg{COVISE_MESSAGE_UI, tmp};
    send(&msg);

    updateUI();

    // wait for OK from Mapeditor
    msg = Message{};
    recv_msg(&msg);

    if (msg.type == COVISE_MESSAGE_MSG_OK)
    {
        return 1;
    }
    return 0;
}


PythonInterface::PythonInterface(const RemoteHost& host, const StaticModuleInfo &info)
:Userinterface(host, info)
{
}

bool PythonInterface::start(const UIOptions &options, const CRBModule &crb, bool restart){
    m_crb = &crb;
    string instanz("001");

    if (!Module::start("001"))
        return false;
    if (!connect(crb))
        return false;


    // send status-Message
    string tmp = statusNames[m_status];
    Message msg{COVISE_MESSAGE_UI, tmp};
    send(&msg);
    updateUI();

    // wait for OK from Mapeditor

    msg = Message{};
    recv_msg(&msg);

    if (msg.type == COVISE_MESSAGE_MSG_OK)
    {
        rendererIsPossible = false;
        if (msg.data.data() && !strcmp(msg.data.data(), "RENDERER_INSIDE_OK"))
        {
            rendererIsPossible = true;
        }
        return true;
    }
    return false;
}