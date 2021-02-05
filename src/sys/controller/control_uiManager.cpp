#include "control_uiManager.h"
#include <CTRLGlobal.h>
using namespace covise::controller;

UIManager::UIManager()
{
}

void UIManager::init(const UIOptions& options)
{
    cerr << "* Starting user interface....                                                 *" << endl;
    switch (options.type)
    {
    case UIOptions::python:
        start_local_pythonInterface();
        break;
    case UIOptions::gui:
        start_local_Mapeditor();
#ifdef HAVE_GSOAP
        start_local_WebService();
#endif
        break;

    default:
        break;
    }
}

bool UIManager::start_local_Mapeditor()
{
    auto local = UiList::value_type(new UIMapEditor);
    setLocalUIParams(*local);
    if (local->start(false))
    {
        m_localUis.push_back(local);
        if (m_options.iconify)
        {
            Message msg{COVISE_MESSAGE_UI, "ICONIFY"};
            local->send(&msg);
        }

        if (m_options.maximize)
        {
#ifndef _AIRBUS
            Message msg{COVISE_MESSAGE_UI, "MAXIMIZE"};
#else
            Message msg{COVISE_MESSAGE_UI, "PREPARE_CSCW"};
#endif
            local->send(&msg);
        }
        return true;
    }
    return false;
}

bool UIManager::start_local_WebService()
{
    auto local = UiList::value_type(new UISoap);
    setLocalUIParams(*local);

    if (local->start(false))
    {
        addLocalUi(local);
        return true;
    }
    return false;
}

// test by RM
bool UIManager::start_local_pythonInterface()
{
    auto local = UiList::value_type(new UIMapEditor);
    setLocalUIParams(*local);
    if (local->xstart(m_options.pyFile))
    {
        addLocalUi(local);
        return true;
    }
    return false;
}

void UIManager::setLocalUIParams(userinterface &localUi)
{
    DM_data *tmp_dm = CTRLGlobal::getInstance()->dataManagerList->get_local();
    string local_name = tmp_dm->get_hostname();
    string local_user = tmp_dm->get_user();
    localUi.set_host(local_name);
    localUi.set_userid(local_user);
    localUi.set_status("MASTER");
}

void UIManager::addLocalUi(const UiList::value_type &localUi)
{
    m_uis.push_back(localUi);
    m_localUis.push_back(localUi);
}