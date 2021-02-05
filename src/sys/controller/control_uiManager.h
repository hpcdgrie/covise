#ifndef CONTROL_UI_MANAGER_H
#define CONTROL_UI_MANAGER_H

#include "control_modlist.h"

#include <vector>
#include <string>

namespace covise
{

namespace controller
{
struct UIOptions{
    enum Type
    {
        python,
        gui,
        miniGui,
        nogui
    }type;
    std::string pyFile;
    bool iconify = false;
    bool maximize = false;
};

class UIManager
{
public:

    UIManager();
    void init(const UIOptions &options);
    void startUI();

private:
    typedef std::vector<std::shared_ptr<userinterface>> UiList;
    UiList m_uis;
    UiList m_localUis;
    bool m_iconify = false;
    bool m_maximize = false;
    UIOptions m_options;
    bool start_local_Mapeditor();
    bool start_local_WebService();
    bool start_local_pythonInterface();
    void setLocalUIParams(userinterface &localUi);
    void addLocalUi(const UiList::value_type &localUi);



};
} // namespace controller
} // namespace covise

#endif