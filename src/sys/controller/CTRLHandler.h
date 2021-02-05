/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef CTRL_CTRL_H
#define CTRL_CTRL_H

#include "control_remoteHost.h"
#include "control_uiManager.h"
#include "control_moduleManager.h"

#include <QMap>
#include <QStringList>
#include "CTRLGlobal.h"
#include <vrb/client/VRBClient.h>
#include <string>
class QString;
namespace boost{
namespace program_options{
class variables_map;
}
}
namespace covise
{

class net_module;
class Message;
class ControlConfig;
class AccessGridDaemon;
class AppModule;
class SSLClient;
class NEW_UI;

// == == == == == == == == == == == == == == == == == == == == == == == ==
namespace controller{
class CTRLHandler
// == == == == == == == == == == == == == == == == == == == == == == == ==
{
public:
    enum readMode
    {
        NETWORKMAP,
        CLIPBOARD,
        UNDO
    };

    CTRLHandler(int argc, char **argv);

    bool m_miniGUI = false, m_rgbTextOpen = false;
    int m_numRunning = 0, m_numRendererRunning = 0;

    ControlConfig *Config;

    static CTRLHandler *instance();
    void handleClosedMsg(Message *msg);
    void handleAndDeleteMsg(Message *msg);
    string handleBrowserPath(const string &name, const string &nr, const string &host, const string &oldhost,
                             const string &parameterName, const string &parameterValue);

    vector<string> splitString(string text, const string &sep);
    bool recreate(string buffer, readMode mode);

private:
    static CTRLHandler *singleton;
    FILE *fp;
    string m_globalFilename, m_netfile, m_clipboardBuffer, m_collaborationRoom;
    string m_localUser, m_scriptName, m_filename, m_filePartnerHost, m_pyFile;
struct CommandLineOptions{
    std::string netFile;
    std::string crbLaunchScript;
    std::string localUserName;
    bool quit = false; //m_quitAfterExececute
    bool execute = false; //m_executeOnLoad
    bool isLoaded = true;
    UIOptions uiOptions;

} m_options;
bool m_globalLoadReady = true, m_clipboardReady = true;
bool m_readConfig = true; /* all Mapeditors started */
bool m_useGUI = true;     /* use an user interface */
bool m_isLoaded = true, m_executeOnLoad = false, m_iconify = false, m_maximize = false;
bool m_quitAfterExececute = false;
bool m_xuif = false, m_startScript = false;
SSLClient *m_SSLClient;
bool m_clientRegistered = false;
HostManager m_remoteHostManager;
UIManager m_uiManager;
ModuleManager m_moduleManager;
void lookupSiblings();
void loop();
void parseCommandLine(int argc, char **argv);
void startCrbUiAndDatamanager();
void loadNetworkFile();
void handleQuit(Message *msg);
void handleUI(Message *msg, string data);
void handleNewUi(const NEW_UI &msg);
void handleFinall(Message *msg, string data);
void delModuleNode(vector<net_module *> liste);
int initModuleNode(const string &name, const string &nr, const string &host, int, int, const string &, int, ExecFlag flags);
void makeConnection(const string &from_mod, const string &from_nr, const string &from_host, const string &from_port,
                    const string &to_mod, const string &to_nr, const string &to_host, const string &to_port);
void sendNewParam(const string &name, const string &nr, const string &host,
                  const string &parname, const string &partype, const string &parvalue, const string &apptype,
                  const string &oldhost, bool init = false);
bool checkModule(const string &modname, const string &modhost);
bool waitForVrbRegistration();
bool vrbUpdate();
void vrbClientsUpdate(const Message &userInfoMessage);
void getAllConnections();
void resetLists();
string writeClipboard(const string &keyword, vector<net_module *> liste, bool all = false);
void addBuffer(const QString &text);
void sendCollaborativeState();
void addHost(const vrb::RemoteClient &rc);
void addPartner(const vrb::RemoteClient &rc);
void removeClient(const vrb::RemoteClient &rc);

std::string m_autosavefile;

bool m_writeUndoBuffer = true;
QStringList m_qbuffer;
QStringList m_undoBuffer;

int m_nconn;
vector<string> from_name, from_inst, from_host, from_port;
vector<string> to_name, to_inst, to_host, to_port;
typedef std::list<std::pair<std::string, std::string>> slist;

slist siblings;
};
}
}//controller
#endif
