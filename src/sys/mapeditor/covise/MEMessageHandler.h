/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef ME_MESSAGEHANDLER_H
#define ME_MESSAGEHANDLER_H

#include <QObject>
#include <net/message_sender_interface.h>
class QTimer;

namespace covise
{
class Message;
class NEW_UI;
class UdpMessage;
class UserInterface;
typedef std::vector<std::pair<int, std::string>> ClientList;
} // namespace covise

class QStringList;

class MENode;
class MEUserInterface;
class MEMainHandler;

//================================================
class MEMessageHandler : public QObject, public covise::MessageSenderInterface
//================================================
{
    Q_OBJECT
public:
    MEMessageHandler(int argc, char **argv);
    ~MEMessageHandler();

    static MEMessageHandler *instance();

    bool isStandalone()
    {
        return m_standalone;
    };
    void sendMessage(int, const QString &);
    covise::UserInterface *getUIF()
    {
        return m_userInterface;
    };

public slots:

    void dataReceived(int);
    void handleWork();
protected:
    virtual bool sendMessage(const covise::Message *msg) override;
    virtual bool sendMessage(const covise::UdpMessage *msg) override;
private:
    static MEMessageHandler *singleton;

    bool m_standalone;
    MENode *m_currentNode, *m_clonedNode;
    covise::UserInterface *m_userInterface;
    QTimer *m_periodictimer;

    void receiveUIMessage(covise::Message *);
    void receiveUIMessage(const covise::NEW_UI&);
};
#endif
