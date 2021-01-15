#include "MERemotePartner.h"
#include "ui_MERemotePartner.h"

#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>

ClientWidget::ClientWidget(int clientID, const QString &clientInfo, QWidget *parent)
    : QWidget(parent), m_clientID(clientID)
{

    this->setMaximumHeight(60);
    QString s = clientInfo;
    s += ", id:" + QString::number(clientID);

    auto *layout = new QVBoxLayout(this);
    layout->setDirection(QVBoxLayout::LeftToRight);
    auto *content = new QLabel(s, this);
    layout->addWidget(content, 0, Qt::AlignTop);
    for (size_t i = 0; i < m_clients.size(); i++)
    {
        m_clients[i] = new QCheckBox(this);
        layout->addWidget(m_clients[i]);
    }
}

ClientWidgetList::ClientWidgetList(QScrollArea *scrollArea, QWidget *parent)
    : QWidget(parent), m_layout(new QVBoxLayout(this))
{
    m_layout->setDirection(QVBoxLayout::TopToBottom);
    scrollArea->setWidget(this);

    //auto headlineWidget = new QWidget(this);
    //headlineWidget->setMaximumHeight(60);
    //auto headlineLayout = new QVBoxLayout(headlineWidget);
    //headlineLayout->setDirection(QVBoxLayout::LeftToRight);
    //auto clients = new QLabel("Clients", headlineWidget);
    //headlineLayout->addWidget(clients);
    //auto partners = new QLabel("Partners", headlineWidget);
    //headlineLayout->addWidget(partners);
    //auto hosts = new QLabel("Hosts", headlineWidget);
    //headlineLayout->addWidget(hosts);
}

void ClientWidgetList::addClient(int clientID, const QString &clientInfo)
{
    auto cw = new ClientWidget(clientID, clientInfo, this);
    m_layout->addWidget(cw);
    removeClient(clientID);
    m_clients[clientID] = cw;
    for (size_t i = 0; i < cw->m_clients.size(); i++)
    {
        connect(cw->m_clients[i], &QCheckBox::stateChanged, this, [this, i]() {
            checkClientsSelected(static_cast<covise::LaunchStyle>(i));
        });
    }
}

void ClientWidgetList::removeClient(int clientID)
{
    auto cl = m_clients.find(clientID);
    if (cl != m_clients.end())
    {
        m_layout->removeWidget(cl->second);
        delete cl->second;
        cl->second = nullptr;
        m_clients.erase(cl);
    }
}

std::vector<int> ClientWidgetList::getSelectedClients(covise::LaunchStyle launchStyle)
{
    std::vector<int> retval;
    for (const auto &cl : m_clients)
    {
        if (cl.second->m_clients[static_cast<int>(launchStyle)]->isChecked())
        {
            retval.push_back(cl.first);
        }
    }
    return retval;
}

void ClientWidgetList::checkClientsSelected(covise::LaunchStyle launchStyle)
{
    auto it = std::find_if(m_clients.begin(), m_clients.end(), [launchStyle](const std::pair<int, ClientWidget *> &cl) {
        return cl.second->m_clients[static_cast<int>(launchStyle)]->isChecked();
    });
    emit atLeastOneClientSelected(launchStyle, it != m_clients.end());
}

MERemotePartner::MERemotePartner(QWidget *parent)
    : QDialog(parent), m_ui(new Ui::MERemotePartner)
{
    qRegisterMetaType<covise::LaunchStyle>();
	qRegisterMetaType<std::vector<int>>();

    m_ui->setupUi(this);

    m_actions[0] = m_ui->addPartnersBtn;
    m_actions[1] = m_ui->addHostsBtn;
    m_actions[2] = m_ui->disconnectBtn;
    m_clients = new ClientWidgetList(m_ui->partnersArea, this);
    for (size_t i = 0; i < m_actions.size(); i++)
    {
        m_actions[i]->setEnabled(false);
        connect(m_actions[i], &QPushButton::clicked, this, [this, i]() {
            emit takeAction(static_cast<covise::LaunchStyle>(i),
                            m_clients->getSelectedClients(static_cast<covise::LaunchStyle>(i)));
                });
    }
    connect(m_ui->cancelBtn, &QPushButton::clicked, this, [this]() {
        for (auto btn : m_actions)
        {
            btn->setEnabled(false);
        }
        this->hide();
    });
    connect(m_clients, &ClientWidgetList::atLeastOneClientSelected, this, [this](covise::LaunchStyle launchStyle, bool state) {
        m_actions[static_cast<int>(launchStyle)]->setEnabled(state);
    });
}

void MERemotePartner::setPartners(const std::vector<std::pair<int, std::string>> &partners)
{
    for (const auto &p : partners)
    {
        createParnerWidget(p.first, p.second);
    }
}

void MERemotePartner::createParnerWidget(int id, const std::string &hostname)
{
    m_clients->addClient(id, hostname.c_str());
}
