#include "VrbCredentials.h"
#include <config/CoviseConfig.h>

using namespace vrb;
using namespace covise;

VrbCredentials::VrbCredentials(const std::string &ip, unsigned int tcp, unsigned int udp)
: ipAddress(ip)
, tcpPort(tcp)
, udpPort(udp)
{

}

VrbCredentials::VrbCredentials()
    : ipAddress(coCoviseConfig::getEntry("System.VRB.Server"))
    , tcpPort(coCoviseConfig::getInt("tcpPort", "System.VRB.Server", 31800))
    , udpPort(coCoviseConfig::getInt("udpPort", "System.VRB.Server", 31801))
{
}

covise::TokenBuffer &vrb::operator<<(covise::TokenBuffer &tb, const VrbCredentials &cred){
    tb << cred.ipAddress << static_cast<int>(cred.tcpPort) << static_cast<int>(cred.udpPort);
    return tb;
}
