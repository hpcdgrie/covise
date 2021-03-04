/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#ifndef CONTROLLER_MODULE_INFO_H
#define CONTROLLER_MODULE_INFO_H

#include "port.h"
#include <string>
#include <vector>

namespace covise{
namespace controller{
//Holds the input and output ports of a module and their parameters
struct ModuleNetConnectivity
{
    std::vector<C_interface> interfaces;
    std::vector<parameter> inputParams, outputParams;
    void addInterface(const std::string &name, const std::string &type, Direction direction, const std::string &text, const std::string &demand);
    void addParameter(const std::string &name, const std::string &type, const std::string &text, const std::string &value, const std::string ext, Direction dir);

    const parameter &getParam(const std::string& paramName) const;
    parameter &getParam(const std::string& paramName);

    const net_interface &getInterface(const std::string &interfaceName) const;
    net_interface &getInterface(const std::string &interfaceName);

    void forAllNetInterfaces(const std::function < void(net_interface &)> &func);
    void forAllNetInterfaces(const std::function < void(const net_interface &)> &func) const;
};

const parameter *getParameter(const std::vector<parameter> &params, const std::string &parameterName);
parameter *getParameter(std::vector<parameter> &params, const std::string &parameterName);

//name, category and connectivity info of a module class
//and the module count wich is used to give each instance of a module its instance
struct ModuleInfo
{
    ModuleInfo(const std::string &name, const std::string &category);
    const std::string name;  //executable of the module must be under $COVISE_PATH/$ARCHSUFFIX/bin/name
    const std::string category; //if set used as sub-directory in $COVISE_PATH/$ARCHSUFFIX/bin/category/name
    mutable size_t count = 0;
    const ModuleNetConnectivity connectivity() const;
    const std::string &description() const;
    void readConnectivity(const char *buff);
    bool operator==(const ModuleInfo &other) const;
    bool operator<(const ModuleInfo &other) const;


private:
    std::string m_description;
    ModuleNetConnectivity m_connectivity;
};

} // namespace controller
} // namespace covise
#endif // !CONTROLLER_MODULE_INFO_H
