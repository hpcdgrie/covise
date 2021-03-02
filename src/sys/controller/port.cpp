/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include "handler.h"
#include "list.h"
#include "port.h"
#include "util.h"
#include "renderModule.h"
//**********************************************************************
//
// 				PORT
//
//**********************************************************************

using namespace covise;
using namespace covise::controller;
port::port()
{
}

void port::set_name(const string &str)
{
    name = str;
}

void port::set_type(const string &str)
{
    type = str;
}

void port::set_text(const string &str)
{
    text = str;
}

//**********************************************************************
//
// 				INTERFACE
//
//**********************************************************************

C_interface::C_interface()
    : port()
{
}

void C_interface::set_demand(const string &str)
{
    demand = str;
}

void C_interface::set_direction(controller::Direction dir)
{
    direction = dir;
}

//**********************************************************************
//
// 				VALUE_LIST
//
//**********************************************************************

void Value::set(string str)
{
    string tmp = str;
    int len = (int)str.length();
    if (len > 1 && (tmp[0] == '"') && (tmp[len - 1] == '"')) // we have a string with quotes, so remove them
    {
        tmp.erase(len, 1);
        tmp.erase(0, 1);
    }
    s = str;
}

value_list::value_list()
    : Liste<Value>()
{
    count = 0;
}

//**********************************************************************
//
// 				PARAMETER
//
//**********************************************************************

parameter::parameter()
    : port()
{
    values = new value_list;

    extension = "-1";
    panel = "-1";
}

int parameter::get_count() const
{
    return values->count;
}

void parameter::set_addvalue(const string &add_para)
{
    panel = add_para;
}

std::string parameter::serialize() const
{
    std::stringstream buff;
    buff << get_name() << "\n"
         << get_type() << "\n"
         << get_count() << "\n";

    for (int i = 1; i <= get_count(); i++)
        buff << get_value(i) << "\n";
    return buff.str();
}

std::string parameter::getDescription() const
{
    std::stringstream ss;
    ss << get_name() << "\n"
       << get_type() << "\n"
       << get_text() << "\n"
       << get_org_val() << "\n";
    return ss.str();
}

void parameter::set_extension(const string &ext)
{
    extension = ext;
}

//Wird aufgerufen, wenn ein Parameter verändert wird
void parameter::set_value(int no, string str)
{
    int i;
    Value *v_tmp = NULL;

    if (values->count < no)
    {
        v_tmp = new Value;
        v_tmp->set(str);
        values->add(v_tmp);
        values->count++;
    }
    else
    {
        values->reset();
        i = 0;
        while (i < no)
        {
            v_tmp = values->next();
            i++;
        }
        v_tmp->set(str);
    }
}

//Wird evtl. auch aufgerufen, wenn ein Parameter verändert wird
void parameter::set_value_list(string strVal)
{
    Value *tmp_val;

    // copy original string into org_val
    org_val = strVal;

    // test, if a values-list exits
    if (values->count > 0) // remove all values
    {
        while (values->count > 0)
        {
            values->reset();
            tmp_val = values->next();
            values->remove(tmp_val);
            values->count--;
        }
    }

    // parse values
    string partype = this->get_type();
    if (partype == "String" || partype == "STRING" || partype == "Text" || partype == "TEXT" || partype == "text")
    {
        Value *tmp_val = new Value();
        tmp_val->set(strVal);
        values->add(tmp_val);
        values->count++;
    }

    else if (partype == "Browser")
    {
        Value *tmp_val = new Value();
        tmp_val->set(strVal);
        values->add(tmp_val);
        values->count++;
    }

    else
    {
        if (strVal.empty())
        {
            cerr << "parameter::set_value_list error: parameter value is NULL" << endl;
            return;
        }

        vector<string> list = splitString(strVal, " ");
        for (int i = 0; i < list.size(); i++)
        {
            Value *tmp_val = new Value;
            tmp_val->set(list[i]);
            values->add(tmp_val);
            values->count++;
        }
    }
}

const string &parameter::get_value(int no) const
{
    assert(no <= values->count);

    values->reset();
    Value *v_tmp;
    int i = 0;
    while (i < no)
    {
        i++;
        v_tmp = values->next();
    }
    return v_tmp->get();
}

string parameter::get_val_list() const
{
    string retVal;
    values->reset();
    for (int i = 0; i < values->count; i++)
    {
        if (i == 0 && get_type() == "Browser")
        {
            retVal.append(this->get_value(i + 1));
        }

        else
        {
            retVal.append(this->get_value(i + 1));
        }

        if (i < values->count - 1)
        {
            retVal.append(" ");
        }
    }

    return retVal;
}

string parameter::get_pyval_list() const
{

    string retVal;
    values->reset();

    if (get_type() == "Browser" || get_type() == "Browser-Filter" || get_type() == "Boolean" || get_type() == "String")
    {
        retVal.append("\"");
        retVal.append(get_value(1));
        retVal.append("\"");
    }

    else if (get_type() == "FloatVector" || get_type() == "FloatSlider" || get_type() == "IntVector" || get_type() == "IntSlider")
    {
        // python uses a fix vector length of 3
        int end = values->count;
        for (int i = 0; i < end; i++)
        {
            retVal.append(get_value(i + 1));
            if (i != end - 1)
                retVal.append(", ");
        }
    }

    else if (get_type() == "Color")
    {
        int end = values->count;
        for (int i = 0; i < end; i++)
        {
            retVal.append(get_value(i + 1));
            if (i != values->count - 1)
                retVal.append(", ");
        }
    }

    else if (get_type() == "Colormap" || get_type() == "BrowserFilter")
    {
        retVal.append("\"");
        ;
        for (int i = 0; i < values->count; i++)
        {
            retVal.append(get_value(i + 1));
            retVal.append(" ");
        }
        retVal.append("\"");
    }

    else
    {
        retVal.append(get_value(1));
    }

    return retVal;
}

//**********************************************************************
//
// 		     CONNECT & CONNECT_OBJ_LIST
//
//**********************************************************************
connect_obj::connect_obj()
{
    conn_obj = NULL;
}

void connect_obj::set_conn(object *obj)
{
    conn_obj = obj;
}

object *connect_obj::get_obj()
{
    return conn_obj;
}

void connect_obj::del_conn()
{
    conn_obj = NULL;
}

void connect_obj::set_oldname(const string &str)
{
    old_name = str;
}

const string &connect_obj::get_oldname() const
{
    return old_name;
}

connect_obj_list::connect_obj_list()
    : Liste<connect_obj>()
{
}

//**********************************************************************
//
// 			NET_INTERFACE
//
//**********************************************************************

net_interface::net_interface()
    : C_interface()
{
    obj = NULL;
}

net_interface::~net_interface()
{
    if (get_direction() == Direction::Output && get_object())
    {
        if (get_conn_state())
        {
            get_object()->del_all_DO(m_alreadyDead);
        }
        if (m_alreadyDead >= 0)
        {
            get_object()->del_old_data();
        }
    }
}

void net_interface::set_connect(object *conn)
{
    obj = conn;
}

void net_interface::del_connect()
{
    obj = NULL;
}

object *net_interface::get_object()
{
    return obj;
}

const object *net_interface::get_object() const
{
    return obj;
}

//----------------------------------------------------------------------
// get_state checks the status of a net_interface
// if the demand is opt and no connection exists, it returns OPT
// if the demand is not opt and no connection exists, it return INIT
// if a connection to the object exists and no Dataobject exists
//    or no connection from the object to another module,
//    it returns INIT,
// if in the connected objects exists new Data, it returns NEW
// if in the connected objects exists Data, which was read before, it
//    returns OLD
//----------------------------------------------------------------------

int net_interface::get_state(const controller::Application *mod) const
{
    int tmp;

    // get object-state:
    if (!obj)
    {
        // INIT, NEW, OLD
        string tmp_state = obj->get_conn_state(mod, this->get_name());
        if (tmp_state == "INIT")
        {
            tmp = S_INIT;
        }
        else if (tmp_state == "NEW")
        {
            tmp = S_NEW;
        }
        else
        {
            tmp = S_OLD;
        }
    }
    else
    {
        // no connection specified
        if (this->demand == "opt")
        {
            // no connections required
            tmp = S_OPT;
        }
        else
        {
            tmp = S_INIT;
        }
    }

#ifdef DEBUG
    cerr << "Module " << mod->get_name() << " Interface " << this->get_name() << " status " << tmp_state << "\n";
#endif

    return tmp;
}

void net_interface::set_outputtype(const string & /*unused*/, const string &DO_type)
{
    if (obj != 0)
    {
        obj->set_outputtype(DO_type);
    }
}

std::string net_interface::serialize() const
{
}

//**********************************************************************
//
// 			RENDER_INTERFACE
//
//**********************************************************************

render_interface::render_interface()
    : C_interface()
{
    connects = new connect_obj_list;
    conn_count = 0;
    wait_count = 0;
}

void render_interface::set_connect(object *obj)
{
    connect_obj *tmp;

    tmp = new connect_obj;
    tmp->set_conn(obj);
    connects->add(tmp);
    conn_count = conn_count + 1;
    // wait_count = wait_count+1;
    wait_count = conn_count;
}

void render_interface::del_connect(object *obj, std::vector<Display>& displays)
{

    connects->reset();
    connect_obj *tmp = connects->next();

    while (tmp && obj != tmp->get_obj())
        tmp = connects->next();

    if (tmp)
    {
        if (!tmp->get_oldname().empty())
        {
            for(Display &d : displays)
                d.send_del(tmp->get_oldname(), "");
        }
        tmp->del_conn();
        connects->remove(tmp);
        conn_count = conn_count - 1;
        wait_count = wait_count - 1;
    }
}

void render_interface::reset_wait()
{
    wait_count = conn_count;
}

void render_interface::decr_wait()
{
    wait_count = wait_count - 1;
}

bool render_interface::get_wait_status()
{
    bool tmp = false;
    if (wait_count >= 1)
        tmp = true;
    return tmp;
}

void render_interface::del_all_connections(const controller::Renderer *mod)
{
    connect_obj *tmp;
    connects->reset();
    while ((tmp = connects->next()) != NULL)
    {
        object *obj = tmp->get_obj();
        obj->del_to_connection(mod->info().name, std::to_string(mod->instance()), mod->host.userInfo().hostName, this->get_name());
        connects->remove(tmp);
        connects->reset();
    }
    delete connects;
}

bool render_interface::get_conn_state()
{
    bool tmp_state;
    connect_obj *tmp;

    connects->reset();
    tmp = connects->next();
    if (tmp == NULL)
    {
        tmp_state = false;
    }
    else
    {
        tmp_state = true;
    }
    return tmp_state;
}

int render_interface::check_conn()
{
    int check;
    connect_obj *tmp;

    connects->reset();
    tmp = connects->next();
    if (tmp == NULL)
    {
        check = 0;
    }
    else
    {
        check = 1;
    }
    return check;
}

//----------------------------------------------------------------------
// get_state checks the status of a render_interface
// if the demand is opt and no connection exists, it returns OPT
// if the demand is req and no connection exists, it return INIT
// if a connection to the object exists and no Dataobject exists
//    or no connection from the object to another module,
//    it returns INIT,
// if in the connected objects exists new Data, it returns NEW
// if in the connected objects exists Data, which was read before, it
//    returns OLD
// falls mindestens in einer Connection neue Daten vorhanden sind, NEW
//----------------------------------------------------------------------
int render_interface::get_state(const controller::Application *mod) const
{
    int state;
    string tmp_state;
    connect_obj *tmp;
    object *obj;

    connects->reset();
    bool new_data = false;
    bool connected = false;

    // search, if in any connection new data exists
    while ((tmp = connects->next()) != NULL)
    {
        connected = true;
        obj = tmp->get_obj();
        //INIT, NEW, OLD
        tmp_state = obj->get_conn_state(mod, this->get_name());

        if (tmp_state == "INIT")
        {
            state = S_INIT;
        }
        else if (tmp_state == "NEW")
        {
            state = S_NEW;
        }
        else
        {
            state = S_OLD;
        }

        // new data found. set signal NEW_DATA
        if (state == S_NEW)
            new_data = true;
    }

    if (connected == false)
    {
        if (this->demand == "opt")
        {
            // no connections required
            state = S_OPT;
        }
        else
        {
            state = S_INIT;
        }
    }
    else if (new_data == true)
    {
        state = S_NEW;
    }
    else
    {
        state = S_OLD;
    }

    return state;
}

connect_obj *render_interface::get_first_NEW(const controller::Renderer *mod)
{
    connect_obj *tmp_conn;
    connects->reset();
    bool new_data = false;

    while (new_data == false)
    {
        tmp_conn = connects->next();
        object *obj = tmp_conn->get_obj();
        string tmp_state = obj->get_conn_state(mod, this->get_name());
        if (tmp_state == "NEW")
            new_data = true;
    }
    return tmp_conn;
}

//**************************************************************************
//
//	get_objlist returns a string with the new names of the dataobjects
//	of the renderer interface. The dataobjects are seperated by \n
//	At the end and the beginning of the string is no \n!
//**************************************************************************

string render_interface::get_objlist() const
{
    bool first = true;
    string tmpbuffer;

    connects->reset();
    connect_obj *tmp_conn;
    while ((tmp_conn = connects->next()) != NULL)
    {
        object *obj = tmp_conn->get_obj();
        string tmp = obj->get_current_name();
        if (tmp.empty())
            tmp = "Not jet created";
        if (first == true)
        {
            tmpbuffer = tmp;
            first = false;
        }

        else
        {
            tmpbuffer = tmp;
#ifdef SWITCH
            tmpbuffer.append("|");
#else
            tmpbuffer.append("\n");
#endif
            tmpbuffer.append(tmp);
        }
    }
    return tmpbuffer;
}

void render_interface::count_init(const controller::Renderer *mod)
{
    int tmp_count;

    connects->reset();
    connect_obj *tmp;
    tmp_count = 0;
    while ((tmp = connects->next()) != NULL)
    {
        object *obj = tmp->get_obj();
        string tmp_state = obj->get_conn_state(mod, get_name());
        if (tmp_state == "INIT")
            tmp_count = tmp_count + 1;
    }
    wait_count = tmp_count;
}

void render_interface::reset_to_NEW(const controller::Renderer *mod)
{
    connects->reset();
    connect_obj *tmp_conn;
    while ((tmp_conn = connects->next()) != NULL)
    {
        object *obj = tmp_conn->get_obj();
        string tmp_state = obj->get_conn_state(mod, get_name());
        if (tmp_state == "OLD")
            obj->set_to_NEW();
    }
}



