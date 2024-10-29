#ifndef COVER_VRMLFIELDTEMPLATE_H
#define COVER_VRMLFIELDTEMPLATE_H

#include "VrmlField.h"
#include "VrmlNode.h"
#include "VrmlScene.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <variant>

#include "vrmlexport.h"
#include "VrmlMFBool.h"
#include "VrmlMFColor.h"
#include "VrmlMFColorRGBA.h"
#include "VrmlMFDouble.h"
#include "VrmlMFFloat.h"
#include "VrmlMFInt.h"
#include "VrmlMFNode.h"
#include "VrmlMFRotation.h"
#include "VrmlMFString.h"
#include "VrmlMFTime.h"
#include "VrmlMFVec2d.h"
#include "VrmlMFVec2f.h"
#include "VrmlMFVec3d.h"
#include "VrmlMFVec3f.h"
#include "VrmlNodeType.h"
#include "VrmlSFBool.h"
#include "VrmlSFColor.h"
#include "VrmlSFColorRGBA.h"
#include "VrmlSFDouble.h"
#include "VrmlSFFloat.h"
#include "VrmlSFImage.h"
#include "VrmlSFInt.h"
#include "VrmlSFMatrix.h"
#include "VrmlSFNode.h"
#include "VrmlSFRotation.h"
#include "VrmlSFString.h"
#include "VrmlSFTime.h"
#include "VrmlSFVec2d.h"
#include "VrmlSFVec2f.h"
#include "VrmlSFVec3d.h"
#include "VrmlSFVec3f.h"
namespace vrml{




class VrmlNodeChildTemplateImpl;

class VRMLEXPORT VrmlNodeTemplate : public VrmlNode
{

public:
    VrmlNodeTemplate(VrmlScene *scene);
    VrmlNodeTemplate(const VrmlNodeTemplate& other);
    ~VrmlNodeTemplate();
    template<typename T>
    void registerField(const std::string& name, T &field, const std::function<void()> &updateCb = std::function<void()>{});
    bool fieldInitialized(const std::string& name) const;
    bool allFieldsInitialized() const;

private:
    std::unique_ptr<VrmlNodeChildTemplateImpl> m_impl;
    void setField(const char *fieldName, const VrmlField &fieldValue) override;
protected:
    static std::map<std::string, vrml::VrmlNodeType*> m_creators;
    static std::map<std::string, std::function<VrmlNode *(const VrmlNode *)>> m_clones;
};

template<typename Derived>
class VrmlNodTemplateTemplate : public VrmlNodeTemplate
{
public:
    
    static VrmlNode *creator(vrml::VrmlScene *scene){
        auto node = new Derived(scene);
        Derived::initFields(node, nullptr);
        return node;
    }
    
    static vrml::VrmlNodeType *defineType(vrml::VrmlNodeType *t = nullptr)
    {
        static VrmlNodeType *st = 0;
        if (!t)
        {
            if (st)
                return st; // Only define the type once.
            t = st = new VrmlNodeType(Derived::name(), creator);
        }

        VrmlNodeTemplate::defineType(t); // Parent class
        
        Derived::initFields(nullptr, t);

        return t;
    }

    vrml::VrmlNode *cloneMe() const override
    {
        auto node = new Derived(dynamic_cast<const Derived&>(*this));
        Derived::initFields(node, nullptr);
        return node;
    }

    vrml::VrmlNodeType *nodeType() const override
    {
        return defineType();
    }

protected:
    VrmlNodTemplateTemplate(VrmlScene *scene)
    : VrmlNodeTemplate(scene)
    {}
};

template<typename T>
VrmlField::VrmlFieldType toEnumType(const T *t = nullptr);

enum FieldAccessibility{
    Private, Exposed
};

template <typename T, FieldAccessibility FT>
struct NameValueStruct {
    std::string name;
    T &value;
};

template<typename T>
NameValueStruct<T, FieldAccessibility::Private> field(const std::string &name, T &value) {
    return NameValueStruct<T, FieldAccessibility::Private>{name, value};
}

template<typename T>
NameValueStruct<T, FieldAccessibility::Exposed> exposedField(const std::string &name, T &value) {
    return NameValueStruct<T, FieldAccessibility::Exposed>{name, value};
}

template <typename T>
void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<T, FieldAccessibility::Private> &field);

template <typename T>
void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<T, FieldAccessibility::Exposed> &field);

template <typename...Args>
void initFieldsHelper(VrmlNodeTemplate *node, VrmlNodeType *t, const Args&... fields) {
        (initFieldsHelperImpl(node, t, fields), ...);
}

#define FOR_ALL_VRML_TYPES(code)\
    code(VrmlSFBool)\
    code(VrmlSFColor)\
    code(VrmlSFColorRGBA)\
    code(VrmlSFDouble)\
    code(VrmlSFFloat)\
    code(VrmlSFInt)\
    code(VrmlSFRotation)\
    code(VrmlSFTime)\
    code(VrmlSFVec2d)\
    code(VrmlSFVec3d)\
    code(VrmlSFVec2f)\
    code(VrmlSFVec3f)\
    code(VrmlSFImage)\
    code(VrmlSFString)\
    code(VrmlMFBool)\
    code(VrmlMFColor)\
    code(VrmlMFColorRGBA)\
    code(VrmlMFDouble)\
    code(VrmlMFFloat)\
    code(VrmlMFInt)\
    code(VrmlMFRotation)\
    code(VrmlMFString)\
    code(VrmlMFTime)\
    code(VrmlMFVec2d)\
    code(VrmlMFVec3d)\
    code(VrmlMFVec2f)\
    code(VrmlMFVec3f)\
    code(VrmlSFNode)\
    code(VrmlMFNode)\
    code(VrmlSFMatrix)


#define VRMLNODECHILD2_TEMPLATE_DECL(type) \
extern template void VRMLEXPORT VrmlNodeTemplate::registerField<type>(const std::string& name, type &field, const std::function<void()> &updateCb);
FOR_ALL_VRML_TYPES(VRMLNODECHILD2_TEMPLATE_DECL)

#define TO_VRML_FIELD_TYPES_DECL(type) \
extern template VrmlField::VrmlFieldType VRMLEXPORT toEnumType(const type *t);
FOR_ALL_VRML_TYPES(TO_VRML_FIELD_TYPES_DECL)

#define INIT_FIELDS_HELPER_DECL(type) \
extern template void VRMLEXPORT initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Private> &field); 
FOR_ALL_VRML_TYPES(INIT_FIELDS_HELPER_DECL)

#define INIT_EXPOSED_FIELDS_HELPER_DECL(type) \
extern template void VRMLEXPORT initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Exposed> &field); 
FOR_ALL_VRML_TYPES(INIT_EXPOSED_FIELDS_HELPER_DECL)









//experiments
//____________________________________________________________________________________


class VrmlNode3 : public VrmlNodeTemplate
{
public:
    // VrmlNode3(vrml::VrmlScene *s, const std::string &name);

    // vrml::VrmlNode *cloneMe() const override;

    // vrml::VrmlNodeType *nodeType() const override;
    
    template<typename Derived>
    static VrmlNode *creator(vrml::VrmlScene *scene){
        auto node = new Derived(scene);
        Derived::initFields(node, nullptr);
        return node;
    }
    
    template<typename Derived>
    static vrml::VrmlNodeType *defineType(vrml::VrmlNodeType *t = nullptr)
    {
        static VrmlNodeType *st = 0;
        if (!t)
        {
            if (st)
                return st; // Only define the type once.
            t = st = new VrmlNodeType(Derived::name(), creator<Derived>);
            m_creators[Derived::name()] = t;
            m_clones[Derived::name()] = [](const VrmlNode *node){ 
                auto newNode = new Derived(*dynamic_cast<const Derived*>(node)); 
                Derived::initFields(newNode, nullptr);
                return newNode;
            };
        }

        VrmlNode::defineType(t); // Parent class
        
        Derived::initFields(nullptr, t);

        return t;
    }

    VrmlNode3(VrmlScene *s, const std::string &name)
    : VrmlNodeTemplate(s)
    , m_name(name)
    {}

    vrml::VrmlNode *cloneMe() const
    {
        return m_clones[m_name](this); 
    }

    vrml::VrmlNodeType *nodeType() const
    {
        return m_creators[m_name];
    }

private:
    // static std::map<std::string, vrml::VrmlNodeType*> m_creators;
    const std::string m_name;
};










} // vrml


#endif // COVER_VRMLFIELDTEMPLATE_H