#ifndef COVER_VRMLFIELDTEMPLATE_H
#define COVER_VRMLFIELDTEMPLATE_H

#include "VrmlField.h"
#include "VrmlNode.h"

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

class VrmlScene;


class VrmlNodeUpdateRegistry;

class VRMLEXPORT VrmlNodeTemplate : public VrmlNode
{
private:
    std::unique_ptr<VrmlNodeUpdateRegistry> m_impl;
    struct Constructors{
        vrml::VrmlNodeType* creator;
        // std::function<vrml::VrmlNode*(const VrmlNode*)> clone;
        vrml::VrmlNode*(*clone)(const VrmlNode*);
    };
    static std::map<std::string, Constructors> m_constructors;
    const std::map<std::string, Constructors>::const_iterator m_constructor;

    void setField(const char *fieldName, const VrmlField &fieldValue) override;
protected:

    enum FieldAccessibility{
        Private, Exposed, EventIn, EventOut
    };

    using FieldUpdateCallback = std::function<void()>;
    template<typename VrmlType>
    static void registerField(VrmlNodeTemplate *node, const std::string& name, VrmlType *field, const FieldUpdateCallback &updateCb = std::function<void()>{});

    VrmlNodeTemplate(VrmlScene *scene, const std::string &name);
    VrmlNodeTemplate(const VrmlNodeTemplate& other);

    template <typename VrmlType, FieldAccessibility FT>
    struct NameValueStruct {
        std::string name;
        VrmlType *value;
        FieldUpdateCallback updateCb;
    };

    template<typename VrmlType>
    static NameValueStruct<VrmlType, FieldAccessibility::Private> field(const std::string &name, VrmlType &value, const FieldUpdateCallback &updateCb = FieldUpdateCallback()) {
        return NameValueStruct<VrmlType, FieldAccessibility::Private>{name, &value, updateCb};
    }

    template<typename VrmlType>
    static NameValueStruct<VrmlType, FieldAccessibility::Exposed> exposedField(const std::string &name, VrmlType &value, const FieldUpdateCallback &updateCb = FieldUpdateCallback()) {
        return NameValueStruct<VrmlType, FieldAccessibility::Exposed>{name, &value, updateCb};
    }

    template<typename VrmlType>
    static NameValueStruct<VrmlType, FieldAccessibility::EventIn> eventInCallBack(const std::string &name, const FieldUpdateCallback &updateCb) {
        return NameValueStruct<VrmlType, FieldAccessibility::EventIn>{name, nullptr, updateCb};
    }

    template<typename VrmlType>
    static NameValueStruct<VrmlType, FieldAccessibility::EventOut> eventOutCallBack(const std::string &name, const FieldUpdateCallback &updateCb) {
        return NameValueStruct<VrmlType, FieldAccessibility::EventOut>{name, nullptr, updateCb};
    }

    template <typename VrmlType>
    static void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<VrmlType, FieldAccessibility::Private> &field);

    template <typename VrmlType>
    static void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<VrmlType, FieldAccessibility::Exposed> &field);
    
    template <typename VrmlType>
    static void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<VrmlType, FieldAccessibility::EventIn> &field);

    template <typename VrmlType>
    static void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<VrmlType, FieldAccessibility::EventOut> &field);

    template <typename...Args>
    static void initFieldsHelper(VrmlNodeTemplate *node, VrmlNodeType *t, const Args&... fields) {
            (initFieldsHelperImpl(node, t, fields), ...);
    }

public:
    ~VrmlNodeTemplate();
    std::ostream &printFields(std::ostream &os, int indent) override;
    const VrmlField *getField(const char *fieldName) const override;
    //hide setField
    void setFieldByName(const char *fieldName, const VrmlField &fieldValue);

    bool fieldInitialized(const std::string& name) const;
    bool allFieldsInitialized() const;

    vrml::VrmlNode *cloneMe() const;
    vrml::VrmlNodeType *nodeType() const;

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
            Constructors cs;
            cs.creator = t;
            cs.clone = [](const VrmlNode *node){ 
                auto newNode = new Derived(*dynamic_cast<const Derived*>(node)); 
                Derived::initFields(newNode, nullptr);
                return static_cast<vrml::VrmlNode*>(newNode);
            };
            m_constructors[Derived::name()] = cs;
        }

        VrmlNode::defineType(t); // Parent class
        
        Derived::initFields(nullptr, t);

        return t;
    }


};

template<typename VrmlType>
VrmlField::VrmlFieldType toEnumType(const VrmlType *t = nullptr);

struct DummyType{};

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
    code(VrmlSFMatrix)\
    code(VrmlField)


#define VRMLNODECHILD2_TEMPLATE_DECL(type) \
extern template void VRMLEXPORT VrmlNodeTemplate::registerField<type>(VrmlNodeTemplate *node, const std::string& name, type *field, const std::function<void()> &updateCb);
FOR_ALL_VRML_TYPES(VRMLNODECHILD2_TEMPLATE_DECL)

#define TO_VRML_FIELD_TYPES_DECL(type) \
extern template VrmlField::VrmlFieldType VRMLEXPORT toEnumType(const type *t);
FOR_ALL_VRML_TYPES(TO_VRML_FIELD_TYPES_DECL)

#define INIT_FIELDS_HELPER_DECL(type) \
extern template void VRMLEXPORT VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Private> &field); 
FOR_ALL_VRML_TYPES(INIT_FIELDS_HELPER_DECL)

#define INIT_EXPOSED_FIELDS_HELPER_DECL(type) \
extern template void VRMLEXPORT VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Exposed> &field); 
FOR_ALL_VRML_TYPES(INIT_EXPOSED_FIELDS_HELPER_DECL)

#define INIT_EVENT_IN_HELPER_DECL(type) \
extern template void VRMLEXPORT VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::EventIn> &field); 
FOR_ALL_VRML_TYPES(INIT_EVENT_IN_HELPER_DECL)

#define INIT_EVENT_OUT_HELPER_DECL(type) \
extern template void VRMLEXPORT VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::EventOut> &field); 
FOR_ALL_VRML_TYPES(INIT_EVENT_OUT_HELPER_DECL)

} // vrml


#endif // COVER_VRMLFIELDTEMPLATE_H