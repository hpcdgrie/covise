#include "VrmlNodeTemplate.h"
#include "VrmlField.h"
#include "VrmlNode.h"
#include "VrmlScene.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <variant>

#include <cassert>

namespace vrml{

    //keep in line with VrmlField enum
    typedef std::tuple<
        VrmlSFBool,
        VrmlSFColor,
        VrmlSFColorRGBA,
        VrmlSFDouble,
        VrmlSFFloat,
        VrmlSFInt,
        VrmlSFRotation,
        VrmlSFTime,
        VrmlSFVec2d,
        VrmlSFVec3d,
        VrmlSFVec2f,
        VrmlSFVec3f,
        VrmlSFImage,
        VrmlSFString,
        VrmlMFBool,
        VrmlMFColor,
        VrmlMFColorRGBA,
        VrmlMFDouble,
        VrmlMFFloat,
        VrmlMFInt,
        VrmlMFRotation,
        VrmlMFString,
        VrmlMFTime,
        VrmlMFVec2d,
        VrmlMFVec3d,
        VrmlMFVec2f,
        VrmlMFVec3f,
        VrmlSFNode,
        VrmlMFNode,
        VrmlSFMatrix,
        VrmlField
    > VrmlTypesTuple;


template <typename VrmlType, typename Tuple, std::size_t Index = 0>
struct TypeToEnumHelper {
    static VrmlField::VrmlFieldType value() {
        if constexpr (Index < std::tuple_size_v<Tuple>) {
            if constexpr (std::is_same_v<VrmlType, std::tuple_element_t<Index, Tuple>>) {
                return static_cast<VrmlField::VrmlFieldType>(Index + 1);
            } else {
                return TypeToEnumHelper<VrmlType, Tuple, Index + 1>::value();
            }
        } else {
            return VrmlField::VrmlFieldType::NO_FIELD;
        }
    }
};

// Function to map type to enumeration value
template<typename VrmlType>
VrmlField::VrmlFieldType toEnumType(const VrmlType *t) {
    return TypeToEnumHelper<VrmlType, VrmlTypesTuple>::value();
}


// Helper to extract types from tuple and create a variant with pointers to those types
template<typename Tuple, std::size_t... Indices>
auto tuple_to_variant_ptr_impl(std::index_sequence<Indices...>) {
    return std::variant<std::add_pointer_t<std::tuple_element_t<Indices, Tuple>>...>{};
}

template<typename Tuple>
using tuple_to_variant_ptr = decltype(tuple_to_variant_ptr_impl<Tuple>(std::make_index_sequence<std::tuple_size_v<Tuple>>{}));


class VrmlNodeUpdateRegistry
{

public:
    VrmlNodeUpdateRegistry(VrmlNodeTemplate *nodeChild)
    : m_nodeChild(nodeChild)
    {}


private:
    //use pointers to avoid memory overhead for arrays
    
    using VrmlTypesVariant = tuple_to_variant_ptr<VrmlTypesTuple>;

    struct VrmlTypeStruct{
        VrmlTypesVariant type;
        bool initialized = false;
        std::function<void()> updateCb;
    };
    std::map<std::string, VrmlTypeStruct> m_fields;
    VrmlNode *m_nodeChild;
    template<typename VrmlType>
    const VrmlType* getField(const VrmlField &fieldValue, const VrmlType *t){
        const VrmlType* val = dynamic_cast<const VrmlType*>(&fieldValue);
        if(!val)
            return nullptr;
        return val;
    }
    template<>
    const VrmlField* getField(const VrmlField &fieldValue, const VrmlField *t){
        if(fieldValue.fieldType() != t->fieldType()){
            return nullptr;
        }
        return &fieldValue;
    }

public:
    const VrmlField *getField(const char *fieldName) const
    {
        auto it = m_fields.find(fieldName);
        if(it == m_fields.end()){
            return m_nodeChild->getField(fieldName);
        }
        auto& field = it->second;
        
        return std::visit([](auto&& arg){
            return static_cast<const VrmlField*>(arg);
        }, field.type);
        return nullptr;
    }
    
    
    void setField(const char *fieldName, const VrmlField &fieldValue) {
        auto it = m_fields.find(fieldName);
        if(it == m_fields.end()){
            m_nodeChild->setField(fieldName, fieldValue);
            return;
        }
        auto& field = it->second;
        std::visit([fieldName, &fieldValue, this](auto&& arg){
            auto val = getField(fieldValue, arg);
            if(!val){
                System::the->error("Invalid type (%s) for %s field.\n",
                    fieldValue.fieldTypeName(), fieldName);
                return;
            }
            *arg = *val;
        }, field.type);
        field.initialized = true;
        if(field.updateCb){
            field.updateCb();
        }
    }
    
    template<typename VrmlType>
    void registerField(const std::string& name, VrmlType &field, const std::function<void()> &updateCb =  std::function<void()>{}){
        m_fields[name] = VrmlTypeStruct{ &field, false, updateCb};
    }

    bool initialized(const std::string& name){
        auto it = m_fields.find(name);
        if(it == m_fields.end()){
            return false;
        }
        return it->second.initialized;
    }

    bool allInitialized(){
        for(auto& [name, field] : m_fields){
            if(!field.initialized){
                return false;
            }
        }
        return true;
    }

    template<typename VrmlType>
    VrmlType* copy(const VrmlType* other){
        return new VrmlType(*other);
    }

    template<>
    VrmlField* copy(const VrmlField* other){
        assert(!("can not copy abstract VrmlField"));
        return nullptr;
    }

    // use new pointers with this inital values of other
    VrmlNodeUpdateRegistry(const VrmlNodeUpdateRegistry& other)
    : m_fields(other.m_fields)
    {
        for(auto& [name, field] : m_fields){
            std::visit([this, &field](auto&& arg){
                arg = copy(arg);
            }, field.type);
        }
    }

    template<typename VrmlType>
    void deleter(VrmlType* t){
        delete t;
    }
    std::ostream &printFields(std::ostream &os, int indent)
    {
        for(auto& [name, field] : m_fields){
            os << std::string(indent, ' ') << name << " : ";
            std::visit([&os](auto&& arg){
                os << *arg;
            }, field.type);
            os << std::endl;
        }
        return os;
    }

};

std::map<std::string, VrmlNodeTemplate::Constructors> VrmlNodeTemplate::m_constructors;

VrmlNodeTemplate::VrmlNodeTemplate(VrmlScene *scene, const std::string &name)
: VrmlNode(scene)
, m_constructor(m_constructors.find(name)) 
, m_impl(std::make_unique<VrmlNodeUpdateRegistry>(this)) {}

// use new pointers with this inital values of other
VrmlNodeTemplate::VrmlNodeTemplate(const VrmlNodeTemplate& other)
: VrmlNode(other)
, m_impl(std::make_unique<VrmlNodeUpdateRegistry>(*other.m_impl)) {}

VrmlNodeTemplate::~VrmlNodeTemplate() = default;

vrml::VrmlNode *VrmlNodeTemplate::cloneMe() const
{
    return m_constructor->second.clone(this); 
}

vrml::VrmlNodeType *VrmlNodeTemplate::nodeType() const
{
    return m_constructor->second.creator; 
}

bool VrmlNodeTemplate::fieldInitialized(const std::string& name) const
{
    return m_impl->initialized(name);
}

bool VrmlNodeTemplate::allFieldsInitialized() const
{
    return m_impl->allInitialized();
}

void VrmlNodeTemplate::setField(const char *fieldName, const VrmlField &fieldValue) 
{
    m_impl->setField(fieldName, fieldValue);
}

std::ostream &VrmlNodeTemplate::printFields(std::ostream &os, int indent)
{
    return m_impl->printFields(os, indent);
}

const VrmlField *VrmlNodeTemplate::getField(const char *fieldName) const
{
    return m_impl->getField(fieldName);
}

void VrmlNodeTemplate::setFieldByName(const char *fieldName, const VrmlField &fieldValue)
{
    m_impl->setField(fieldName, fieldValue);
}


template<typename VrmlType>
void VrmlNodeTemplate::registerField(VrmlNodeTemplate *node, const std::string& name, VrmlType &field, const std::function<void()> &updateCb)
{
    return node->m_impl->registerField<VrmlType>(name, field, updateCb);
}

template <typename VrmlType>
void addField(VrmlNodeType *t, const std::string &name, VrmlType &field) {
    t->addField(name.c_str(), toEnumType<std::remove_reference_t<VrmlType>>());
}

template <typename VrmlType>
void addExposedField(VrmlNodeType *t, const std::string &name, VrmlType &field) {
    t->addExposedField(name.c_str(), toEnumType<std::remove_reference_t<VrmlType>>());
}

template <typename VrmlType>
void VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<VrmlType, FieldAccessibility::Private> &field)
{
    if (node) 
        registerField(node, field.name, field.value, field.updateCb);
    if (t) 
        addField(t, field.name, field.value);
}

template <typename VrmlType>
void VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<VrmlType, FieldAccessibility::Exposed> &field)
{
    if (node) 
        registerField(node, field.name, field.value);
    if (t) 
        addExposedField(t, field.name, field.value);
}

#define VRMLNODECHILD2_TEMPLATE_IMPL(type) \
template void VRMLEXPORT VrmlNodeTemplate::registerField<type>(VrmlNodeTemplate *node, const std::string& name, type &field, const std::function<void()> &updateCb);
FOR_ALL_VRML_TYPES(VRMLNODECHILD2_TEMPLATE_IMPL)

#define TO_VRML_FIELD_TYPES_IMPL(type) \
template VrmlField::VrmlFieldType VRMLEXPORT toEnumType(const type *t);
FOR_ALL_VRML_TYPES(TO_VRML_FIELD_TYPES_IMPL)

#define INIT_FIELDS_HELPER_IMPL(type) \
template void VRMLEXPORT VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Private> &field); 
FOR_ALL_VRML_TYPES(INIT_FIELDS_HELPER_IMPL)

#define INIT_EXPOSED_FIELDS_HELPER_IMPL(type) \
template void VRMLEXPORT VrmlNodeTemplate::initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Exposed> &field); 
FOR_ALL_VRML_TYPES(INIT_EXPOSED_FIELDS_HELPER_IMPL)


} // vrml

