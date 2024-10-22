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
        VrmlSFMatrix
    > VrmlTypesTuple;


template <typename T, typename Tuple, std::size_t Index = 0>
struct TypeToEnumHelper {
    static VrmlField::VrmlFieldType value() {
        if constexpr (Index < std::tuple_size_v<Tuple>) {
            if constexpr (std::is_same_v<T, std::tuple_element_t<Index, Tuple>>) {
                return static_cast<VrmlField::VrmlFieldType>(Index + 1);
            } else {
                return TypeToEnumHelper<T, Tuple, Index + 1>::value();
            }
        } else {
            return VrmlField::VrmlFieldType::NO_FIELD;
        }
    }
};

// Function to map type to enumeration value
template<typename T>
VrmlField::VrmlFieldType toEnumType(const T *t) {
    return TypeToEnumHelper<T, VrmlTypesTuple>::value();
}


// Helper to extract types from tuple and create a variant with pointers to those types
template<typename Tuple, std::size_t... Indices>
auto tuple_to_variant_ptr_impl(std::index_sequence<Indices...>) {
    return std::variant<std::add_pointer_t<std::tuple_element_t<Indices, Tuple>>...>{};
}

template<typename Tuple>
using tuple_to_variant_ptr = decltype(tuple_to_variant_ptr_impl<Tuple>(std::make_index_sequence<std::tuple_size_v<Tuple>>{}));


class VrmlNodeChildTemplateImpl
{

public:
    VrmlNodeChildTemplateImpl(VrmlNodeTemplate *nodeChild)
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
    template<typename T>
    const T* getField(const VrmlField &fieldValue, const T *t){
        const T* val = dynamic_cast<const T*>(&fieldValue);
        if(!val)
            std::cerr << "Field type mismatch" << std::endl;
        return val;
    }
    

public:
    void setField(const char *fieldName, const VrmlField &fieldValue) {
        auto it = m_fields.find(fieldName);
        if(it == m_fields.end()){
            m_nodeChild->setField(fieldName, fieldValue);
            return;
        }
        auto& field = it->second;
        std::visit([&fieldValue, this](auto&& arg){
            *arg = *getField(fieldValue, arg);
        }, field.type);
        field.initialized = true;
        if(field.updateCb){
            field.updateCb();
        }
    }
    
    template<typename T>
    void registerField(const std::string& name, T &field, const std::function<void()> &updateCb =  std::function<void()>{}){
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

    template<typename T>
    T* copy(const T* other){
        return new T(*other);
    }

    // use new pointers with this inital values of other
    VrmlNodeChildTemplateImpl(const VrmlNodeChildTemplateImpl& other)
    : m_fields(other.m_fields)
    {
        for(auto& [name, field] : m_fields){
            std::visit([this, &field](auto&& arg){
                arg = copy(arg);
            }, field.type);
        }
    }

    template<typename T>
    void deleter(T* t){
        delete t;
    }

};

VrmlNodeTemplate::VrmlNodeTemplate(VrmlScene *scene)
: VrmlNode(scene)
, m_impl(std::make_unique<VrmlNodeChildTemplateImpl>(this)) {}

// use new pointers with this inital values of other
VrmlNodeTemplate::VrmlNodeTemplate(const VrmlNodeTemplate& other)
: VrmlNode(other)
, m_impl(std::make_unique<VrmlNodeChildTemplateImpl>(*other.m_impl)) {}

VrmlNodeTemplate::~VrmlNodeTemplate() = default;

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

template<typename T>
void VrmlNodeTemplate::registerField(const std::string& name, T &field, const std::function<void()> &updateCb)
{
    return m_impl->registerField<T>(name, field, updateCb);
}

template <typename T>
void registerField(VrmlNodeTemplate *node, const std::string &name, T &field) {
    node->registerField(name, field);
}

template <typename T>
void addField(VrmlNodeType *t, const std::string &name, T &field) {
    t->addField(name.c_str(), toEnumType<std::remove_reference_t<T>>());
    std::cerr << "adding field " << name <<  " whith type " << toEnumType<std::remove_reference_t<T>>() <<  std::endl;
}

template <typename T>
void addExposedField(VrmlNodeType *t, const std::string &name, T &field) {
    t->addExposedField(name.c_str(), toEnumType<std::remove_reference_t<T>>());
    std::cerr << "adding exposed field " << name <<  " whith type " << toEnumType<std::remove_reference_t<T>>() <<  std::endl;
}

template <typename T>
void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<T, FieldAccessibility::Private> &field)
{
    if (node) 
        registerField(node, field.name, field.value);
    if (t) 
        addField(t, field.name, field.value);
}

template <typename T>
void initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<T, FieldAccessibility::Exposed> &field)
{
    if (node) 
        registerField(node, field.name, field.value);
    if (t) 
        addExposedField(t, field.name, field.value);
}

#define VRMLNODECHILD2_TEMPLATE_IMPL(type) \
template void VRMLEXPORT VrmlNodeTemplate::registerField<type>(const std::string& name, type &field, const std::function<void()> &updateCb);
FOR_ALL_VRML_TYPES(VRMLNODECHILD2_TEMPLATE_IMPL)

#define TO_VRML_FIELD_TYPES_IMPL(type) \
template VrmlField::VrmlFieldType VRMLEXPORT toEnumType(const type *t);
FOR_ALL_VRML_TYPES(TO_VRML_FIELD_TYPES_IMPL)

#define INIT_FIELDS_HELPER_IMPL(type) \
template void VRMLEXPORT initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Private> &field); 
FOR_ALL_VRML_TYPES(INIT_FIELDS_HELPER_IMPL)

#define INIT_EXPOSED_FIELDS_HELPER_IMPL(type) \
template void VRMLEXPORT initFieldsHelperImpl(VrmlNodeTemplate *node, VrmlNodeType *t, const NameValueStruct<type, FieldAccessibility::Exposed> &field); 
FOR_ALL_VRML_TYPES(INIT_EXPOSED_FIELDS_HELPER_IMPL)

} // vrml

