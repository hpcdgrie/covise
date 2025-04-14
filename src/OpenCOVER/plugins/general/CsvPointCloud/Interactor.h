#ifndef COVISE_CSV_POINT_CLOUD_INTERACTOR_H
#define COVISE_CSV_POINT_CLOUD_INTERACTOR_H

#include <cover/coInteractor.h>
#include <PluginUtil/colors/coColorMap.h>
#include <functional>
namespace opencover
{
class RenderObject;
}

class CsvInteractor : public opencover::coInteractor
{
public:
    CsvInteractor() = default;
    void setColorMap(const covise::ColorMap &cm);
    covise::ColorMap getColorMap() const;
    void setName(const std::string &name);
    void setMinMax(float min, float max);
    virtual ~CsvInteractor() = default;

    /// returns true, if Interactor comes from same Module as intteractor i;
    bool isSameModule(coInteractor *i) const override { return dynamic_cast<CsvInteractor *>(i) != nullptr; }

    /// returns true, if Interactor is exactly the same as interactor i;
    bool isSame(coInteractor *i) const override { return i == this; }

    /// execute the Module
    void executeModule() override {}

    /// copy the Module to same host
    void copyModule() override {}

    /// copy the Module to same host and execute the copied one
    void copyModuleExec() override {}

    /// delete the Module
    void deleteModule() override {}

    // --- all getParameter Functions
    //       - return -1 on fail (illegal type requested), 0 if ok
    //       - only work for coFeedback created parameter messages
    //       - do not change the value fields if type incorrect

    int getBooleanParam(const std::string &paraName, int &value) const override {return 0;}
    int getIntScalarParam(const std::string &paraName, int &value) const override;
    int getFloatScalarParam(const std::string &paraName, float &value) const override;
    int getIntSliderParam(const std::string &paraName, int &min, int &max, int &val) const override;
    int getFloatSliderParam(const std::string &paraName, float &min, float &max, float &val) const override;
    int getIntVectorParam(const std::string &paraName, int &numElem, int *&val) const override{ return 0; }
    int getFloatVectorParam(const std::string &paraName, int &numElem, float *&val) const override {return 0;}
    int getStringParam(const std::string &paraName, const char *&val) const override {return 0;}
    int getChoiceParam(const std::string &paraName, int &num, char **&labels, int &active) const override {return 0;}
    int getFileBrowserParam(const std::string &paraName, char *&val) const override {return 0;}

    // --- set-Functions:

    /// set Boolean Parameter
    void setBooleanParam(const char *name, int val) override {}

    /// set float scalar parameter
    void setScalarParam(const char *name, float val) override {}

    /// set int scalar parameter
    void setScalarParam(const char *name, int val) override;

    /// set float slider parameter
    void setSliderParam(const char *name, float min, float max, float value) override {}

    /// set int slider parameter
    void setSliderParam(const char *name, int min, int max, int value) override {}

    /// set float Vector Param
    void setVectorParam(const char *name, int numElem, float *field) override;
    void setVectorParam(const char *name, float u, float v, float w) override {}

    /// set int vector parameter
    void setVectorParam(const char *name, int numElem, int *field) override {}
    void setVectorParam(const char *name, int u, int v, int w) override {}

    /// set string parameter
    void setStringParam(const char *name, const char *val) override {}

    /// set choice parameter, pos starts with 0
    void setChoiceParam(const char *name, int pos) override {}
    void setChoiceParam(const char *name, int num, const char *const *list, int pos) override {}

    /// set browser parameter
    void setFileBrowserParam(const char *name, const char *val) override {}

    // name of the covise data object which has feedback attributes
    const char *getObjName() override { return "CsvPointCloud"; }

    // the covise data object which has feedback attributes
    opencover::RenderObject *getObject() override { return nullptr; }

    // Fake that this adresses the ColorBars plugin
    const char *getPluginName() override { return "ColorBars"; };

    // just a a random name
    const char *getModuleName() override { return ""; }

    // get the instance number of the module which created the data object
    int getModuleInstance() override { return 0; }

    // get the hostname of the module which created the data object
    const char *getModuleHost() override { return "localhost"; }

    // -- The following functions only works for coFeedback attributes
    /// Get the number of Parameters
    int getNumParam() const override { return 0; }

    /// Get the number of User Strings
    int getNumUser() const override { return 0; };

    // get a User-supplied string
    const char *getString(unsigned int i) const override;

private:
    std::string m_name;
    covise::ColorMap m_colorMap;
    float m_minSlider = 0, m_maxSlider = 0;
    float m_min = 0, m_max = 0;
    int m_numColorSteps = 0;
};

#endif // COVISE_CSV_POINT_CLOUD_INTERACTOR_H
