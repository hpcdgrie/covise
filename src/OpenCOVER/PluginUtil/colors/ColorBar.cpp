/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include <cover/coVRPluginSupport.h>
#include "ColorBar.h"

#include <util/common.h>
#include <config/CoviseConfig.h>

#include <cover/ui/Menu.h>
#include <cover/ui/VruiView.h>
#include <cover/ui/Action.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>
#include <cover/ui/SpecialElement.h>

#include <cover/VRVruiRenderInterface.h>
#include <cover/coVRMSController.h>
#include <cover/coVRConfig.h>
#include <OpenVRUI/osg/mathUtils.h>

static const char MINMAX[] = "MinMax";
static const char STEPS[] = "numSteps";
static const char AUTOSCALE[] = "autoScales";

static const char V_STEPS[] = "steps";
static const char V_MIN[] = "min";
static const char V_MAX[] = "max";
static const char V_AUTOSCALE[] = "auto_range";
static const char V_CENTER[] = "center";
static const char V_COMPRESS[] = "range_compression";

static const char V_NEST[] = "nest";
static const char V_INSETAUTOCENTER[] = "auto_center";
static const char V_INSETRELATIVE[] = "inset_relative";
static const char V_INSETCENTER[] = "inset_center";
static const char V_INSETWIDTH[] = "inset_width";
static const char V_OPACITYFACTOR[] = "opacity_factor";
static const char V_BLENDWITHMATERIAL[] = "blend_with_material";

using namespace  vrui;

namespace opencover
{

    opencover::ColorMap opencover::interpolateColorMap(const opencover::ColorMap &cm, int numSteps)
    {
        opencover::ColorMap interpolatedMap;
        interpolatedMap.r.resize(numSteps);
        interpolatedMap.g.resize(numSteps);
        interpolatedMap.b.resize(numSteps);
        interpolatedMap.a.resize(numSteps);
        interpolatedMap.samplingPoints.resize(numSteps);

        auto numColors = cm.samplingPoints.size();
        double delta = 1.0 / (numSteps - 1);
        int idx = 0;

        for (int i = 0; i < numSteps - 1; i++)
        {
            double x = i * delta;
            while (cm.samplingPoints[idx + 1] <= x)
            {
                idx++;
                if (idx > numColors - 2)
                {
                    idx = numColors - 2;
                    break;
                }
            }

            double d = (x - cm.samplingPoints[idx]) / (cm.samplingPoints[idx + 1] - cm.samplingPoints[idx]);
            interpolatedMap.r[i] = static_cast<float>((1 - d) * cm.r[idx] + d * cm.r[idx + 1]);
            interpolatedMap.g[i] = static_cast<float>((1 - d) * cm.g[idx] + d * cm.g[idx + 1]);
            interpolatedMap.b[i] = static_cast<float>((1 - d) * cm.b[idx] + d * cm.b[idx + 1]);
            interpolatedMap.a[i] = static_cast<float>((1 - d) * cm.a[idx] + d * cm.a[idx + 1]);
            interpolatedMap.samplingPoints[i] = static_cast<float>(i) / (numSteps - 1);
        }

        interpolatedMap.r[numSteps - 1] = cm.r[numColors - 1];
        interpolatedMap.g[numSteps - 1] = cm.g[numColors - 1];
        interpolatedMap.b[numSteps - 1] = cm.b[numColors - 1];
        interpolatedMap.a[numSteps - 1] = cm.a[numColors - 1];
        interpolatedMap.samplingPoints[numSteps - 1] = 1.0f;

        interpolatedMap.min = cm.min;
        interpolatedMap.max = cm.max;
        interpolatedMap.steps = numSteps;

        return interpolatedMap;
    }

ColorBar::ColorBar(ui::Menu *menu)
: ui::Owner(std::string("ColorBar"), menu)
, species_("NoColors")
{
    colorsMenu_ = menu;

    show_ = new ui::Button("Show", this);
    colorsMenu_->add(show_);
    show_->setVisible(false, ui::View::VR);
    show_->setState(false);
    show_->setCallback([this](bool state){
        show(state);
    });

    if (cover->vruiView)
    {
        uiColorBar_ = new ui::SpecialElement("VruiColorBar", this);

        colorsMenu_->add(uiColorBar_);
        uiColorBar_->registerCreateDestroy(cover->vruiView->typeBit(),
                [this](ui::SpecialElement *se, ui::View::ViewElement *ve){
                auto vve = dynamic_cast<ui::VruiViewElement *>(ve);
                assert(vve);
                colorbar_ = new coColorBar(name_, species_, selectedMap_);
                vve->m_menuItem = colorbar_;
                },
                [this](ui::SpecialElement *se, ui::View::ViewElement *ve){
                auto vve = dynamic_cast<ui::VruiViewElement *>(ve);
                assert(vve);
                assert(!colorbar_ || !vve->m_menuItem || vve->m_menuItem == colorbar_);
                delete colorbar_;
                colorbar_ = nullptr;
                vve->m_menuItem = nullptr;
                });
    }

    autoScale_ = new ui::Button("AutoRange", this);
    colorsMenu_->add(autoScale_);
    autoScale_->setText("Auto range");
    autoScale_->setState(false);
    autoScale_->setCallback([this](bool state){
        if (!inter_)
            return;
        inter_->setBooleanParam(AUTOSCALE, state);
        inter_->setBooleanParam(V_AUTOSCALE, state);
    });

    float diff = (selectedMap_.max - selectedMap_.min) / 2;
    minSlider_ = new ui::Slider("Min", this);
    colorsMenu_->add(minSlider_);
    minSlider_->setBounds(selectedMap_.min-diff, selectedMap_.min+diff);
    minSlider_->setValue(selectedMap_.min);
    minSlider_->setCallback([this](double value, bool released){
        if (!inter_)
            return;
        float dummy = 0.;
        if (inter_->getFloatScalarParam(V_MIN, dummy) != -1)
            inter_->setScalarParam(V_MIN, static_cast<float>(value));
        float minmax[2];
        minmax[0] = value;
        minmax[1] = maxSlider_->value();
        inter_->setVectorParam(MINMAX, 2, minmax);
    });

    maxSlider_ = new ui::Slider("Max", this);
    colorsMenu_->add(maxSlider_);
    maxSlider_->setBounds(selectedMap_.max-diff, selectedMap_.max+diff);
    maxSlider_->setValue(selectedMap_.max);
    maxSlider_->setCallback([this](double value, bool released){
        if (!inter_)
            return;
        float dummy = 0.;
        if (inter_->getFloatScalarParam(V_MAX, dummy) != -1)
            inter_->setScalarParam(V_MAX, static_cast<float>(value));
        float minmax[2];
        minmax[0] = minSlider_->value();
        minmax[1] = value;
        inter_->setVectorParam(MINMAX, 2, minmax);
    });

    center_ = new ui::Slider("Center", this);
    colorsMenu_->add(center_);
    center_->setBounds(0., 1.);
    center_->setValue(0.5);
    center_->setCallback([this](double value, bool released){
        if (!inter_)
            return;
        inter_->setScalarParam(V_CENTER, static_cast<float>(value));
    });

    compress_ = new ui::Slider("RangeCompression", this);
    colorsMenu_->add(compress_);
    compress_->setText("Range compression");
    compress_->setBounds(-1, 1);
    compress_->setValue(0);
    compress_->setCallback([this](double value, bool released){
        if (!inter_)
            return;
        inter_->setScalarParam(V_COMPRESS, static_cast<float>(value));
    });

    stepSlider_ = new ui::Slider("Steps", this);
    colorsMenu_->add(stepSlider_);
    stepSlider_->setBounds(2, selectedMap_.numColors());
    stepSlider_->setIntegral(true);
    stepSlider_->setScale(ui::Slider::Logarithmic);
    stepSlider_->setCallback([this](double value, bool released){
        if(m_callback)
        {
            interpolatedMap_ = opencover::interpolateColorMap(selectedMap_, static_cast<int>(value));
            displayColorMap(interpolatedMap_);
            m_callback(interpolatedMap_);
        }
        
        if (!inter_)
            return;
        int num = static_cast<int>(value);
        inter_->setScalarParam(STEPS, num);
        inter_->setScalarParam(V_STEPS, num);
        //inter_->executeModule();
    });

    insetCenter_ = new ui::Slider("InsetCenter", this);
    colorsMenu_->add(insetCenter_);
    insetCenter_->setText("Inset center");
    insetCenter_->setBounds(0, 1);
    insetCenter_->setValue(0.5);
    insetCenter_->setCallback([this](double value, bool released){
        if (!inter_)
            return;
        inter_->setScalarParam(V_INSETCENTER, static_cast<float>(value));
    });

    insetWidth_ = new ui::Slider("InsetWidth", this);
    colorsMenu_->add(insetWidth_);
    insetWidth_->setText("Inset width");
    insetWidth_->setBounds(0., 1.);
    insetWidth_->setValue(0.1);
    insetWidth_->setCallback([this](double value, bool released){
        if (!inter_)
            return;
        inter_->setScalarParam(V_INSETWIDTH, static_cast<float>(value));
    });

    opacityFactor_ = new ui::Slider("OpacityFactor", this);
    colorsMenu_->add(opacityFactor_);
    opacityFactor_->setText("Opacity factor");
    opacityFactor_->setBounds(0., 1.);
    opacityFactor_->setValue(1.);
    opacityFactor_->setCallback([this](double value, bool released){
        if (!inter_)
            return;
        inter_->setScalarParam(V_OPACITYFACTOR, static_cast<float>(value));
    });

    execute_ = new ui::Action("Execute", this);
    colorsMenu_->add(execute_);
    execute_->setCallback([this](){
        if (inter_)
            inter_->executeModule();
    });

    updateTitle();
}

ColorBar::~ColorBar()
{
    delete colorbar_;
    colorbar_ = nullptr;
    delete hudbar_;
    hudbar_ = nullptr;

    if (inter_)
    {
        inter_->decRefCount();
        inter_ = NULL;
    }
}

bool ColorBar::hudVisible() const
{
    return hudbar_ && hudbar_->isVisible();
}

ColorBar::HudPosition::HudPosition(float hudScale)
{
    if (coVRMSController::instance()->isMaster() && coVRConfig::instance()->numScreens() > 0) {
        const auto &s0 = coVRConfig::instance()->screens[0];
        hpr = s0.hpr;
        auto sz = osg::Vec3(s0.hsize, 0., s0.vsize);
        osg::Matrix mat;
        MAKE_EULER_MAT_VEC(mat, hpr);
        m_bottomLeft = s0.xyz - sz * mat * 0.5;
        auto minsize = std::min(s0.hsize, s0.vsize);
        m_bottomLeft += osg::Vec3(minsize, 0., minsize) * mat * 0.02;
        m_offset = osg::Vec3(s0.vsize/2.5, 0 , 0) * mat * hudScale;
    }
    for (int i=0; i<3; ++i)
    {
        coVRMSController::instance()->syncData(&m_bottomLeft[i], sizeof(m_bottomLeft[i]));
        coVRMSController::instance()->syncData(&hpr[i], sizeof(hpr[i]));
        coVRMSController::instance()->syncData(&m_offset[i], sizeof(m_offset[i]));
    }
    scale = m_offset[0]/480;
    bottomLeft = m_bottomLeft;
}

void ColorBar::HudPosition::setNumHuds(int numHuds)
{
    bottomLeft = m_bottomLeft + m_offset * numHuds;
}

void ColorBar::setHudPosition(const HudPosition &pos)
{
    if (!hudbar_)
        return;

    auto mat = coUIElement::getMatrixFromPositionHprScale(pos.bottomLeft[0], pos.bottomLeft[1], pos.bottomLeft[2], pos.hpr[0], pos.hpr[1], pos.hpr[2], pos.scale);
    auto uie = hudbar_->getUIElement();
    auto vtr = uie->getDCS();
    vtr->setMatrix(mat);
    vruiRendererInterface::the()->deleteMatrix(mat);
}

void ColorBar::updateTitle()
{
    title_ = name_;
    if (species_ != "Color" && !species_.empty())
    {
        title_ += ": " + species_;
    }
    colorsMenu_->setText(title_);
}

void ColorBar::displayColorMap(const ColorMap &map)
{
    if (colorbar_)
        colorbar_->update(map);
    if (hudbar_)
        hudbar_->update(map);
}


void
ColorBar::update(const std::string &species, const ColorMap &map)
{
    selectedMap_ = map;
    interpolatedMap_ = map;
    species_ = species;
    updateTitle();

    if (colorbar_)
        colorbar_->update(map);
    if (hudbar_)
        hudbar_->update(map);

    if (stepSlider_)
    {
        int imin=0, imax=0, ival=0;
        if (!inter_ || inter_->getIntSliderParam(V_STEPS, imin, imax, ival) == -1)
        {
            if (map.numColors() > stepSlider_->max())
            {
                stepSlider_->setBounds(2, map.numColors());
            }
            stepSlider_->setValue(map.numColors());
        }
    }

    if (minSlider_ && maxSlider_)
    {
        float smin = 0.f, smax = 0.f, sval = 0.f;
        if (!inter_
            || inter_->getFloatSliderParam(V_MIN, smin, smax, sval) == -1
            || inter_->getFloatSliderParam(V_MAX, smin, smax, sval) == -1)
        {
            float diff = (map.max - map.min) / 2;

            minSlider_->setBounds(map.min-diff, map.min+diff);
            maxSlider_->setBounds(map.max-diff, map.max+diff);
        }
    }
}

void ColorBar::updateInteractor()
{
    if (!inter_)
        return;

    if (stepSlider_)
    {
        int imin=0, imax=0, ival=0;
        if (inter_->getIntSliderParam(V_STEPS, imin, imax, ival) != -1)
        {
            stepSlider_->setBounds(imin, imax);
            stepSlider_->setValue(ival);
        }
    }

    int num = 0;
    float *minmax = nullptr;
    if (inter_->getFloatVectorParam(MINMAX, num, minmax) != -1 && num == 2)
    {
        if (minSlider_)
        {
            minSlider_->setValue(minmax[0]);
        }

        if (maxSlider_)
        {
            maxSlider_->setValue(minmax[1]);
        }
    }

    float smin = 0.f, smax = 0.f, sval = 0.f;
    if (minSlider_ && inter_->getFloatSliderParam(V_MIN, smin, smax, sval) != -1)
    {
        minSlider_->setBounds(smin, smax);
        minSlider_->setValue(sval);

    }

    if (maxSlider_ && inter_->getFloatSliderParam(V_MAX, smin, smax, sval) != -1)
    {
        maxSlider_->setBounds(smin, smax);
        maxSlider_->setValue(sval);
    }

    int state = 0;
    if (inter_->getBooleanParam(AUTOSCALE, state) == -1)
        inter_->getBooleanParam(V_AUTOSCALE, state);
    if (state)
        autoScale_->setState(true);
    else
        autoScale_->setState(false);

    int nest = 0;
    if (inter_->getBooleanParam(V_NEST, nest) == -1)
        nest = 0;

    float center = 0.;
    if (center_ && inter_->getFloatScalarParam(V_CENTER, center) != -1)
    {
        center_->setValue(center);
        center_->setVisible(nest == 0);
    }
    else
    {
        center_->setVisible(false);
    }

    float compress = 0.;
    if (compress_ && inter_->getFloatScalarParam(V_COMPRESS, compress) != -1)
    {
        compress_->setValue(compress);
        compress_->setVisible(nest == 0);
    }
    else
    {
        compress_->setVisible(false);
    }

    int autocenter = 1;
    if (inter_->getBooleanParam(V_INSETAUTOCENTER, autocenter) == -1)
        autocenter = 1;
    int inset_rel = 0;
    if (inter_->getBooleanParam(V_INSETRELATIVE, inset_rel) == -1)
        inset_rel = 0;

    float insetCenter = 0.;
    if (insetCenter_ && inter_->getFloatScalarParam(V_INSETCENTER, insetCenter) != -1)
    {
        insetCenter_->setValue(insetCenter);
        insetCenter_->setVisible(inset_rel == 1 && nest != 0 && autocenter == 0);
    }
    else
    {
        insetCenter_->setVisible(false);
    }

    float insetWidth = 0.;
    if (insetWidth_ && inter_->getFloatScalarParam(V_INSETWIDTH, insetWidth) != -1)
    {
        insetWidth_->setValue(insetWidth);
        insetWidth_->setVisible(inset_rel == 1 && nest != 0);
    }
    else
    {
        insetWidth_->setVisible(false);
    }

    int blendWithMaterial = 0;
    if (inter_->getBooleanParam(V_BLENDWITHMATERIAL, blendWithMaterial) == -1)
        blendWithMaterial = 0;

    float opacityFactor = 1.;
    if (opacityFactor_ && inter_->getFloatScalarParam(V_OPACITYFACTOR, opacityFactor) != -1)
    {
        opacityFactor_->setValue(opacityFactor);
        opacityFactor_->setVisible(blendWithMaterial != 0);
    }
    else
    {
        opacityFactor_->setVisible(false);
    }
}

void ColorBar::setCallback(const std::function<void(const ColorMap &)> &f)
{
  m_callback = f;
}

void
ColorBar::setName(const char *name)
{
    if (name)
        name_ = name;
    else
        name_.clear();

    updateTitle();
}

void ColorBar::show(bool state)
{
    if (state)
    {
        if(!hudbar_)
        {
            hudbar_ = new coColorBar(name_, species_, interpolatedMap_, false);
            hudbar_->getUIElement()->createGeometry();
        }
        auto vtr = hudbar_->getUIElement()->getDCS();
        VRVruiRenderInterface::the()->getAlwaysVisibleGroup()->addChild(vtr);
    }
    hudbar_->setVisible(state);
    show_->setState(state);
}

const char *
ColorBar::getName()
{
    if (colorbar_)
        return colorbar_->getName();

    return "";
}

void
ColorBar::addInter(coInteractor *inter)
{
    inter->incRefCount();
    if (inter_)
    {
        inter_->decRefCount();
        inter_ = NULL;
    }
    inter_ = inter;

    updateInteractor();
}

ColorMap ColorBar::parseAttrib(const char *attrib, std::string &species)
{
    // convert to a istringstream
    int bufLen = strlen(attrib) + 1;
    istringstream attribs(attrib);
    ColorMap map;
    //fprintf(stderr,"colorsPlugin::addColorbar [%s]\n", name);

    // COLORS_1_OUT_001 pressure min max ncolors 0 r g b rgb rgb ....
    char *s = new char[bufLen];

    attribs.getline(s, bufLen, '\n'); // overread obj name
    attribs.getline(s, bufLen, '\n'); // read species
    species = s;
    delete[] s;

    int v = 0;
    int numColors = 0;
    attribs >> map.min >> map.max >> numColors >> v;

    map.r.resize(numColors);
    map.g.resize(numColors);
    map.b.resize(numColors);
    map.a.resize(numColors);
    map.unit = s;
    for (int i = 0; i < numColors; i++)
    {
        attribs >> map.r[i] >> map.g[i] >> map.b[i] >> map.a[i];
    }
    return map;
}

void ColorBar::parseAttrib(const char *attrib)
{
    update(species_, parseAttrib(attrib, species_));
}

void ColorBar::setVisible(bool visible)
{
    float scale;
    std::string sizeStr = covise::coCoviseConfig::getEntry("AKToolbar.Scale");
    if (!sizeStr.empty())
        sscanf(sizeStr.c_str(), "%5f", &scale);
    else
        scale = 0.2;
    //colorsMenu_->setScale(scale);
    colorsMenu_->setVisible(visible);
}

bool ColorBar::isVisible()
{
    return colorsMenu_->visible();
}

}
