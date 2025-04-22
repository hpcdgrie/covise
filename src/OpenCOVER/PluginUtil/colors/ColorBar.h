/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _COLOR_BAR_H_
#define _COLOR_BAR_H_

#include "coColorBar.h"

#include <OpenVRUI/coMenu.h>
#include <OpenVRUI/coRowMenu.h>
#include <OpenVRUI/coSubMenuItem.h>
#include <OpenVRUI/coSliderMenuItem.h>
#include <OpenVRUI/coCheckboxMenuItem.h>
#include <OpenVRUI/coButtonMenuItem.h>

#include <cover/coTabletUI.h>
#include <util/coTabletUIMessages.h>
#include <cover/coVRTui.h>

#include <util/coTypes.h>
#include <cover/coInteractor.h>

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// class ColorBar
//
// ColorBar manages a coColorbar, the submenu in which the colorbar appears
// and the button which opens/closes the submenu
//
// Initial version: 2002-02, dr
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// (C) 2002 by Vircinity IT Consulting
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace opencover
{
namespace ui
{
class SpecialElement;
}

class PLUGIN_UTILEXPORT ColorBar: public ui::Owner
{
private:
    vrui::vruiMatrix *floatingMat_ = nullptr;
    coColorBar *colorbar_ = nullptr, *hudbar_ = nullptr;
    ui::Menu *colorsMenu_ = nullptr;
    std::string title_;
    std::string name_;
    std::string species_;
    ui::SpecialElement *uiColorBar_ = nullptr;
    ui::Slider *minSlider_ = nullptr;
    ui::Slider *maxSlider_ = nullptr;
    ui::Slider *stepSlider_ = nullptr;
    ui::Button *autoScale_ = nullptr;
    ui::Action *execute_ = nullptr;
    ui::Slider *center_ = nullptr;
    ui::Slider *compress_ = nullptr;
    ui::Slider *insetCenter_ = nullptr;
    ui::Slider *insetWidth_ = nullptr;
    ui::Slider *opacityFactor_ = nullptr;
    ui::Button *show_ = nullptr;

    opencover::coInteractor *inter_ = nullptr;

    void updateTitle();

   ColorMap map_;

public:

    /** constructor when the colorbar is not to be opened from the pinboard
       *  create create containers, texture and labels
       *  @param colorsButton the button that opens the colorbar
       *  @param moduleMenu the coRowMenu used instead of the pinboard
       *  @param species data species name, currently not displayed
       *  @param min data minimum
       *  @param max data maximum
       *  @param numColors number of different colors in colorbar
       *  @param r red colors
       *  @param g green colors
       *  @param b blue colors
       *  @param a red colors
       */
    ColorBar(ui::Menu *menu);

    /// destructor
    ~ColorBar();

    bool hudVisible() const;
    struct PLUGIN_UTILEXPORT HudPosition
    {
      HudPosition(float hudScale = 1.f);
      void setNumHuds(int numHuds); //changes the offset so that all huds are visible
      osg::Vec3 bottomLeft, hpr;
      float scale;
   private:
      osg::Vec3 m_bottomLeft, m_offset;
    };
    void setHudPosition(const HudPosition &pos);

    /** colorbar update
       *  @param species title bar content
       *  @param min data minimum
       *  @param max data maximum
       *  @param numColors number of different colors in colorbar
       *  @param r red colors
       *  @param g green colors
       *  @param b blue colors
       *  @param a red colors
       */
    void update(const std::string &species, const ColorMap &map);

    /** set name */
    void setName(const char *name);
    void show(bool state);
    /** get name
     *  @return name the name of the colorbar, identical with module name, eg, g, Colors_1
     */
    const char *getName();

    /** parseAttrib
       * @param attrib COLORMAP attribute
       * @param species: get species (the client should delete this pointer)
       * @return map: colormap to be filled
       */
    static ColorMap parseAttrib(const char *attrib, std::string &species);

    void parseAttrib(const char *attrib);
    void setVisible(bool);
    bool isVisible();

    void addInter(opencover::coInteractor *inter);
    void updateInteractor();
};
}
#endif
