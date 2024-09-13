#ifndef TOOLMACHINETOOLCHANGER_H
#define TOOLMACHINETOOLCHANGER_H

#include <cover/ui/Menu.h>
#include <cover/ui/VectorEditField.h>

#include <cover/ui/Slider.h>
#include <cover/ui/Action.h>
struct AnimationManagerFinder;
class Arm;
class ToolChanger{
public:
    ToolChanger(opencover::ui::Menu *menu);
    ~ToolChanger(); //do not implement destructor in header because it can't delete m_animationFinder
    void update();
private:

    void positionArms(float offset);

    std::vector<std::unique_ptr<Arm>> m_arms;
    bool m_update = false;
    int m_currentArm = 0;
    opencover::ui::EditField *m_anim;
    opencover::ui::Slider *m_maxSpeed;
    float m_speed = 0;
    float m_offset = 0;
    opencover::ui::Action *m_action;
    bool m_changeTool = false;
    Arm * m_selectedArm = nullptr;
    float m_distanceToSeletedArm = 0;
    bool decellerate = false;
};

#endif // TOOLMACHINETOOLCHANGER_H
