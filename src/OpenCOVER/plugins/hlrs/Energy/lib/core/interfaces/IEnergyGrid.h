#ifndef _CORE_INTERFACES_IENERGYGRID_H
#define _CORE_INTERFACES_IENERGYGRID_H

#include "IColorable.h"
#include "IDrawables.h"
#include "ITimedependable.h"

namespace core::interface {
class IEnergyGrid : public IDrawables, public IColorable, public ITimedependable {
 public:
  virtual ~IEnergyGrid() = default;
  virtual void update() = 0;
};
}  // namespace core::interface

#endif
