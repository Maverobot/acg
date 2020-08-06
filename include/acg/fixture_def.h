#pragma once

#include <acg/macro.h>
#include <box2d/box2d.h>

namespace acg {

struct FixtureDef : public b2FixtureDef {
  FixtureDef() = default;

  ACG_MEM_ACCESSOR(def_, b2Shape*, shape)
  ACG_MEM_ACCESSOR(def_, void*, userData);
  ACG_MEM_ACCESSOR(def_, float, friction);
  ACG_MEM_ACCESSOR(def_, float, restitution);
  ACG_MEM_ACCESSOR(def_, float, density);
  ACG_MEM_ACCESSOR(def_, bool, isSensor);
  ACG_MEM_ACCESSOR(def_, b2Filter, filter)

  b2FixtureDef* get() { return &def_; }

  operator b2FixtureDef const *() const { return &def_; }

 private:
  b2FixtureDef def_;
};
}  // namespace acg
