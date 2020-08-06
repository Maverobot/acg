#pragma once

#include <acg/macro.h>
#include <box2d/box2d.h>

namespace acg {

struct BodyDef {
  BodyDef() = default;
  virtual ~BodyDef() = default;

  ACG_MEM_ACCESSOR(def_, b2BodyType, type);
  ACG_MEM_ACCESSOR(def_, b2Vec2, position);
  ACG_MEM_ACCESSOR(def_, float, angle);
  ACG_MEM_ACCESSOR(def_, b2Vec2, linearVelocity);
  ACG_MEM_ACCESSOR(def_, float, angularVelocity);
  ACG_MEM_ACCESSOR(def_, float, linearDamping);
  ACG_MEM_ACCESSOR(def_, float, angularDamping);
  ACG_MEM_ACCESSOR(def_, bool, allowSleep);
  ACG_MEM_ACCESSOR(def_, bool, awake);
  ACG_MEM_ACCESSOR(def_, bool, fixedRotation);
  ACG_MEM_ACCESSOR(def_, bool, bullet);
  ACG_MEM_ACCESSOR(def_, bool, enabled);
  ACG_MEM_ACCESSOR(def_, void*, userData);
  ACG_MEM_ACCESSOR(def_, float, gravityScale);

  b2BodyDef* get() { return &def_; }

  operator b2BodyDef const *() const { return &def_; }

 private:
  b2BodyDef def_;
};
}  // namespace acg
