#pragma once

#include <acg/fixture_def.h>
#include <acg/options.h>
#include <acg/world.h>

namespace acg {

inline void addLinkWithJoint(
    acg::World& world,
    b2Body* parent,
    b2Vec2 joint_local_pos,
    const acg::LinkOptions& link_options,
    const acg::JointOptions& joint_options,
    std::function<void(b2Body* const&, b2RevoluteJoint* const&)> link_joint_callback = nullptr) {
  // Link1
  b2PolygonShape shape;
  shape.SetAsBox(link_options.link_width() / 2, link_options.link_length() / 2,
                 b2Vec2(0.0f, link_options.link_length() / 2), 0.0f);

  b2BodyDef bd;
  bd.type = b2_dynamicBody;
  bd.position = parent->GetWorldPoint(joint_local_pos);
  b2Body* link = world.CreateBody(&bd);
  link->CreateFixture(acg::FixtureDef().shape(&shape).density(link_options.density()));

  // joint1
  b2RevoluteJointDef jd;
  jd.Initialize(parent, link, bd.position);
  jd.motorSpeed = joint_options.motor_speed();
  jd.maxMotorTorque = joint_options.max_motor_torque();
  jd.enableMotor = joint_options.enable_motor();
  jd.enableLimit = joint_options.enable_limit();
  jd.upperAngle = joint_options.upper_angle();
  jd.lowerAngle = joint_options.lower_angle();
  b2RevoluteJoint* joint = (b2RevoluteJoint*)world.CreateJoint(&jd);

  if (link_joint_callback) {
    link_joint_callback(link, joint);
  }
}

}  // namespace acg
