#include <acg/acg.h>
#include <acg/body_def.h>
#include <acg/fixture_def.h>
#include <acg/world.h>

#include <NamedType/named_type.hpp>

#include <iostream>

using Width = fluent::NamedType<float, struct WidthTag, fluent::FunctionCallable>;
using Length = fluent::NamedType<float, struct LengthTag, fluent::FunctionCallable>;

void addLink(
    acg::World& world,
    b2Body* parent,
    b2Vec2 joint_local_pos,
    Width link_width,
    Length link_length,
    std::function<void(b2Body* const&, b2RevoluteJoint* const&)> link_joint_callback = nullptr,
    bool enable_motor = false,
    bool enable_limit = false) {
  // Link1
  b2PolygonShape shape;
  shape.SetAsBox(link_width / 2, link_length / 2, b2Vec2(0.0f, link_length / 2), 0.0f);

  b2BodyDef bd;
  bd.type = b2_dynamicBody;
  bd.position = parent->GetWorldPoint(joint_local_pos);
  b2Body* link = world.CreateBody(&bd);
  link->CreateFixture(acg::FixtureDef().shape(&shape).density(5.0f));

  // joint1
  b2RevoluteJointDef jd;
  jd.Initialize(parent, link, bd.position);
  jd.motorSpeed = 0.1;
  jd.maxMotorTorque = 10000.0f;
  jd.enableMotor = enable_motor;
  jd.enableLimit = enable_limit;
  b2RevoluteJoint* joint = (b2RevoluteJoint*)world.CreateJoint(&jd);

  if (link_joint_callback) {
    link_joint_callback(link, joint);
  }
}

int main(int, char**) {
  acg::World world(b2Vec2(0.0f, -10.0f));

  b2Body* ground = nullptr;
  b2Body* link1 = nullptr;
  b2RevoluteJoint* joint1 = nullptr;
  b2Body* link2 = nullptr;
  b2RevoluteJoint* joint2 = nullptr;

  // Config
  const float kGroundLength = 80.0f;

  const Length kLink1Length{10.0f};
  const Width kLink1Width{0.5f};
  const b2Vec2 kGroundTJoint1 = {0.0f, 22.0f};

  const Length kLink2Length{10.0f};
  const Width kLink2Width{0.5f};
  const b2Vec2 kLink1TJoint2 = b2Vec2(0.0f, kLink1Length);

  // Parameters
  bool enable_limit = false;
  bool enable_motor = false;
  float motor_speed = 1.0f;

  // Ground
  {
    b2BodyDef bd;
    ground = world.CreateBody(&bd);

    b2EdgeShape shape;
    shape.SetTwoSided(b2Vec2(-kGroundLength / 2, 0.0f), b2Vec2(kGroundLength / 2, 0.0f));
    ground->CreateFixture(acg::FixtureDef().shape(&shape));
  }

  addLink(world, ground, kGroundTJoint1, kLink1Width, kLink1Length,
          [&](auto* const& link, auto* const& joint) {
            link1 = link;
            joint1 = joint;
          });

  addLink(world, link1, kLink1TJoint2, kLink2Width, kLink2Length,
          [&](auto* const& link, auto* const& joint) {
            link2 = link;
            joint2 = joint;
          });

  auto update_ui = [&] {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::Begin("Joint Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::Checkbox("Limit", &enable_limit)) {
      joint1->EnableLimit(enable_limit);
      joint2->EnableLimit(enable_limit);
    }

    if (ImGui::Checkbox("Motor", &enable_motor)) {
      joint1->EnableMotor(enable_motor);
      joint2->EnableMotor(enable_motor);
    }

    if (ImGui::SliderFloat("Speed", &motor_speed, -20.0f, 20.0f, "%.0f")) {
      joint1->SetMotorSpeed(motor_speed);
      joint2->SetMotorSpeed(motor_speed);
    }

    ImGui::End();
  };

  auto print_state = [&] {
    float torque = joint1->GetMotorTorque(60);
    update_ui();
  };

  acg::ACG sim;
  sim.run(world, print_state);
}
