#include <acg/acg.h>
#include <acg/articulation.h>
#include <acg/body_def.h>
#include <acg/fixture_def.h>
#include <acg/options.h>
#include <acg/world.h>

#include <iostream>

int main(int, char**) {
  acg::World world(b2Vec2(0.0f, -10.0f));

  b2Body* ground = nullptr;
  b2Body* link1 = nullptr;
  b2RevoluteJoint* joint1 = nullptr;
  b2Body* link2 = nullptr;
  b2RevoluteJoint* joint2 = nullptr;

  // Parameters
  const float kGroundLength = 80.0f;
  bool enable_limit = false;
  bool enable_motor = false;
  float motor_speed = 1.0f;

  const auto kLink1Options = acg::LinkOptions(Width{0.5f}, Length{10.0f});
  const auto kJoint1Options =
      acg::JointOptions().enable_motor(enable_motor).motor_speed(motor_speed);
  const b2Vec2 kGroundTJoint1 = {0.0f, 22.0f};

  const auto kLink2Options = acg::LinkOptions(Width{0.5f}, Length{10.0f});
  const auto kJoint2Options =
      acg::JointOptions().enable_motor(enable_motor).motor_speed(motor_speed);
  const b2Vec2 kLink1TJoint2 = b2Vec2(0.0f, kLink1Options.link_length());

  // Ground
  {
    b2BodyDef bd;
    ground = world.CreateBody(&bd);

    b2EdgeShape shape;
    shape.SetTwoSided(b2Vec2(-kGroundLength / 2, 0.0f), b2Vec2(kGroundLength / 2, 0.0f));
    ground->CreateFixture(acg::FixtureDef().shape(&shape));
  }

  addLinkWithJoint(world, ground, kGroundTJoint1, kLink1Options, kJoint1Options,
                   [&](auto* const& link, auto* const& joint) {
                     link1 = link;
                     joint1 = joint;
                   });

  addLinkWithJoint(world, link1, kLink1TJoint2, kLink2Options, kJoint1Options,
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
