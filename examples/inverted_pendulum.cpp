#include <acg/acg.h>
#include <acg/body_def.h>
#include <acg/fixture_def.h>
#include <acg/world.h>

#include <iostream>

int main(int, char**) {
  acg::World world(b2Vec2(0.0f, -10.0f));

  b2Body* ground = NULL;
  b2Body* link1 = NULL;
  b2RevoluteJoint* joint1;

  // Parameters
  bool enable_limit = false;
  bool enable_motor = false;
  float motor_speed = 1.0f;

  // Ground
  {
    b2BodyDef bd;
    ground = world.CreateBody(&bd);

    b2EdgeShape shape;
    shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
    ground->CreateFixture(acg::FixtureDef().shape(&shape));
  }

  {
    // Link1
    b2PolygonShape shape;
    shape.SetAsBox(0.25f, 5.0f, b2Vec2(0.0f, 5.0f), 0.0f);

    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(0.0f, 20.0f);
    link1 = world.CreateBody(&bd);
    link1->CreateFixture(acg::FixtureDef().shape(&shape).density(5.0f));

    // joint1
    b2RevoluteJointDef jd;
    jd.Initialize(ground, link1, b2Vec2(0.0f, 20.0f));
    jd.motorSpeed = motor_speed;
    jd.maxMotorTorque = 10000.0f;
    jd.enableMotor = enable_motor;
    jd.enableLimit = enable_limit;
    joint1 = (b2RevoluteJoint*)world.CreateJoint(&jd);
  }

  {
    // Link2
    b2PolygonShape shape;
    shape.SetAsBox(0.25f, 5.0f, b2Vec2(0.0f, 5.0f), 0.0f);

    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(0.0f, 30.0f);
    b2Body* body = world.CreateBody(&bd);
    body->CreateFixture(acg::FixtureDef().shape(&shape).density(5.0f));

    // joint2
    b2RevoluteJointDef jd;
    jd.Initialize(link1, body, b2Vec2(0.0f, 30.0f));
    jd.motorSpeed = motor_speed;
    jd.maxMotorTorque = 10000.0f;
    jd.enableMotor = enable_motor;
    jd.enableLimit = enable_limit;
    joint1 = (b2RevoluteJoint*)world.CreateJoint(&jd);
  }

  auto update_ui = [&] {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::Begin("Joint Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::Checkbox("Limit", &enable_limit)) {
      joint1->EnableLimit(enable_limit);
    }

    if (ImGui::Checkbox("Motor", &enable_motor)) {
      joint1->EnableMotor(enable_motor);
    }

    if (ImGui::SliderFloat("Speed", &motor_speed, -20.0f, 20.0f, "%.0f")) {
      joint1->SetMotorSpeed(motor_speed);
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
