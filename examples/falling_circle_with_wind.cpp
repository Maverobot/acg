#include <acg/acg.h>
#include <acg/body_def.h>
#include <acg/fixture_def.h>
#include <acg/world.h>

#include <iostream>

int main(int, char**) {
  acg::World world(b2Vec2(0.0f, -10.0f));

  // Ground body
  b2Body* groundBody =
      world.CreateBody(acg::BodyDef().type(b2_staticBody).position({0.0f, -10.0f}));

  // Ground fixture
  b2PolygonShape groundBox;
  groundBox.SetAsBox(50.0f, 10.0f);
  groundBody->CreateFixture(&groundBox, 0.0f);

  // Dynamic body
  b2Body* body = world.CreateBody(acg::BodyDef().type(b2_dynamicBody).position({0.0f, 40.0f}));

  // Dynamic body fixture
  b2CircleShape dynamicCircle;
  dynamicCircle.m_radius = 1.0f;

  body->CreateFixture(acg::FixtureDef().shape(&dynamicCircle).density(1.0f).friction(0.3f));

  auto print_state = [&body] {
    body->ApplyForceToCenter(b2Vec2(1.0f, 0), true);
    b2Vec2 position = body->GetPosition();
    float angle = body->GetAngle();
    printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
  };

  acg::ACG sim;
  sim.run(world, print_state);
}
