#include <acg/acg.h>
#include <acg/articulation.h>
#include <acg/body_def.h>
#include <acg/fixture_def.h>
#include <acg/options.h>
#include <acg/world.h>

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>

namespace {
// Source: https://stackoverflow.com/a/45067593/6018272
inline bool kbhit() {
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);

  tcsetattr(0, TCSANOW, &term);

  return byteswaiting > 0;
}

// Source: https://stackoverflow.com/a/912796/6018272
inline char getch() {
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
    perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
    perror("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0)
    perror("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
    perror("tcsetattr ~ICANON");
  return (buf);
}
}  // namespace

int main(int argc, char* argv[]) {
  acg::World world(b2Vec2(0.0f, 0.0f));

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
    static b2Vec2 force(0.f, 0.f);
    if (kbhit()) {
      if (auto key = getch(); key != -1) {
        switch (key) {
          case 119:  // w
            force.y += 1.0f;
            break;
          case 115:  // s
            force.y -= 1.0f;
            break;
          case 97:  // a
            force.x -= 1.0f;
            break;
          case 100:  // d
            force.x += 1.0f;
            break;
        }
      }
    } else {
      force.SetZero();
    }

    printf("force: [%f, %f]\n", force.x, force.y);

    body->ApplyForceToCenter(force, true);
    b2Vec2 position = body->GetPosition();
    float angle = body->GetAngle();
  };

  acg::ACG sim;
  sim.run(world, print_state);
  return 0;
}
