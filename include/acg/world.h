#pragma once

#include <string>

#include <box2d/b2_world.h>

#include <acg/agent.h>
#include <acg/obstacle.h>

namespace acg {
class World : public b2World {
 public:
  using b2World::b2World;

  // Agent is a dynamic object on which force can be applied
  void addAgent(const std::string& name, const Agent& agent){};

  // Obstacle is a dynamic/static object
  void addObstacle(const Obstacle& obstacle){};
};
}  // namespace acg
