#include <mbot/mbot_channels.h>
#include <unistd.h>
#include <utils/lcm_config.h>
#include <cmath>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>

int main(int argc, char** argv) {
  using mbot_lcm_msgs::path2D_t;
  using mbot_lcm_msgs::pose2D_t;

  std::cout << "Commanding robot to drive the checkpoint 1 path\n";

  path2D_t path;

  constexpr float SEGMENT_LENGTH = 0.61f;
  constexpr float EPS = 1e-3;

  path.path.push_back({0, 0, 0, 0});
  path.path.push_back({0, SEGMENT_LENGTH, 0, 0});
  path.path.push_back({0, SEGMENT_LENGTH, -SEGMENT_LENGTH, -M_PI_2 + EPS});
  path.path.push_back({0, 2 * SEGMENT_LENGTH, -SEGMENT_LENGTH, 0});
  path.path.push_back({0, 2 * SEGMENT_LENGTH, SEGMENT_LENGTH, M_PI_2 - EPS});
  path.path.push_back({0, 3 * SEGMENT_LENGTH, SEGMENT_LENGTH, 0});
  path.path.push_back({0, 3 * SEGMENT_LENGTH, -SEGMENT_LENGTH, -M_PI_2 + EPS});
  path.path.push_back({0, 4 * SEGMENT_LENGTH, -SEGMENT_LENGTH, 0});
  path.path.push_back({0, 4 * SEGMENT_LENGTH, -SEGMENT_LENGTH, 0});
  path.path.push_back({0, 4 * SEGMENT_LENGTH, 0, M_PI_2 - EPS});
  path.path.push_back({0, 5 * SEGMENT_LENGTH, 0, 0});

  path.path_length = path.path.size();

  lcm::LCM lcmInstance(MULTICAST_URL);
  std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
  lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
  sleep(1);

  return 0;
}
