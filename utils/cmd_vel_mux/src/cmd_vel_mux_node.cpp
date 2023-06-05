#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "cmd_vel_mux/cmd_vel_mux.hpp"

/** Main node entry point. */
int main(int argc, char ** argv)
{
  // Configures stdout stream for no buffering
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // handle callbacks until shut down
  rclcpp::spin(
    std::make_shared<cmd_vel_mux::CmdVelMux>(
      rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
