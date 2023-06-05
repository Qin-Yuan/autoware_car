#include <cmath>
#include "geometry_msgs/msg/pose.hpp"
namespace bt_utils
{
    class BTUtils
    {
    public:
        bool check_distance(
            const geometry_msgs::msg::Pose & current_pose,
            const geometry_msgs::msg::Pose & desired_pose,
            const float & threshold)
        {
            float error_x = current_pose.position.x - desired_pose.position.x;
            float error_y = current_pose.position.y - desired_pose.position.y;

            float distance = hypot(error_x,error_y);
            std::cout <<"distance is" << distance <<std::endl;
            return (distance < threshold)? true : false;

        }


    };
}