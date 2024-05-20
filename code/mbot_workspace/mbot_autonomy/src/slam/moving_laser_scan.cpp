#include <slam/moving_laser_scan.hpp>
#include <utils/geometric/interpolation.hpp>
#include <mbot_lcm_msgs/lidar_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>

MovingLaserScan::MovingLaserScan(const mbot_lcm_msgs::lidar_t& scan,
                                 const mbot_lcm_msgs::pose2D_t& beginPose,
                                 const mbot_lcm_msgs::pose2D_t& endPose,
                                 int rayStride)
{
    // Ensure a valid scan was received before processing the rays
    if(scan.num_ranges > 0)
    {
        // The stride must be at least one, or else can't iterate through the scan
        if(rayStride < 1)
        {
            rayStride = 1;
        }

        for(int n = 0; n < scan.num_ranges; n += rayStride)
        {
            if(scan.ranges[n] > 0.1f) //all ranges less than a robot radius are invalid
            {
                /// TODO: Do something about those ranges that are equal to the maximum value (assumed 5.5)
                if (scan.ranges[n] >= MAXSCANRANGE)
                {
                    continue;
                }
                mbot_lcm_msgs::pose2D_t rayPose = interpolate_pose_by_time(scan.times[n], beginPose, endPose);

                adjusted_ray_t ray;

                ray.origin = Point<float>(rayPose.x, rayPose.y);
                ray.range = scan.ranges[n];
                /// TODO: do we need to modify the ray theta based on the robot theta? 
                ray.theta = scan.thetas[n] + rayPose.theta;

                adjustedRays_.push_back(ray);
            }
        }
    }
}
 