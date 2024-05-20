#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
#include <cmath>

using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.
    std::vector<adjusted_ray_t>::const_iterator ptr; 
    for (ptr = movingScan.begin(); ptr < movingScan.end(); ptr++) 
    {
        scoreEndpoint(*ptr, map);
        scoreRay(*ptr, map);
    }
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  
    Point<float> world_p0 = ray.origin;
    Point<float> world_p1;
    world_p1.x = world_p0.x + ray.range * cos(ray.theta);
    world_p1.y = world_p0.y + ray.range * sin(ray.theta);
    Point<int> p1 = global_position_to_grid_cell(world_p1, map);
    /// TODO: 1. if we need to use log for Kmissodds
    /// 2. if this is the right way to modify the parameters.
    // map(p1.x, p1.y) = int(min(float(map(p1.x, p1.y)) + log(float(kHitOdds_)), 127.));  
    if (((float)map(p1.x, p1.y) + log((float)kHitOdds_)  < 127) && ((float)map(p1.x, p1.y) + log((float)kHitOdds_)  > -127)) 
    {
        map(p1.x, p1.y) = (int8_t)(map(p1.x, p1.y) + log(kHitOdds_));    
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    std::vector<Point<int>> cells = bresenham(ray, map);

    for (int i = 0; i < cells.size() - 1; i++)
    {
        Point<int> cell = cells[i];
        /// TODO: if this is the right way to modify the parameters.
        if (((float)map(cell.x, cell.y) - log((float)kMissOdds_)  < 127) && ((float)map(cell.x, cell.y) - log((float)kMissOdds_)  > -127)) 
        {
            // std::cerr << "[DEBUG]: kmissodds: " << float(kMissOdds_) << " log(kmissodds:) " << log(kMissOdds_) << std::endl;
            map(cell.x, cell.y) = (int8_t)std::floor(map(cell.x, cell.y) - log(kMissOdds_));
            // std::cerr << "[DEBUG]: " << (int32_t) map(cell.x, cell.y) << std::endl;
        }
        // Mapping::setLogOdds(cell.x, cell.y, map(cell.x, cell.y) + log(Mapping.kMissOdds_));
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    // 
    std::vector<Point<int>> cells;
    Point<float> world_p0 = ray.origin;
    Point<float> world_p1;
    world_p1.x = world_p0.x + ray.range * cos(ray.theta);
    world_p1.y = world_p0.y + ray.range * sin(ray.theta);
    Point<int> p0 = global_position_to_grid_cell(world_p0, map);
    Point<int> p1 = global_position_to_grid_cell(world_p1, map);

    int dx = abs(p1.x - p0.x);
    int dy = abs(p1.y - p0.y);    
    int sx = (p1.x - p0.x) / dx;
    int sy = (p1.y - p0.y) / dy;
    int err = dx - dy;
    int x = p0.x;
    int y = p0.y;
    cells.push_back(Point<int>(x,y));
    while (x != p1.x || y != p1.y)
    {
        int e2 = 2 * err;
        if (e2 >= -dy)
        {
            err = err - dy;
            x = x + sx;
        }
        if (e2 <= dx)
        {
            err = err + dx;
            y = y + sy;
        }
        cells.push_back(Point<int>(x, y));
    }
    return cells;    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}
