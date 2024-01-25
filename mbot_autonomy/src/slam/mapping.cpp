#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
// , kHitOdds_(3.5f)
// , kMissOdds_(1.0f)
, initialized_(false)
{
    // std::cout << "hitodds: " << kHitOdds_ << "missodds: " << kMissOdds_ << "\n";
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: (Done) Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.

    int raySize = movingScan.size();
    std::vector<Point<int>> rayInGrid;
    Point<int> pointInGrid;
    for (size_t i = 0; i < raySize; i++)
    {
        // rayInGrid = std::move(bresenham(movingScan[i], map));
        rayInGrid = bresenham(movingScan[i], map);
        int pointSize = rayInGrid.size();
        // std::cout << pointSize << " ";

        // for cells it passed through.
        for (size_t j = 0; j < pointSize-1; j++)
        {
            pointInGrid = rayInGrid[j];
            if (map.isCellInGrid(pointInGrid.x, pointInGrid.y))
            {
                CellOdds odd = map.logOdds(pointInGrid.x, pointInGrid.y);
                if (odd - kMissOdds_ < -127)
                    map.setLogOdds(pointInGrid.x, pointInGrid.y, -127);
                else
                    map.setLogOdds(pointInGrid.x, pointInGrid.y, odd - 1.0f);
                // map(pointInGrid.x, pointInGrid.y) -= kMissOdds_;
            }
        }

        // for cells the laser hit
        pointInGrid = rayInGrid[pointSize-1];
        if (map.isCellInGrid(pointInGrid.x, pointInGrid.y))
        {
            // map(pointInGrid.x, pointInGrid.y) += kHitOdds_;
            CellOdds odd = map.logOdds(pointInGrid.x, pointInGrid.y);
            if (odd + kHitOdds_ > 127)
                map.setLogOdds(pointInGrid.x, pointInGrid.y, 127);
            else
                map.setLogOdds(pointInGrid.x, pointInGrid.y, odd + 3.5f);
        }
    }
    std::cout << "\n" << "updated: (" << pointInGrid.x << ", " << pointInGrid.y << ")\n";
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  
    
    
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    
    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: (Done) Implement the Bresenham's line algorithm to find cells touched by the ray.
    Point<int> ray_origin_gird = global_position_to_grid_cell(ray.origin, map);
    Point<float> ray_end;
    ray_end.x = ray.origin.x + ray.range * cosf32(ray.theta);
    ray_end.y = ray.origin.y + ray.range * sinf32(ray.theta);
    Point<int> ray_end_gird = global_position_to_grid_cell(ray_end, map);
    int x1 = ray_origin_gird.x;
    int y1 = ray_origin_gird.y;
    int x2 = ray_end_gird.x;
    int y2 = ray_end_gird.y;
    std::vector<Point<int>> result;
    // std::cout << ray.origin.x << " " << ray.origin.y << " " << ray_end.x << " " << ray_end.y << "\n";
    // std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << "\n";

    // Find Delta
    float dx = x2-x1;
    float dy = y2-y1;
    // Find Signs
    int sx = (dx>=0) ? 1 : (-1);
    int sy = (dy>=0) ? 1 : (-1);
    // Get Initial Points
    float x = x1;
    float y = y1;
    // Flag to check if swapping happens
    int isSwaped = 0;
    // Swap if needed
    if(abs(dy) > abs(dx))
    {
        // swap dx and dy
        float tdx = dx;
        dx = dy;
        dy = tdx;
        isSwaped = 1;
    }
    // Decision parameter
    float p = 2*(abs(dy)) - abs(dx);
    //Print Initial Point
    // putpixels(x,y);
    result.push_back(Point<int>(x, y));
    // Loop for dx times
    for(int i = 0; i < abs(dx);i++)
    {
        // Depending on decision parameter
        if(p < 0)
        {
            if(isSwaped == 0){
                x = x + sx;
                // putpixels(x,y);
                result.push_back(Point<int>(x, y));
            }
            else{
                y = y + sy;
                result.push_back(Point<int>(x, y));
                // putpixels(x,y);
            }
            p = p + 2*abs(dy);
        }
        else
        {
            x = x + sx;
            y = y + sy;
            // putpixels(x,y);
            result.push_back(Point<int>(x, y));
            p = p + 2*abs(dy) - 2*abs(dx);
        }
    }
    return result;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}
