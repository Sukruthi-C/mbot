#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int gen_maze_wpts(mbot_lcm_msgs::path2D_t& path,mbot_lcm_msgs::pose2D_t& nextPose, int n){
    nextPose.x += 0.61f;
    path.path[n] = nextPose;
    n++;

    nextPose.y += -0.61f;
    path.path[n] = nextPose;
    n++;

    nextPose.x += 0.61f;
    path.path[n] = nextPose;
    n++;

    nextPose.y += 1.22f;
    path.path[n] = nextPose;
    n++;

    nextPose.x += 0.61f;
    path.path[n] = nextPose;
    n++;

    nextPose.y += -1.22f;
    path.path[n] = nextPose;
    n++;

    nextPose.x += 0.61f;
    path.path[n] = nextPose;
    n++;

    nextPose.y += 0.61f;
    path.path[n] = nextPose;
    n++;

    nextPose.x += 0.61f;
    path.path[n] = nextPose;
    n++;
    return n;
}

int gen_test_wpts(mbot_lcm_msgs::path2D_t& path,mbot_lcm_msgs::pose2D_t& nextPose, int n){
    nextPose.x += 0.61f;
    path.path[n] = nextPose;
    n++;

    nextPose.x += -0.61f;
    path.path[n] = nextPose;
    n++;
    return n;
}




int main(int argc, char** argv)
{
    int numNodes = 100;
    int n = 0;

    if(argc > 1)
    {
        numNodes = std::atoi(argv[1]);
    }

    std::cout << "performing checkpoint1 maze.\n";

    mbot_lcm_msgs::path2D_t path;
    path.path.resize(numNodes);

    mbot_lcm_msgs::pose2D_t nextPose;
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[n] = nextPose;
    n++;

    path.path[n] = nextPose;
    n++;



    n = gen_maze_wpts(path, nextPose, n);

    // n = gen_test_wpts(path, nextPose, n);
    


    path.path_length = n;
    for (int i = 0; i<n; i++){
        std::cout<<path.path[i].x<<" "<<path.path[i].y<<"\n";
    }

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
