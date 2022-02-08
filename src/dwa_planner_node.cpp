#include "dwa_planner/dwa_planner.h"
#include "dwa_planner/dwa_planner_for_mecanum.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dwa_planner");

    if (argc == 2 && std::string(argv[1]) == "--help") {
        ROS_INFO("Usage: rosrun dwa_planner dwa_planner_node [--help | --use-mecanum-model]");
        return 0;
    } else if (argc == 2 && std::string(argv[1]) == "--use-mecanum-model") {
        ROS_INFO("=== Using mecanum model ===");
        dwa_planner::DWAPlannerForMecanum planner;
        planner.process();
    } else {
        ROS_INFO("=== Using differential model ===");
        dwa_planner::DWAPlanner planner;
        planner.process();
    }
    return 0;
}
