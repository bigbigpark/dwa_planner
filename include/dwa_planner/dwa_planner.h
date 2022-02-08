#ifndef __DWA_PLANNER_H
#define __DWA_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

namespace dwa_planner {
struct State {
    State(double _x, double _y, double _yaw, double _velocity_x, double _velocity_y, double _yawrate);
    double x; // robot position x
    double y; // robot posiiton y
    double yaw; // robot orientation yaw
    double velocity_x; // robot linear velocity x
    double velocity_y; // robot linear velocity y
    double yawrate; // robot angular velocity
};

struct Window {
    Window(void);
    Window(const double _min_velocity_x, const double _max_velocity_x, const double _min_velocity_y, const double _max_velocity_y, const double _min_yawrate, const double _max_yawrate);
    double min_velocity_x;
    double max_velocity_x;
    double min_velocity_y;
    double max_velocity_y;
    double min_yawrate;
    double max_yawrate;
};

class DWAPlanner {
public:
    DWAPlanner(void);

    void initialize_node(ros::NodeHandle& n, ros::NodeHandle& ln);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
    void target_velocity_callback(const geometry_msgs::TwistConstPtr& msg);
    virtual Window calc_dynamic_window(const geometry_msgs::Twist& current_velocity);
    virtual float calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    virtual float calc_speed_cost(const std::vector<State>& traj, const float target_velocity);
    virtual float calc_obstacle_cost(const std::vector<State>& traj, const std::vector<std::vector<float>>&);
    virtual void motion(State& state, const double velocity, const double yawrate);
    std::vector<std::vector<float>> scan_to_obs(void);
    std::vector<std::vector<float>> raycast(void);
    void visualize_trajectories(const std::vector<std::vector<State>>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub);
    void visualize_trajectory(const std::vector<State>& trajectory, const double r, const double g, const double b, const ros::Publisher& pub);
    virtual std::vector<State> dwa_planning(Window dynamic_window, Eigen::Vector3d goal, std::vector<std::vector<float>> obs_list);
    void process(void);

protected:
    double HZ;
    std::string ROBOT_FRAME;
    double TARGET_VELOCITY;
    double MAX_VELOCITY_X;
    double MIN_VELOCITY_X;
    double MAX_YAWRATE;
    double MAX_ACCELERATION;
    double MAX_D_YAWRATE;
    double MAX_DIST;
    double VELOCITY_RESOLUTION;
    double YAWRATE_RESOLUTION;
    double ANGLE_RESOLUTION;
    double PREDICT_TIME;
    double TO_GOAL_COST_GAIN;
    double SPEED_COST_GAIN;
    double OBSTACLE_COST_GAIN;
    double DT;
    bool USE_SCAN_AS_INPUT;
    double GOAL_THRESHOLD;
    double TURN_DIRECTION_THRESHOLD;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Subscriber local_map_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber target_velocity_sub;
    tf::TransformListener global_to_robot_tf;
    tf::TransformListener lidar_tf;

    geometry_msgs::PoseStamped local_goal;
    sensor_msgs::LaserScan scan;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::Twist current_velocity;
    bool local_goal_subscribed;
    bool scan_updated;
    bool local_map_updated;
    bool odom_updated;
};
} // namespace dwa_planner

#endif //__DWA_PLANNER_H
