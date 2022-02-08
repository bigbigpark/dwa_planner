#include "dwa_planner/dwa_planner_for_mecanum.h"

namespace dwa_planner {
DWAPlannerForMecanum::DWAPlannerForMecanum(void)
    : DWAPlanner()
{
    local_nh.param("HZ", HZ, { 20 });
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, { "base_link" });
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, { 0.8 });
    local_nh.param("MAX_VELOCITY_X", MAX_VELOCITY_X, { 1.0 });
    local_nh.param("MIN_VELOCITY_X", MIN_VELOCITY_X, { 0.0 });
    local_nh.param("MAX_VELOCITY_Y", MAX_VELOCITY_Y, { 1.0 });
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, { 1.0 });
    local_nh.param("MAX_DIST", MAX_DIST, { 10.0 });
    local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, { 0.1 });
    local_nh.param("PREDICT_TIME", PREDICT_TIME, { 3.0 });
    local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, { 1.0 });
    local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, { 1.0 });
    local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, { 1.0 });
    local_nh.param("USE_SCAN_AS_INPUT", USE_SCAN_AS_INPUT, { false });
    local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, { 0.3 });
    MIN_VELOCITY_Y = -MAX_VELOCITY_Y;
    DT = 1.0 / HZ;
    MAX_YAWRATE = 0.0;
    TURN_DIRECTION_THRESHOLD = 0.0;

    ROS_INFO_STREAM("HZ: " << HZ);
    ROS_INFO_STREAM("DT: " << DT);
    ROS_INFO_STREAM("ROBOT_FRAME: " << ROBOT_FRAME);
    ROS_INFO_STREAM("TARGET_VELOCITY: " << TARGET_VELOCITY);
    ROS_INFO_STREAM("MAX_VELOCITY_X: " << MAX_VELOCITY_X);
    ROS_INFO_STREAM("MIN_VELOCITY_X: " << MIN_VELOCITY_X);
    ROS_INFO_STREAM("MAX_VELOCITY_Y: " << MAX_VELOCITY_Y);
    ROS_INFO_STREAM("MIN_VELOCITY_Y: " << MIN_VELOCITY_Y);
    ROS_INFO_STREAM("MAX_ACCELERATION: " << MAX_ACCELERATION);
    ROS_INFO_STREAM("MAX_DIST: " << MAX_DIST);
    ROS_INFO_STREAM("VELOCITY_RESOLUTION: " << VELOCITY_RESOLUTION);
    ROS_INFO_STREAM("PREDICT_TIME: " << PREDICT_TIME);
    ROS_INFO_STREAM("TO_GOAL_COST_GAIN: " << TO_GOAL_COST_GAIN);
    ROS_INFO_STREAM("SPEED_COST_GAIN: " << SPEED_COST_GAIN);
    ROS_INFO_STREAM("OBSTACLE_COST_GAIN: " << OBSTACLE_COST_GAIN);
    ROS_INFO_STREAM("GOAL_THRESHOLD: " << GOAL_THRESHOLD);

    initialize_node(nh, local_nh);
}

Window DWAPlannerForMecanum::calc_dynamic_window(const geometry_msgs::Twist& current_velocity)
{
    Window window(MIN_VELOCITY_X, MAX_VELOCITY_X, MIN_VELOCITY_Y, MAX_VELOCITY_Y, 0.0, 0.0);
    window.min_velocity_x = std::max((current_velocity.linear.x - MAX_ACCELERATION * DT), MIN_VELOCITY_X);
    window.max_velocity_x = std::min((current_velocity.linear.x + MAX_ACCELERATION * DT), MAX_VELOCITY_X);
    window.min_velocity_y = std::max((current_velocity.linear.y - MAX_ACCELERATION * DT), MIN_VELOCITY_Y);
    window.max_velocity_y = std::min((current_velocity.linear.y + MAX_ACCELERATION * DT), MAX_VELOCITY_Y);
    return window;
}

float DWAPlannerForMecanum::calc_speed_cost(const std::vector<State>& traj, const float target_velocity)
{
    float cost_x = fabs(target_velocity - fabs(traj[traj.size() - 1].velocity_x));
    float cost_y = fabs(target_velocity - fabs(traj[traj.size() - 1].velocity_y));
    return std::max(cost_x, cost_y);
}

void DWAPlannerForMecanum::motion(State& state, const double velocity_x, const double velocity_y)
{
    state.x += velocity_x * DT;
    state.y += velocity_y * DT;
    state.velocity_x = velocity_x;
    state.velocity_y = velocity_y;
}

std::vector<State> DWAPlannerForMecanum::dwa_planning(
    Window dynamic_window,
    Eigen::Vector3d goal,
    std::vector<std::vector<float>> obs_list)
{
    float min_cost = 1e6;
    float min_obs_cost = min_cost;
    float min_goal_cost = min_cost;
    float min_speed_cost = min_cost;

    std::vector<std::vector<State>> trajectories;
    std::vector<State> best_traj;

    for (float vx = dynamic_window.min_velocity_x; vx <= dynamic_window.max_velocity_x; vx += VELOCITY_RESOLUTION) {
        for (float vy = dynamic_window.min_velocity_y; vy <= dynamic_window.max_velocity_y; vy += VELOCITY_RESOLUTION) {
            State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.linear.y, 0.0);
            std::vector<State> traj;
            for (float t = 0; t <= PREDICT_TIME; t += DT) {
                motion(state, vx, vy);
                traj.push_back(state);
            }
            trajectories.push_back(traj);

            float to_goal_cost = calc_to_goal_cost(traj, goal);
            float speed_cost = calc_speed_cost(traj, TARGET_VELOCITY);
            float obstacle_cost = calc_obstacle_cost(traj, obs_list);
            float final_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost + OBSTACLE_COST_GAIN * obstacle_cost;
            if (min_cost >= final_cost) {
                min_goal_cost = TO_GOAL_COST_GAIN * to_goal_cost;
                min_obs_cost = OBSTACLE_COST_GAIN * obstacle_cost;
                min_speed_cost = SPEED_COST_GAIN * speed_cost;
                min_cost = final_cost;
                best_traj = traj;
            }
        }
    }
    ROS_INFO_STREAM("Cost: " << min_cost);
    ROS_INFO_STREAM("- Goal cost: " << min_goal_cost);
    ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
    ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
    ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
    visualize_trajectories(trajectories, 0, 1, 0, 1000, candidate_trajectories_pub);
    if (min_cost == 1e6) {
        std::vector<State> traj;
        State state(0.0, 0.0, 0.0, current_velocity.linear.x, current_velocity.linear.y, 0.0);
        traj.push_back(state);
        best_traj = traj;
    }
    return best_traj;
}
} // namespace dwa_planner
