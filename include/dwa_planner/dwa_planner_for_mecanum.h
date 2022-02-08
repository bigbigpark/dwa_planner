#ifndef __DWA_PLANNER_FOR_MECANUM_H
#define __DWA_PLANNER_FOR_MECANUM_H

#include "dwa_planner/dwa_planner.h"

namespace dwa_planner {
class DWAPlannerForMecanum : public DWAPlanner {
public:
    DWAPlannerForMecanum(void);

    virtual Window calc_dynamic_window(const geometry_msgs::Twist& current_velocity) override;
    virtual float calc_speed_cost(const std::vector<State>& traj, const float target_velocity) override;
    virtual void motion(State& state, const double velocity_x, const double velocity_y) override;
    virtual std::vector<State> dwa_planning(Window dynamic_window, Eigen::Vector3d goal, std::vector<std::vector<float>> obs_list) override;

protected:
    double MAX_VELOCITY_Y;
    double MIN_VELOCITY_Y;
};
}

#endif // __DWA_PLANNER_FOR_MECANUM_H
