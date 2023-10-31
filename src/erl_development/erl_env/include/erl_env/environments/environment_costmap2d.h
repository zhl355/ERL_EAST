#ifndef __ENV_COSTMAP2D_H_
#define __ENV_COSTMAP2D_H_

#include <erl_env/environments/environment_metric.h>
#include <erl_env/systems/motion_primitive.h>
#include <erl_map/grid_map.h>
#include <erl_utilities/trajectories/piecewise_polynomial.h>

#include <memory>   // std::unique_ptr
#include <utility>  // std::pair
#include <vector>

#include <string> // std::to_string()
#include <iostream>

namespace erl {

/**
 * @brief The EnvironmentCostmap2D class implements the Environment specification for a 2D Costmap Grid environment.
 */
template <class U>
class EnvironmentCostmap2D : public EnvironmentMetric<std::vector<int>, U> {
 public:
  using EnvironmentMetric<std::vector<int>, U>::map;  // Bring Parent variables into the namespace.

  std::vector<MotionPrimitive<std::pair<double, double>, std::string>> mprim_;  // Motion Primitives
  /**
   * @brief Constructs the EnvironmentCostmap2D via the GridMap (2d map encode cost) provided. Use this constructor when there are
   * multiple environments to avoid storing multiple copies of the maps.
   * @param map The map of the environment.
   */
  EnvironmentCostmap2D(const std::shared_ptr<erl::GridMap<U>> &map) : EnvironmentMetric<std::vector<int>, U>(map), mprim_(8) {
    // Initialize 2-D Motion Primitives.
    initMprim();
  }

  /**
   * @brief Constructs the EnvironmentCostmap2D via regular GridMap. Internally they are converted to shared pointer.
   * @param map The map of the environment.
   */
  EnvironmentCostmap2D(const erl::GridMap<U> &map)
      : EnvironmentMetric<std::vector<int>, U>(std::make_shared<erl::GridMap<U>>(map)), mprim_(8) {
    initMprim();
  }

  /**
   * @brief Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  void getSuccessors(const std::vector<int> &curr, std::vector<std::vector<int>> &succ, std::vector<double> &succ_cost,
                     std::vector<int> &action_idx) const override {
    size_t num_traj = mprim_.size(); // for 2d planning always 8
    size_t len_traj = mprim_[0].xVecVec.size(); // for 2d planning always 1

    double min_x = map->min()[0];
    double res_x = map->res()[0];
    int sz_x = map->size()[0];
    double min_y = map->min()[1];
    double res_y = map->res()[1];
    int sz_y = map->size()[1];

    // Reserve space for successors
    succ.reserve(num_traj * len_traj); // # motion * # steps/motion, for 2d, 8 * 1
    succ_cost.reserve(num_traj * len_traj); // cost for each step, for 2d, reduce to 8 cost for each motion
    action_idx.reserve(num_traj * len_traj); // action for each step, for 2d, reduce to action for each motion

    for (unsigned tr = 0; tr < num_traj; ++tr)
      for (unsigned len = 0; len < len_traj; ++len) {
        // check for collisions along the microstates
        bool collided = false;
        unsigned int len_fine_sz = mprim_[tr].xVecVec[len].size();
        // micro-states between steps, for 2d, len_fine_sz = 1
        for (unsigned int len_fine = 0; len_fine < len_fine_sz; ++len_fine) {
          // x_val, y_val are just 2d cell indices in graph
          int x_val = erl::meters2cells(
              erl::cells2meters(curr[0], min_x, res_x) + mprim_[tr].xVecVec[len][len_fine].first, min_x, res_x);
          int y_val = erl::meters2cells(
              erl::cells2meters(curr[1], min_y, res_y) + mprim_[tr].xVecVec[len][len_fine].second, min_y, res_y);

          // Discard motion primitives that go outside the map
          if (x_val < 0 || x_val >= sz_x || y_val < 0 || y_val >= sz_y) {
            collided = true;
            break;
          }
          // Discard motion primitives that collide
          size_t lindix = map->subv2ind({x_val, y_val});

          // ASTAR_COSTMAP2D check go to cost to surrounding neighbor, map internal data here is 1d
          int8_t cell_cost_xy = map->map().at(lindix);

          // cell cost greater than planning_cutoff_cost will be considered as obstacle
          if (cell_cost_xy > m_planning_cutoff_cost) {
            collided = true;
            break;
          }
          // If End of trajectory, we arrive at the successor.
          if (len_fine == len_fine_sz - 1) {
            succ.push_back({x_val, y_val});
            // ASTAR_COSTMAP2D calculate cost using non-binary costmap (previous cell_cost_xy = 0)
            succ_cost.push_back(mprim_[tr].cVec[len] + (double)cell_cost_xy); 
            action_idx.push_back(tr * len_traj + len);  // action_id, for 2d, index reduces to (tr * 1 + 0)
          }
        }
        // No need to check the rest of the trajectory length if we collided already
        if (collided) break;
      }
  }

  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   */
  std::vector<std::vector<int>> forwardAction(const std::vector<int> &curr, int action_id) const override {
    // Output.
    std::vector<std::vector<int>> next_micro;
    size_t len_traj = mprim_[0].xVecVec.size();

    // convert action_id to specific control input
    unsigned int tr = action_id / len_traj;  // find which trajectory it is
    unsigned int len = action_id % len_traj;
    unsigned int len_fine_sz = mprim_[tr].xVecVec[len].size();

    for (unsigned int len_fine = 0; len_fine < len_fine_sz; ++len_fine) {
      int x_val = erl::meters2cells(
          erl::cells2meters(curr[0], map->min()[0], map->res()[0]) + mprim_[tr].xVecVec[len][len_fine].first,
          map->min()[0], map->res()[0]);
      int y_val = erl::meters2cells(
          erl::cells2meters(curr[1], map->min()[1], map->res()[1]) + mprim_[tr].xVecVec[len][len_fine].second,
          map->min()[1], map->res()[1]);
      next_micro.push_back({x_val, y_val});
    }
    return next_micro;
  }

  /**
   * @brief Computes the state representation of the metric state.
   * @param metric The metric state.
   * @return The state representation.
   */
  virtual std::vector<int> toState(const std::vector<double> &metric) const override {
    return map->meters2cells(metric);
  }

  /**
   * @brief Returns the metric representation of the state form.
   * @param cell The state.
   * @return The metric state.
   */
  virtual std::vector<double> toMetric(const std::vector<int> &cell) const override { return map->cells2meters(cell); }

  /**
   * @brief Generate PiecewisePolynomial trajectories from a 2D path.
   * @param path The path given.
   * @param tau The uniform time scaling parameter to constrain the velocity profile.
   * @param th_init The initial heading in radians. Set to zero if this is not needed.
   * @param th_end The final heading in radians. Set to zero if this is not needed.
   * @param vel_init The initial velocity. Default to 1 m/s.
   * @return The resulting PiecewisePolynomial trajectory.
   */
  trajectories::PiecewisePolynomial<double> toTrajectory(const std::vector<std::vector<double>> &path, double tau = 0.1,
                                                         double th_init = 0.0, double th_end = 0.0,
                                                         double vel_init = 1.0) const {
    // Compute the segment times
    std::vector<double> segment_times(path.size());
    segment_times[0] = 0.0;
    for (int k = 0; k < path.size() - 1; ++k) segment_times[k + 1] = segment_times[k] + tau;
    std::vector<Eigen::MatrixXd> knots(path.size(), Eigen::MatrixXd(2, 1));
    for (int i = 0; i < path.size(); i++) {
      Eigen::MatrixXd vec(2, 1);
      vec << path[i][0], path[i][1];
      knots[i] = (vec);
    }

    // Start and End Velocity. This may optionally be used to allow for terminal yaw constraints.
    Eigen::MatrixXd v_start(2, 1);
    v_start << vel_init * std::cos(th_init), vel_init * std::sin(th_init);
    Eigen::MatrixXd v_end(2, 1);
    v_end << vel_init * std::cos(th_end), vel_init * std::sin(th_end);

    return trajectories::PiecewisePolynomial<double>::Cubic(segment_times, knots, v_start, v_end);
  }

  /**
   * @brief Set threshold for obstacle lower bound for planning over costmap2d. 
   */
  void setPlanningValues(int8_t planning_cutoff_cost, int8_t planning_unknown_cost) {
    // ASTAR_COSTMAP2D
    // set planning obstacle lb, strictly greater than lb, will be considered obstacle in planner's view
    m_planning_cutoff_cost = planning_cutoff_cost;
    m_planning_unknown_cost = planning_unknown_cost;
    if (m_planning_unknown_cost > m_planning_cutoff_cost){
      throw std::runtime_error("m_planning_cutoff_cost must greater than m_planning_unknown_cost");
    }
  }


  /**
   * @brief Check whether datam (data metric) at (x, y in meter) is feasible 
   * according to costmap2d m_planning_cutoff_cost and unknown_cost.
   */
  bool isLocationFeasibleOnCostmap(const std::vector<double> &datam) {
    bool loc_feasible = false;
    int8_t loc_cost = getLocationValueOnCostmap(datam);
    if (loc_cost <= m_planning_cutoff_cost) 
      loc_feasible = true;
    return loc_feasible;
  }

    /**
   * @brief Get cost from costmap at location specified in datam (coordinates in meter).
   */
  int8_t getLocationValueOnCostmap(const std::vector<double> &datam) {
    // ASTAR_COSTMAP2D get cost value at location in datam (coordinates in meter)
    auto datac = map->meters2cells(datam); // get 2d cell coordinates
    auto costmap_idx = map->subv2ind(datac); // get 1d index in costmap
    int8_t cell_cost_xy = map->map().at(costmap_idx); // get 1d costmap value in stl container

    return cell_cost_xy;
  }

 private:

  // private member planning check obstacle lower bound ASTAR_COSTMAP2D
  int8_t m_planning_cutoff_cost = 11;
  int8_t m_planning_unknown_cost = 3; 

  /**
   * @brief Initialize Motion Primitives.
   */
  void initMprim() {
    // Setup 2-D Grid Motion Primitives
    double x_res = map->res()[0];
    double y_res = map->res()[1];
    mprim_[0].uVec.push_back("Forward");
    mprim_[0].xVecVec.push_back({std::make_pair(x_res, 0)});
    mprim_[1].uVec.push_back("Back");
    mprim_[1].xVecVec.push_back({std::make_pair(-x_res, 0)});
    mprim_[2].uVec.push_back("Right");
    mprim_[2].xVecVec.push_back({std::make_pair(0, y_res)});
    mprim_[3].uVec.push_back("Left");
    mprim_[3].xVecVec.push_back({std::make_pair(0, -y_res)});
    mprim_[4].uVec.push_back("Diag1");
    mprim_[4].xVecVec.push_back({std::make_pair(x_res, y_res)});
    mprim_[5].uVec.push_back("Diag2");
    mprim_[5].xVecVec.push_back({std::make_pair(-x_res, y_res)});
    mprim_[6].uVec.push_back("Diag3");
    mprim_[6].xVecVec.push_back({std::make_pair(x_res, -y_res)});
    mprim_[7].uVec.push_back("Diag4");
    mprim_[7].xVecVec.push_back({std::make_pair(-x_res, -y_res)});

    for (int k = 0; k < 8; ++k) {
      mprim_[k].tVec.push_back(1);
      mprim_[k].cVec.push_back(std::sqrt(mprim_[k].xVecVec[0][0].first * mprim_[k].xVecVec[0][0].first +
                                         mprim_[k].xVecVec[0][0].second * mprim_[k].xVecVec[0][0].second));
    }
  }
};

}  // namespace erl

#endif
