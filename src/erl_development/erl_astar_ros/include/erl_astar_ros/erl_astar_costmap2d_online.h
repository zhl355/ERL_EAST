#include <erl_astar/astar_nx.h>
#include <erl_conversions/erl_msg_utils.h>
#include <erl_msgs/GridMap.h>
#include <erl_msgs/PlanningStatus.h>
#include <erl_msgs/CustomPath.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/duration.h>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <erl_astar/environments/planning_costmap2d.h>


/**
 * @brief ErlAstarCostmap2dOnline The ErlAstarCostmap2dOnline class is responsible for reading in all configuration
 * parameters for 2d of A* planner that can be called, as well as managing necessary
 * subscribers and publishers to generate a 2d path with the ERL Astar package.
 */

class ErlAstarCostmap2dOnline {
public:
    /**
     * @brief Constructs the ErlAstarCostmap2dOnline.
     * @param nh The ROS node handle.
     */
    void init() {
        // Load parameters
        if (!readParameters()) {
            ROS_ERROR("ErlAstarCostmap2dOnline parameter loading failed!");
            ros::requestShutdown();
        } else {
            ROS_INFO("ErlAstarCostmap2dOnline parameter loading successfully");
        }

        // A* planner attach
        erl::ARAStar<std::vector<int>> astar_planner_costmap2d;
        this->AA = std::make_shared<erl::ARAStar<std::vector<int>>>(astar_planner_costmap2d);
    }

    explicit ErlAstarCostmap2dOnline(ros::NodeHandle &nh) : nh(nh) {
        init();
        // Setup Publishers and then Subscribers
        path_pub = nh.advertise<nav_msgs::Path>(path_topic, 1, true);
        if (publish_custom_path) {
            custom_path_pub = nh.advertise<erl_msgs::CustomPath>(custom_path_topic, 1, true);
        }
        planning_status_pub = nh.advertise<erl_msgs::PlanningStatus>(planning_status_topic, 1, true);
        rviz_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic, 1, &ErlAstarCostmap2dOnline::goalCallback, this);
        costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 2, &ErlAstarCostmap2dOnline::costmapCallback, this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, &ErlAstarCostmap2dOnline::odomCallback, this);

    }  // end of constructor

private:
    int8_t planning_status = 0;
    // ----------------------- ROS -------------------------------
    ros::NodeHandle nh;
    // subscribers
    ros::Subscriber odom_sub;       // Odometry Subscriber.
    ros::Subscriber costmap_sub;        // GridMap Subscriber from a mapper.
    ros::Subscriber rviz_goal_sub;  // Goal Subscriber

    std::string odom_topic = "/odom";
    std::string costmap_topic = "/costmap";
    std::string goal_topic = "/move_base_simple/goal";

    // configurable publish planning status
    bool publish_planning_status = true;

    // configurable publish custom path
    bool publish_custom_path = false;

    // ASTAR_COSTMAP2D
    int planning_cutoff_cost = 11;
    int planning_unknown_cost = 3; //

    // publishers
    ros::Publisher path_pub;  // Path publisher
    ros::Publisher planning_status_pub;  // Planning status publisher
    ros::Publisher custom_path_pub;  // custom path publisher

    std::string path_topic = "/path";
    std::string planning_status_topic = "planning_status";
    std::string custom_path_topic = "custom_path";

    std::string planning_frame = "map"; // planning frame computationally grid_msg frame is the best choice


    // status flags
    bool goal_received{false};
    bool map_received{false};

    // transform for planning in map frame, robot state from odometry in odom frame
    tf::TransformListener start_listener;
    tf::StampedTransform start_transform;

    tf::TransformListener goal_listener;
    tf::StampedTransform goal_transform;

    // ----------------------- A* planning --------------------------
    // grid map
    std::shared_ptr<erl::GridMap<int8_t>> grid_map;

    // A* core
    std::shared_ptr<erl::Planning2D<int8_t>> planning_env;
    std::shared_ptr<erl::ARAStar<std::vector<int>>> AA;

    // A* heuristic eps setup
    double astar_eps{1.0};

    // Astar Goal
    double goal_x{0.0};
    double goal_y{0.0};

    // Astar start
    double start_x{0.0};
    double start_y{0.0};

    // ----------------------- FUNCTIONS --------------------------

    /**
     * Update goal from rviz
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        goal_received = true;
        goal_x = msg->pose.position.x;
        goal_y = msg->pose.position.y;

        // transform goal in planning frame, assume 2d, frame orientation are the same
        if (msg->header.frame_id != planning_frame) {
            ROS_WARN("planning_frame [%s] is different from occupancy msg frame_id [%s]", planning_frame.c_str(), msg->header.frame_id.c_str());
            try {
                goal_listener.waitForTransform(planning_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
                goal_listener.lookupTransform(planning_frame, msg->header.frame_id, ros::Time(0), goal_transform);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }
            goal_x += goal_transform.getOrigin().x();
            goal_y += goal_transform.getOrigin().y();
        }
        callAStar(start_x, start_y);
    }

    /**
     * Call A* after the robot pose is updated.
     */
    void callAStar(double start_pos_x, double start_pos_y) {
        if (!goal_received) return;
        // Update the goal cell as it might have been changed due to the expanding map
        this->planning_env->goal_coord = this->planning_env->map->meters2cells({goal_x, goal_y});

        // Call A* to update path
        auto start_cell = planning_env->toState({start_pos_x, start_pos_y});  // Convert metric to cell state.

        if (planning_frame != "map") {
            ROS_INFO("planning in [%s] frame", planning_frame.c_str());
        }

        auto goal_cell = this->planning_env->toState({goal_x, goal_y});
        ROS_INFO("Begin Astar\nStart 2D: %6.2f %6.2f\nGoal  2D: %6.2f %6.2f", start_pos_x, start_pos_y, goal_x, goal_y);
        ROS_INFO("\nStart 2D cell: %5d %5d\nGoal  2D cell: %5d %5d", start_cell[0], start_cell[1], goal_cell[0], goal_cell[1]);

        // check if start and goal is feasible
        planning_status = 0;
        bool start_feasible = this->planning_env->isLocationFeasibleOnCostmap({start_pos_x, start_pos_y});

        if (!start_feasible) {
            auto start_cost = this->planning_env->getLocationValueOnCostmap({start_pos_x, start_pos_y});
            ROS_WARN_THROTTLE(1.0, "\nstart (%6.2f, %6.2f) on costmap value = %d > %d\n", start_pos_x, start_pos_y, start_cost, planning_cutoff_cost);
        } else {
            planning_status += 4; // init feasible
        }

        bool goal_feasible = this->planning_env->isLocationFeasibleOnCostmap({goal_x, goal_y});
        if (!goal_feasible) {
            auto goal_cost = this->planning_env->getLocationValueOnCostmap({goal_x, goal_y});
            ROS_WARN_THROTTLE(1.0, "\ngoal (%6.2f, %6.2f) on costmap value = %d > %d\n", goal_x, goal_y, goal_cost, planning_cutoff_cost);
        } else {
            planning_status += 2;
        }

        auto output = AA->Astar(start_cell, *planning_env, astar_eps);
        ROS_INFO("Finished Astar: Path 2D Length (in cell): %ld\t\t Path Cost: %.2f\n", output.action_idx.size(), output.pcost);

        std::vector<std::vector<double>> metric_path;
        for (const auto &state: output.path) {
            // metric_path contains 2d location in planning frame (map frame)
            metric_path.push_back(planning_env->toMetric(state));
        }
        if (!metric_path.empty()) {
            // Publish Path Message
            ROS_DEBUG_THROTTLE(1.0, "A* has a valid path");
            planning_status += 1;
        } else {

            ROS_WARN_THROTTLE(1.0, "A* on costmap has problem cost = %.2f", output.pcost);
            ROS_WARN_THROTTLE(1.0, "Begin Astar\nStart 2D: %6.2f %6.2f\nGoal  2D: %6.2f %6.2f", start_pos_x, start_pos_y, goal_x, goal_y);
            ROS_WARN_THROTTLE(1.0, "\nStart 2D cell: %5d %5d\nGoal  2D cell: %5d %5d", start_cell[0], start_cell[1], goal_cell[0], goal_cell[1]);

            auto linear_index = this->planning_env->map->subv2ind(start_cell);
            int start_cost = int(this->planning_env->map->map().at(linear_index));
            ROS_WARN_THROTTLE(1.0, "\nstart costmap val = %d", start_cost);

            linear_index = this->planning_env->map->subv2ind(goal_cell);
            int goal_cost = int(this->planning_env->map->map().at(linear_index));
            ROS_WARN_THROTTLE(1.0, "\ngoal costmap val = %d , %d (planning_cutoff_cost)", goal_cost, planning_cutoff_cost);

            // trick to tell downstream this path is invalid
            metric_path.push_back({start_pos_x, start_pos_y});
            metric_path.push_back({start_pos_x, start_pos_y});
        }

        publishPathMessage(metric_path);
    }

    /**
     * Callback for Odometry messages. Writes the robot current state to the class members.
     * @param msg The incoming Odometry message.
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        // Pose below is in Odom frame
        auto pose_odom = msg->pose.pose;

        if (!map_received || !goal_received) {
            return;
        }

        // start_x, start_y in msg transform it to planning_frame
        start_x = pose_odom.position.x;
        start_y = pose_odom.position.y;

        // transform goal in planning frame, assume 2d, frame orientation are the same
        if (msg->header.frame_id != planning_frame) {
            try {
                start_listener.waitForTransform(planning_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
                start_listener.lookupTransform(planning_frame, msg->header.frame_id, ros::Time(0), start_transform);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }
            start_x += start_transform.getOrigin().x();
            start_y += start_transform.getOrigin().y();
        }

        callAStar(start_x, start_y);
    }

    /**
     * Callback for Odometry messages. Writes the robot current state to the class members.
     * @param msg The incoming Odometry message.
     * Convert ROS Occupancy Grid Map (costmap msg) to erl::GridMap for planning
     * ASTAR_COSTMAP2D do not use inflation here, use cost to determine lethal obstacle
     */
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg) {

        auto gm = erl::fromROSCostmap(*costmap_msg);
        grid_map = std::make_shared<erl::GridMap<int8_t>>(gm);

        // check on planning_frame consistency,
        auto msg_frame_id = costmap_msg->header.frame_id;
        if (planning_frame != msg_frame_id) {
            ROS_ERROR("planning_frame [%s] is different from occupancy msg frame_id [%s]", planning_frame.c_str(), msg_frame_id.c_str());
        }

        // Generate new map parameter the first time, only update map contents next update.
        if (!map_received) {
            erl::EnvironmentCostmap2D<int8_t> curr_env(*grid_map);
            // pass config params to env
            // curr_env.setPlanningValues(planning_cutoff_cost, costmap_msg_unknown_cost, planning_unknown_cost);
            curr_env.setPlanningValues(planning_cutoff_cost, planning_unknown_cost);
            erl::Planning2D<int8_t> planning_env_costmap2d(curr_env, {goal_x, goal_y});
            this->planning_env = std::make_shared<erl::Planning2D<int8_t>>(planning_env_costmap2d);
            map_received = true;
        }
        // Update map
        this->planning_env->map = grid_map;
    }

    /**
     * Utils function. transform 2d location to 3d pose ros msg
     * @param x robot location coord x in map frame
     * @param y robot location coord y in map frame
     */
    inline geometry_msgs::PoseStamped getRos3DPoseFromAstar2DLocation(double x, double y) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        return pose;
    }

    /**
     * Publish output of A as ros path msg.
     * @param metric_path A* output, path 2d
     */
    void publishPathMessage(const std::vector<std::vector<double>> &metric_path) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = planning_frame;
        path_msg.header.stamp = ros::Time::now();
        for (const auto &loc2d: metric_path) {
            path_msg.poses.push_back(getRos3DPoseFromAstar2DLocation(loc2d[0], loc2d[1]));
        }
        path_pub.publish(path_msg);

        // publish optional planning status
        if (publish_planning_status) {
            erl_msgs::PlanningStatus planning_status_msg;
            planning_status_msg.header = path_msg.header;
            planning_status_msg.planning_status = planning_status;
            planning_status_pub.publish(planning_status_msg);
        }

        // publish optional custom path
        if (publish_planning_status && publish_custom_path) {
            erl_msgs::CustomPath custom_path_msg;
            custom_path_msg.header = path_msg.header;
            custom_path_msg.planning_status = planning_status;
            custom_path_msg.poses = path_msg.poses;
            custom_path_pub.publish(custom_path_msg);
        }

    }

    /**
     * @brief Read parameters from ROS parameter server
     */

    bool readParameters() {
        // for odom topic
        ROS_WARN("ErlAstarCostmap2d Reading Parameter...");
        if (!nh.getParam("odom_topic", odom_topic)) {
            ROS_ERROR("Could not find [odom_topic] parameter");
            return false;
        } else {
            ROS_INFO_STREAM("Read odom_topic:" << odom_topic);
        }
        // for map topic
        if (!nh.getParam("costmap_topic", costmap_topic)) {
            ROS_ERROR("Could not find [costmap_topic] parameter");
            return false;
        } else {
            ROS_INFO_STREAM("Read costmap_topic:" << costmap_topic);
        }
        // make sure planning frame and map frame are the same
        if (!nh.getParam("planning_frame", planning_frame)) {
            ROS_ERROR("Planning_frame is not specified ! %s", planning_frame.c_str());
            return false;
        } else {
            ROS_INFO_STREAM("Read planning_frame:" << planning_frame);
        }

        // for goal topic
        if (!nh.getParam("goal_topic", goal_topic)) {
            ROS_ERROR("Could not find [goal_topic] parameter");
            return false;
        } else {
            ROS_INFO_STREAM("Read goal_topic:" << goal_topic);
        }

        // for A* path2d publish
        if (!nh.getParam("path_topic", path_topic)) {
            ROS_ERROR("Could not find [path_topic] parameter");
            return false;
        } else {
            ROS_INFO_STREAM("Read path_topic:" << path_topic);
        }

        // for A* eps multiplier for heuristic
        if (!nh.getParam("astar_eps", astar_eps)) {
            ROS_WARN("Could not find [astar_eps] parameter use default value 1.0");
        } else {
            ROS_WARN_STREAM("Read astar_eps:" << astar_eps);
        }

        // for A* planning over costmap 2d ASTAR_COSTMAP2D
        if (!nh.getParam("planning_cutoff_cost", planning_cutoff_cost)) {
            ROS_ERROR("Could not find [planning_cutoff_cost] parameter");
            return false;
        } else {
            ROS_WARN_STREAM("Read planning_cutoff_cost:" << planning_cutoff_cost);
        }

        // for A* planning over costmap 2d ASTAR_COSTMAP2D unknown cost used for planning
        if (!nh.getParam("planning_unknown_cost", planning_unknown_cost)) {
            ROS_ERROR("Could not find [planning_unknown_cost] parameter");
            return false;
        } else {
            ROS_WARN_STREAM("Read planning_unknown_cost:" << planning_unknown_cost);
        }

        // for A* planning status publish
        if (!nh.getParam("publish_planning_status", publish_planning_status)) {
            ROS_WARN("Could not find [publish_planning_status], default true");
        } else {
            ROS_INFO_STREAM("Set publish_planning_status:" << publish_planning_status);
        }

        // // for A* custom_path publish
        if (!nh.getParam("publish_custom_path", publish_custom_path)) {
            ROS_WARN("Could not find [publish_custom_path], default is false");
        } else {
            ROS_INFO_STREAM("Set publish_custom_path:" << publish_custom_path);
        }

        // final reading_parameter status return
        return true;
    }

    /**
     * @brief Init all parameters. 1) from parameter server 2) attach A* planner
     */
};
