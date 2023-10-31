#ifndef __COSTMAP_H_
#define __COSTMAP_H_

#include <cstring>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>

namespace erl {

/**
 * @brief The CostMap2D class implements costmap a 2D Grid environment.
 * Note that lots of code are just for better costmap visualization in RViz.
 */
struct CostMap2D {
    struct Setting {
        // hector map,
        // lower and upper bounds of obstacle characterization, and unknown grid value
        int8_t kUnknown = -1;
        int8_t kObsLb = 80;
        int8_t kObsUb = 100;
        
        // ClearPath jackal robot
        double kCostRadius1 = 0.430 / 2.0; // inscribed radius
        double kCostRadius2 = 0.4;  // circumscribed inscribed radius/ inflation radius in classical binary planning setup
        double kCostGamma = 7.0;
        
        int8_t kCostUnknown = 3;
        int8_t kCostLethal = 19;
        int8_t kCostInscribed = 18;

        int8_t kCostCutoff = 6; // corresponding to 0.37 meter on cost curve

        int8_t kVisUnknown = -1;
        int8_t kVisLethal = 100;
        int8_t kVisInscribed = 90;
        int8_t kVisCutoff = -3;
        int8_t kVisZero = 110;
    };

    // containers
    std::shared_ptr<Setting> setting = nullptr;
    std::vector<uint8_t> bw_vec = {};
    std::vector<int8_t> planning_costmap_data = {};
    std::vector<int8_t> rviz_costmap_data = {};

    // constructor
    explicit CostMap2D(std::shared_ptr<Setting> in_setting): setting(std::move(in_setting)) {} 

    // Given ros occupancy grid, compute costmap using cv2
    void compute(const nav_msgs::OccupancyGrid::ConstPtr &gridmap_msg) {
        auto gm_data_1d_ptr = gridmap_msg->data.data();
        int map_width = gridmap_msg->info.width;
        int map_height = gridmap_msg->info.height;

        int n_elements = map_width * map_height;
        double map_resolution = gridmap_msg->info.resolution;

        // note that occupancy grid map is row-major, cv Mat is row-major, c++ is stl row-major
        // init 1d vector containers
        if (planning_costmap_data.empty()) {
            bw_vec.resize(n_elements);
            planning_costmap_data.resize(n_elements);
            rviz_costmap_data.resize(n_elements);
        }

        // put hector map to binary map, 0 value for obstacle, 255 for non obstacle
        for (int i = 0; i < n_elements; ++i) {
            if (gm_data_1d_ptr[i] >= setting->kObsLb && gm_data_1d_ptr[i] <= setting->kObsUb) {
                bw_vec[i] = 0;
            } else {
                bw_vec[i] = 255;
            }
        }

        // wrap 1d binary container in 2d cv Mat for further computation without copying the data
        cv::Mat bw_mat_8u(map_height, map_width, CV_8UC1, bw_vec.data());
        cv::Mat dist_mat_32f;
        // be cautious, cv::DIST_L2 return type is CV_32F 
        // https://docs.opencv.org/3.4/d7/d1b/group__imgproc__misc.html#ga8a0b7fdfcb7a13dde018988ba3a43042
        cv::distanceTransform(bw_mat_8u, dist_mat_32f, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        // assign costmap value by distance map, handle unknown type separately
        for (int i = 0; i < n_elements; ++i) {
            if ((gm_data_1d_ptr[i]) == setting->kUnknown) {
                planning_costmap_data[i] = setting->kCostUnknown;
                rviz_costmap_data[i] = setting->kVisUnknown;
            } else{
              assignCostmapValueByDistance(dist_mat_32f, map_resolution, i);
            } 
        } 
    }


    // for known type cell, we assign costmap cell value by distance to obstacle
    void assignCostmapValueByDistance(const cv::Mat &dist_mat, double map_res, int i){
      double dist_metric = dist_mat.at<float>(i) * map_res;

      // real obstacle
      if (dist_metric <= map_res) {
          planning_costmap_data[i] = setting->kCostLethal;
          rviz_costmap_data[i] = setting->kVisUnknown;
      } else if (dist_metric <= setting->kCostRadius1) {
          planning_costmap_data[i] = setting->kCostInscribed;
          rviz_costmap_data[i] = setting->kVisInscribed;
      } else {
          double cell_cost = 0.0;
          // special hack to achieve the shortest path with min clearance for Jackal 0.4 circumscribed radius
          if (setting->kCostGamma >= 15.0){
            cell_cost = 1.0 * setting->kCostInscribed - 1.0; 
            if (dist_metric > setting->kCostRadius2) cell_cost = 0.0; 
          }
          // get cell cost using distance filed and cost curve
          else{
            cell_cost = double(setting->kCostInscribed) * std::exp(-setting->kCostGamma * (dist_metric - setting->kCostRadius1));
          }

          planning_costmap_data[i] = int8_t(std::round(cell_cost));
          if (planning_costmap_data[i] == 0){
            rviz_costmap_data[i] = setting->kVisZero;
          // due to map resolution limitation, to show inflated planning boundary in [cutoff, cutoff+2] 
          }else if (planning_costmap_data[i] >= setting->kCostCutoff && planning_costmap_data[i] <= setting->kCostCutoff + 2){
            rviz_costmap_data[i] = setting->kVisCutoff;
          }else{
            // be cautious, the final result should less than 127, in our case, the max is kCostInscribed * 5 = 19 * 5 = 95
            rviz_costmap_data[i] = planning_costmap_data[i] * 5;
          }
      } // end for non-real obstacle
    }

}; // end of class
} // end of erl namespace

#endif
