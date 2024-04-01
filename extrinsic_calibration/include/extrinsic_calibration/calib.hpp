/**
 * @file calib.hpp
 * @author Giovanni Rodrigues Dias (gigio18)
 * @brief Targetless extrinsic calibration package
 * 
 * Description: this package is used for find the rotation matrix and the translation vector between two sensors 
 * coordinates systems, using only the common region of sensors overlapping field of view. So we can define the 
 * rigid relationship with a unify coordinate system for multiple sensors.
 * 
 * @version 2.0
 * @date 2024-03-28
 */

#ifndef EXTRINSIC_CALIBRATION__CALIB_HPP_
#define EXTRINSIC_CALIBRATION__CALIB_HPP_

// ROS2 C++ Standard APIs
#include "rclcpp/rclcpp.hpp"

// Message filters for topic synchronization
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// PCL (Point Cloud Library) ROS interface stack
#include "pcl_conversions/pcl_conversions.h"   

// Message headers
#include "sensor_msgs/msg/point_cloud2.hpp"

// Linear Algebra library
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL Core
#include <pcl/point_cloud.h>                                 
#include <pcl/point_types.h>
#include <pcl/common/distances.h>

// PCL Iterative Closest Point (ICP) registration method
#include <pcl/registration/icp.h>            

// PCL Filters              
#include <pcl/filters/crop_box.h>                           
#include <pcl/filters/voxel_grid.h>                         
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h> 

// PCL Segmentation method
#include <pcl/segmentation/sac_segmentation.h> 

// C++ system headers
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace extrinsic
{
    /**
     * @class CalibNode 
     * @brief Targetless extrinsic calibration using interative closest point (icp) as main registration method
     */
    class CalibNode : public rclcpp::Node
    {
        public:
            // Aliases for specify namespaces
            using PointCloud2 = sensor_msgs::msg::PointCloud2;
            using sensor_policy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
            using sensor_sync = message_filters::Synchronizer<sensor_policy>;
            using pointT = pcl::PointXYZI;

            // Publishers topics (only for visualization)
            rclcpp::Publisher<PointCloud2>::SharedPtr rviz_cloud1_pub_;    // Sensor1 cloud 
            rclcpp::Publisher<PointCloud2>::SharedPtr rviz_cloud2_pub_;    // Sensor2 transformed cloud
            rclcpp::Publisher<PointCloud2>::SharedPtr rviz_combined_pub_;  // Combined cloud  

            /**
             * @brief Constructor
             */
            CalibNode();

            /**
             * @brief Destructor
             */
            ~CalibNode();
        private:
            // Enable visualization topics 
            bool vis_status_; 
            // Input sensors topics
            std::string sensor1_topic_; 
            std::string sensor2_topic_;
            // Filepath to best matrix
            std::string filepath_;
            
            // 2D ROI
            std::vector<double> min_roi_;
            std::vector<double> max_roi_;

            // Filters parameters
            int num_of_neighbors_;
            double std_deviation_;

            // Segmentation parameters
            double dist_threshold_;
            int sac_max_iter_;

            // ICP parameters
            int icp_max_iter_;
            double icp_tf_eps_;
            double icp_euclidean_fit_eps_;

            // Initial matrix guess
            double ini_x_;
	        double ini_y_; 
	        double ini_z_; 
	        double roll_; 
	        double pitch_; 
	        double yaw_;

            // Extrinsic matrix declaration
            Eigen::Matrix4f extrinsic_matrix_;

            // ICP best fitness score
            double fitness_score_;

            // Sensors topics synchronizers subscribers
            std::shared_ptr<message_filters::Subscriber<PointCloud2>> sensor1_sub_;
            std::shared_ptr<message_filters::Subscriber<PointCloud2>> sensor2_sub_;
            std::shared_ptr<sensor_sync> sensor_sync_;

            // Methods
            /**
             * @brief Assign the initial guess values for extrinsic matrix
             */
            void initializeMatrix();

            /**
             * @brief Preprocessing step the sensors point cloud data
             * 
             * @param sensor_msg : ROS2 PointCloud2 data
             * @param out_cloud_ptr : Pointer to pcl PointCloud data
             */
            void preprocessingCloud(
                const PointCloud2::ConstSharedPtr& sensor_msg, pcl::PointCloud<pointT>::Ptr out_cloud_ptr);

            /**
             * @brief SAC-RANSAC segmentation for detection and removal ground plane
             * 
             * @param cloud_ptr : Pointer to pcl PointCloud data
             * @param out_cloud_ptr : Pointer to pcl PointCloud data
             */
            void groundSegmentation(
                pcl::PointCloud<pointT>::Ptr cloud_ptr, pcl::PointCloud<pointT>::Ptr out_cloud_ptr);

            /**
             * @brief Write the best extrinsic matrix, based on the fitness score, into the yaml configuration file
             */
            void writeConfig();

            /**
             * @brief Callback function to handle with  synchronize data from multiple sensors
             * 
             * @param sensor_msg1 : ROS2 PointCloud2 data
             * @param sensor_msg2 : ROS2 PointCloud2 data
             */
            void sensorsCallback(
                const PointCloud2::ConstSharedPtr& sensor_msg1, const PointCloud2::ConstSharedPtr& sensor_msg2);
    };

} //namespace extrinsic

#endif //EXTRINSIC_CALIBRATION__CALIB_HPP_