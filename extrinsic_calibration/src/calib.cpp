#include "extrinsic_calibration/calib.hpp"

namespace extrinsic
{
    CalibNode::CalibNode() : Node("extrinsic_calib_node")
    {
        // Declare the node parameters
        this->declare_parameter("visualization", false);
        this->declare_parameter("topic1", "tf_left/velodyne_points");
        this->declare_parameter("topic2", "tf_right/velodyne_points");
        this->declare_parameter("filepath", "src/extrinsic_calibration/config/matrix.yaml");
        this->declare_parameter("min_roi", std::vector<double>{-100, -100});
        this->declare_parameter("max_roi", std::vector<double>{100, 100});
        this->declare_parameter("num_of_neighbors", 100);
        this->declare_parameter("std_deviation", 0.5);
        this->declare_parameter("dist_threshold", 0.1);
        this->declare_parameter("sac_max_iter", 500);
        this->declare_parameter("icp_max_iter", 100);
        this->declare_parameter("icp_tf_eps", 1.0);
        this->declare_parameter("icp_euclidean_fit_eps", 1.0);
        
        // Get the parameters
        vis_status_ = this->get_parameter("visualization").as_bool();
        sensor1_topic_ = this->get_parameter("topic1").as_string();
        sensor2_topic_ = this->get_parameter("topic2").as_string();
        filepath_ = this->get_parameter("filepath").as_string();
        min_roi_ = this->get_parameter("min_roi").as_double_array();
        max_roi_ = this->get_parameter("max_roi").as_double_array();
        num_of_neighbors_ = this->get_parameter("num_of_neighbors").as_int();
        std_deviation_ = this->get_parameter("std_deviation").as_double();
        dist_threshold_ = this->get_parameter("dist_threshold").as_double();
        sac_max_iter_ = this->get_parameter("sac_max_iter").as_int();
        icp_max_iter_ = this->get_parameter("icp_max_iter").as_int();
        icp_tf_eps_ = this->get_parameter("icp_tf_eps").as_double();
        icp_euclidean_fit_eps_ = this->get_parameter("icp_euclidean_fit_eps").as_double();

        // Declare initial matrix values
        ini_x_ = -0.115;
	    ini_y_ = -0.125; 
	    ini_z_ = 0.0; 
	    roll_ = -0.15; 
	    pitch_ = 0.0; 
	    yaw_ = 0.0;
            
        // Associate the subscriptions to the topics
        sensor1_sub_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, sensor1_topic_);
        sensor2_sub_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, sensor2_topic_);
        sensor_sync_ = std::make_shared<sensor_sync>(sensor_policy(10), *sensor1_sub_, *sensor2_sub_);
        sensor_sync_->registerCallback(
            std::bind(&CalibNode::sensorsCallback, this, std::placeholders::_1, std::placeholders::_2));

        rviz_cloud1_pub_ = create_publisher<PointCloud2>("left_cloud", 10);
        rviz_cloud2_pub_ = create_publisher<PointCloud2>("right_cloud", 10);
        rviz_combined_pub_ = create_publisher<PointCloud2>("combined_cloud", 10);
    }
            
    CalibNode::~CalibNode()
    {
        RCLCPP_INFO(get_logger(), "Finish the extrinsic calibration process!");
    }

    /**
     * @brief Assign the initial guess values for extrinsic matrix
     */
    void CalibNode::initializeMatrix()
    {
        if (std::ifstream(filepath_).good())
        {
            YAML::Node config = YAML::LoadFile(filepath_);
                    
            fitness_score_ = config["fitness_score"].as<double>();

            std::vector<double> matrix = config["extrinsic_matrix"].as<std::vector<double>>();
            
            if (static_cast<Eigen::Index>(matrix.size()) == extrinsic_matrix_.size())
            {
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        extrinsic_matrix_(i, j) = static_cast<double>(matrix[i * 4 + j]);
                    }
                }
            }
            else
            {
                extrinsic_matrix_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
            }
        }
        else 
        {
            Eigen::Vector3f translation_matrix(ini_x_, ini_y_, ini_z_);
            
            // Converter Euler angles to Quaternion
            Eigen::Quaternionf quaternion = Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX()) * 
                Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY()) * 
                Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ());
            Eigen::Matrix3f rotation_matrix = quaternion.matrix();

            // Extrinsic Matrix
            extrinsic_matrix_ = Eigen::Matrix4f::Identity();
            extrinsic_matrix_.block<3, 3>(0, 0) = rotation_matrix;
            extrinsic_matrix_.block<3, 1>(0, 3) = translation_matrix;
        }
    }

    /**
     * @brief Preprocessing step the sensors point cloud data
     * 
     * @param sensor_msg : ROS2 PointCloud2 data
     * @param out_cloud_ptr : Pointer to pcl PointCloud data
     */
    void CalibNode::preprocessingCloud(
        const PointCloud2::ConstSharedPtr& sensor_msg, pcl::PointCloud<pointT>::Ptr out_cloud_ptr)
    {
        // Convert ROS message to PCL PointCloud format using PointXYZI representation
        pcl::PCLPointCloud2 pcl_cloud;
        pcl_conversions::toPCL(*sensor_msg, pcl_cloud);
        pcl::fromPCLPointCloud2(pcl_cloud, *out_cloud_ptr);
        
        // Remove NaN and Inf values
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*out_cloud_ptr, *out_cloud_ptr, indices);

        //Remove outliers from pcl_pointcloud
        pcl::StatisticalOutlierRemoval<pointT> sor;
        sor.setMeanK(num_of_neighbors_);
        sor.setStddevMulThresh(std_deviation_);

        sor.setInputCloud(out_cloud_ptr);
        sor.filter(*out_cloud_ptr);

        // Define the region of interest
        pcl::CropBox<pcl::PointXYZI> crop_filter;
        crop_filter.setInputCloud(out_cloud_ptr); 
        crop_filter.setMin(Eigen::Vector4f(
            min_roi_[0], min_roi_[1], -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()));
        crop_filter.setMax(Eigen::Vector4f(
            max_roi_[0], max_roi_[1], std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()));
        crop_filter.filter(*out_cloud_ptr);
    }

    /**
     * @brief SAC-RANSAC segmentation for detection and removal ground plane
     * 
     * @param cloud_ptr : Pointer to pcl PointCloud data
     * @param out_cloud_ptr : Pointer to pcl PointCloud data
     */
    void CalibNode::groundSegmentation(
        pcl::PointCloud<pointT>::Ptr cloud_ptr, pcl::PointCloud<pointT>::Ptr non_plane_ptr)
    {   
        // Detect and filter ground planes using RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::PointIndices::Ptr indices_for_segmentation(new pcl::PointIndices);

        for (std::size_t i = 0; i < cloud_ptr->size(); ++i)
        {
            if (cloud_ptr->points[i].z < 0.5)
            {
                indices_for_segmentation->indices.push_back(static_cast<int>(i));
            }
        }

        pcl::SACSegmentation<pointT> sac;
        sac.setInputCloud(cloud_ptr);
        sac.setModelType(pcl::SACMODEL_PLANE);
        sac.setIndices(indices_for_segmentation);
        sac.setMethodType(pcl::SAC_RANSAC);
        sac.setDistanceThreshold(dist_threshold_);
        sac.setMaxIterations(sac_max_iter_);
        sac.setOptimizeCoefficients(true);
        sac.segment(*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model!");
        }
        else
        {
            // Extract the inliers (plane points)
            pcl::ExtractIndices<pointT> extract;
            extract.setInputCloud(cloud_ptr);
            extract.setIndices(inliers);
            extract.setNegative(true);

            // Get the plane points
            extract.filter(*non_plane_ptr);
        }
    }

    /**
     * @brief Write the best extrinsic matrix, based on the fitness score, into the yaml configuration file
     */
    void CalibNode::writeConfig()
    {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;

        emitter << YAML::Key << "fitness_score" << YAML::Value << fitness_score_;

        emitter << YAML::Key << "extrinsic_matrix" << YAML::Value << YAML::Flow << YAML::BeginSeq;

        for (int i = 0; i<4; ++i)
        {
            for (int j = 0; j<4; ++j)
            {
                emitter << extrinsic_matrix_(i, j);
            }
        }
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;

        std::ofstream fout(filepath_);
        fout << emitter.c_str();
    }

    /**
     * @brief Callback function to handle with  synchronize data from multiple sensors
     * 
     * @param sensor1_msg : ROS2 PointCloud2 data
     * @param sensor2_msg : ROS2 PointCloud2 data
     */
    void CalibNode::sensorsCallback(
        const PointCloud2::ConstSharedPtr& sensor1_msg, const PointCloud2::ConstSharedPtr& sensor2_msg)
    {
        pcl::PointCloud<pointT>::Ptr pcl_pointcloud1(new pcl::PointCloud<pointT>);   // Parent Cloud
        pcl::PointCloud<pointT>::Ptr pcl_pointcloud2(new pcl::PointCloud<pointT>);   // Child Cloud

        preprocessingCloud(sensor1_msg, pcl_pointcloud1);
        preprocessingCloud(sensor2_msg, pcl_pointcloud2);

        // Initial guess for extrinsic matrix
        initializeMatrix();

        // Get the non plane points
        pcl::PointCloud<pointT>::Ptr non_plane1(new pcl::PointCloud<pointT>);
        pcl::PointCloud<pointT>::Ptr non_plane2(new pcl::PointCloud<pointT>);
        
        groundSegmentation(pcl_pointcloud1, non_plane1);
        groundSegmentation(pcl_pointcloud2, non_plane2);

        // Registration algorithm
        pcl::PointCloud<pointT>::Ptr output_cloud(new pcl::PointCloud<pointT>);

        pcl::IterativeClosestPoint<pointT, pointT> icp;

        icp.setMaximumIterations(icp_max_iter_);
        icp.setTransformationEpsilon(icp_tf_eps_);
        icp.setEuclideanFitnessEpsilon(icp_euclidean_fit_eps_);

        icp.setInputSource(non_plane2);
        icp.setInputTarget(non_plane1);

        icp.align(*output_cloud, extrinsic_matrix_);

        if (icp.hasConverged())
        {
            double actual_fitness_score = icp.getFitnessScore();
            RCLCPP_INFO(get_logger(), "'%f'", actual_fitness_score);
            try 
            {
                if (actual_fitness_score < fitness_score_)
                {
                    fitness_score_ = actual_fitness_score;
                    extrinsic_matrix_ = icp.getFinalTransformation();
                    writeConfig();
                }
            }
            catch (const std::exception& error)
            {
                std::cerr << "Exception caught: " << error.what() << std::endl;
            }
        }
        else
        {
            // Registration failed to converge
            RCLCPP_WARN(get_logger(), "ICP registration failed to converge.");
        }
 
        if (vis_status_)
        {
            PointCloud2 cloud1_msg;
            pcl::toROSMsg(*non_plane1, cloud1_msg);
            cloud1_msg.header.frame_id = pcl_pointcloud1->header.frame_id;

            PointCloud2 cloud2_msg;
            pcl::toROSMsg(*output_cloud, cloud2_msg);
            cloud2_msg.header.frame_id = pcl_pointcloud1->header.frame_id;

            PointCloud2 cloud3_msg;
            pcl::PointCloud<pointT>::Ptr transformedCloud(new pcl::PointCloud<pointT>());
            pcl::PointCloud<pointT>::Ptr Cloud(new pcl::PointCloud<pointT>());
            pcl::transformPointCloud(*pcl_pointcloud2, *transformedCloud, extrinsic_matrix_);
            *Cloud = *pcl_pointcloud1 + *transformedCloud;
            pcl::toROSMsg(*Cloud, cloud3_msg);
            cloud3_msg.header.frame_id = pcl_pointcloud1->header.frame_id;

            rviz_cloud1_pub_->publish(cloud1_msg);
            rviz_cloud2_pub_->publish(cloud2_msg);
            rviz_combined_pub_->publish(cloud3_msg);       
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<extrinsic::CalibNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Node initialized successfully!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}