#include <chrono>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>

#include <pcl/point_cloud.h>
using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class VoxelGrid_filter : public rclcpp::Node
{
  public:
    VoxelGrid_filter()
    : Node("point_cloud_clustering")
    {
       marker_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
      subscription_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/c16/point_cloud", 10, std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

      publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cloud", 10);


    }

  private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
      {
        pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>) ;
        pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>) ;

        pcl::fromROSMsg(*input_cloud, *pcl_cloud);
  //==================================== Pre Processing Data ====================================
        pcl::PassThrough<PointT> passing_x;
        pcl::PassThrough<PointT> passing_y;
        pcl::PassThrough<PointT> passing_z;
        int radius = 10;
        int min_z = -1;
        int max_z = 1;
        passing_x.setInputCloud(pcl_cloud);
        passing_x.setFilterFieldName("x");
        passing_x.setFilterLimits(-radius,radius);
        passing_x.filter(*cropped_cloud);

        // Along Y Axis

        passing_y.setInputCloud(cropped_cloud);
        passing_y.setFilterFieldName("y");
        passing_y.setFilterLimits(-radius,radius);
        passing_y.filter(*cropped_cloud);

        // // Along Z Axis
        // passing_z.setInputCloud(cropped_cloud);
        // passing_z.setFilterFieldName("z");
        // passing_z.setFilterLimits(min_z,max_z);
        // passing_z.filter(*cropped_cloud);


        // Voxel Filter
        pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>) ;
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cropped_cloud);
        voxel_filter.setLeafSize(0.2 , 0.2, 0.2);
        voxel_filter.filter(*voxel_cloud);
  //==================================== Plane Segmentation  ====================================
        pcl::NormalEstimation<PointT, pcl::Normal> normal_extractor;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        pcl::PointCloud<pcl::Normal>::Ptr planexy_normals(new pcl::PointCloud<pcl::Normal>);

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> planexy_seg_frm_normals;
        pcl::PointIndices::Ptr planexy_inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr planexy_coefficients(new pcl::ModelCoefficients);
        pcl::ExtractIndices<PointT> planexy_extract_indices;
        pcl::PointCloud<PointT>::Ptr planexy_cloud(new pcl::PointCloud<PointT>);


        // Normals Extractions
        normal_extractor.setSearchMethod(tree);
        normal_extractor.setInputCloud(voxel_cloud);
        normal_extractor.setKSearch(30);  // Set the number of nearest neighbors to use for normal estimation. 
                                 // Increasing this value can make the normals smoother but less sensitive to small features.
                                 // Decreasing this value can make the normals more sensitive to small features but noisier.
        normal_extractor.compute(*planexy_normals);

        // Parameters for Planar Segmentation
        planexy_seg_frm_normals.setOptimizeCoefficients(true);
        planexy_seg_frm_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        planexy_seg_frm_normals.setMethodType(pcl::SAC_RANSAC);
        planexy_seg_frm_normals.setNormalDistanceWeight(0.5);  // Set the weight of the normal distance (how much the normal direction influences the fitting).
                                                   // Increasing this value gives more importance to the normal direction in the fitting process.
                                                   // Decreasing this value gives less importance to the normal direction.
        planexy_seg_frm_normals.setMaxIterations(100); // Set the maximum number of iterations for the RANSAC algorithm.
                                            // Increasing this value can improve the accuracy of the model fitting but will take more time.
                                            // Decreasing this value can speed up the process but may reduce accuracy.
        planexy_seg_frm_normals.setDistanceThreshold(0.5); // Set the distance threshold for a point to be considered fitting the model.
                                                // Increasing this value allows more points to be considered as inliers, which can be useful for noisy data.
                                                // Decreasing this value makes the model more strict, considering fewer points as inliers.
        planexy_seg_frm_normals.setInputCloud(voxel_cloud);
        planexy_seg_frm_normals.setInputNormals(planexy_normals); // Set to true to extract points that are not inliers (i.e., the planexy surface).
                                        // Setting this to false would extract the inliers instead.
        planexy_seg_frm_normals.segment(*planexy_inliers,*planexy_coefficients);

        //Extracting Cloud based on Inliers indices
        planexy_extract_indices.setInputCloud(voxel_cloud);
        planexy_extract_indices.setIndices(planexy_inliers);
        planexy_extract_indices.setNegative(true);
        planexy_extract_indices.filter(*planexy_cloud);
  //==================================== object Segmentation  ====================================
    pcl::PointCloud<PointT>::Ptr segmented_cluster (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr all_clusters (new pcl::PointCloud<PointT>);
    tree->setInputCloud (planexy_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;


        struct BBox
    {
      float x_min;
      float x_max;
      float y_min;
      float y_max;
      float z_min;
      float z_max;
      double r = 1.0;
      double g = 0.0;
      double b = 0.0;
    };
    ec.setClusterTolerance (0.4);  // Set the spatial cluster tolerance (distance) in meters. 
                              // Increasing this value will allow points that are farther apart to be considered part of the same cluster, resulting in fewer but larger clusters.
                              // Decreasing this value will make the clustering more strict, resulting in more but smaller clusters.
    ec.setMinClusterSize (75);  // Set the minimum number of points that a cluster needs to be considered valid.
                          // Increasing this value will ignore smaller clusters, which can help remove noise but may miss small objects.
                          // Decreasing this value will allow smaller clusters to be considered, which can include noise.
    ec.setMaxClusterSize (2000); // Set the maximum number of points that a cluster can have to be considered valid.
                           // Increasing this value will allow larger clusters to be considered, which can include large objects.
                           // Decreasing this value will ignore larger clusters, which can help focus on smaller objects.
    ec.setSearchMethod (tree);
    ec.setInputCloud (planexy_cloud);
    ec.extract (cluster_indices);
    std::vector<BBox> bboxes;

    size_t min_reasonable_size = 50; // Define the minimum reasonable size for a cluster to be considered valid.
                                  // Increasing this value will ignore smaller clusters, focusing on larger objects.
                                  // Decreasing this value will allow smaller clusters to be considered, which can include noise.
    size_t max_reasonable_size = 2000;  // Define the maximum reasonable size for a cluster to be considered valid.
                                   // Increasing this value will allow larger clusters to be considered, which can include large objects.
                                   // Decreasing this value will ignore larger clusters, focusing on smaller objects.
    int num_reasonable_clusters = 0;  // Initialize a counter for the number of clusters that fall within the reasonable size range.
    
    
    // Define the threshold value
    const float x_min_size_threshold = 0.8;
    const float x_max_size_threshold = 2.5;

    const float y_min_size_threshold = 0.8;
    const float y_max_size_threshold = 2.5;

    const float z_min_size_threshold = 1.0;
    const float z_max_size_threshold = 30.0;

    const float z_min_threshold = -0.2;

    const float square_ratio_threshold = 1.5;



    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        if (cluster_indices[i].indices.size() > min_reasonable_size && cluster_indices[i].indices.size() < max_reasonable_size)
        {
            pcl::PointCloud<PointT>::Ptr reasonable_cluster (new pcl::PointCloud<PointT>);
            pcl::ExtractIndices<PointT> extract;
            pcl::IndicesPtr indices(new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));
            extract.setInputCloud (planexy_cloud);
            extract.setIndices(indices);
            extract.setNegative (false);
            extract.filter (*reasonable_cluster);
            all_clusters->operator+=(*reasonable_cluster);
            num_reasonable_clusters++;

            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

            pcl::PointXYZ center((min_pt[0] + max_pt[0]) / 2.0, (min_pt[1] + max_pt[1]) / 2.0, (min_pt[2] + max_pt[2]) / 2.0);
            BBox bbox;
            bbox.x_min = min_pt[0];
            bbox.y_min = min_pt[1];
            bbox.z_min = min_pt[2];
            bbox.x_max = max_pt[0];
            bbox.y_max = max_pt[1];
            bbox.z_max = max_pt[2];

            // Check if the cluster is too big or too small. If so, skip it.
            if (
                bbox.x_max - bbox.x_min > x_max_size_threshold || bbox.x_max - bbox.x_min < x_min_size_threshold ||
                bbox.y_max - bbox.y_min > y_max_size_threshold || bbox.y_max - bbox.y_min < y_min_size_threshold || 
                bbox.z_max - bbox.z_min > z_max_size_threshold || bbox.z_max - bbox.z_min < z_min_size_threshold ||
                bbox.z_min > z_min_threshold ||
                (bbox.x_max - bbox.x_min)/(bbox.y_max - bbox.y_min) > square_ratio_threshold || (bbox.y_max - bbox.y_min)/(bbox.x_max - bbox.x_min) > square_ratio_threshold
              )
            {
                continue;
            }
            bboxes.push_back(bbox);
        }
    }

    //==================================== Drawing Boxes  ====================================

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;
    const std_msgs::msg::Header& inp_header = input_cloud->header;

    // Create a marker for each bounding box
    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::msg::Marker top_square_marker;
        top_square_marker.header = inp_header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::msg::Marker::ADD;
        top_square_marker.pose.orientation.w = 1.0;
        top_square_marker.scale.x = 0.06;
        top_square_marker.color.r = bbox.r;
        top_square_marker.color.g = bbox.g;
        top_square_marker.color.b = bbox.b;
        top_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Create the marker for the bottom square
        visualization_msgs::msg::Marker bottom_square_marker;
        bottom_square_marker.header = inp_header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::msg::Marker::ADD;
        bottom_square_marker.pose.orientation.w = 1.0;
        bottom_square_marker.scale.x = 0.04;
        bottom_square_marker.color.r = bbox.r;
        bottom_square_marker.color.g = bbox.g;
        bottom_square_marker.color.b = bbox.b;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the bottom square marker
        geometry_msgs::msg::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::msg::Marker connecting_lines_marker;
        connecting_lines_marker.header = inp_header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        connecting_lines_marker.pose.orientation.w = 1.0;
        connecting_lines_marker.scale.x = 0.04;
        connecting_lines_marker.color.r = 0.0;
        connecting_lines_marker.color.g = 1.0;
        connecting_lines_marker.color.b = 0.0;
        connecting_lines_marker.color.a = 1.0;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::msg::Marker corner_marker;
        corner_marker.header = inp_header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
        corner_marker.action = visualization_msgs::msg::Marker::ADD;
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.4;
        corner_marker.scale.y = 0.4;
        corner_marker.scale.z = 0.4;
        corner_marker.color.r = bbox.r;
        corner_marker.color.g = 0.2;
        corner_marker.color.b = 0.5;
        corner_marker.color.a = 0.64;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        marker_pub->publish(marker_array);
    }
  //==================================== Cloud publishing to ROS  ====================================

        // Convert cloud to ros2 message
        sensor_msgs::msg::PointCloud2 object_seg_ros2;
        pcl::toROSMsg(*planexy_cloud, object_seg_ros2);
        object_seg_ros2.header = input_cloud->header;
        // std::cout << "PointCloud size before voxelization: " << pcl_cloud->size() << std::endl;
        // std::cout << "PointCloud size after voxelization: " << voxel_cloud->size() << std::endl;

        publisher_->publish(object_seg_ros2);


  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGrid_filter>());
  rclcpp::shutdown();
  return 0;
}