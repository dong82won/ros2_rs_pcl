// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/common.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/passthrough.h>          // [추가] PassThrough 필터 헤더
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>

// #include <pcl/kdtree/kdtree.h>
// #include <pcl/segmentation/extract_clusters.h>

// #include "ros2_rs_pcl/clustering/rspcl_clustering_component.hpp"


// RspclClusteringComponent::RspclClusteringComponent() : Node("pclsub")
// {
//   subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//     "/camera/pointcloud",
//     10,
//     std::bind(&RspclClusteringComponent::timer_callback, this, std::placeholders::_1)
//   );

//   using namespace std::chrono_literals;
//   publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);
// }


// void RspclClusteringComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
// {
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::fromROSMsg(*cloud_msg, *cloud);

//   RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)", cloud_msg->height, cloud_msg->width);

//   // 1. [필수] NaN 및 Inf 유효하지 않은 점 제거
//   std::vector<int> nan_indices;
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nan_removed(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_removed, nan_indices);

//   if (cloud_nan_removed->points.empty()) {
//     return;
//   }

//   // 2. [필수] 관심 영역(ROI) 필터링 (리얼센스 카메라 기준 전방 거리 z축)
//   // 0.2m ~ 2.5m 사이 영역만 추출하여 원거리 노이즈 및 오버플로우 방지
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::PassThrough<pcl::PointXYZRGB> pass;
//   pass.setInputCloud(cloud_nan_removed);
//   pass.setFilterFieldName("z");
//   pass.setFilterLimits(0.2, 2.5);
//   pass.filter(*cloud_pass);

//   if (cloud_pass->points.empty()) {
//     return;
//   }

//   // 3. Voxel Grid 다운샘플링
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
//   voxelGrid.setInputCloud(cloud_pass);
//   voxelGrid.setLeafSize(0.02f, 0.02f, 0.02f);
//   voxelGrid.filter(*cloud_filtered);

//   if (cloud_filtered->points.empty()) {
//     return;
//   }

//   // 4. 바닥/테이블 평면 검출 (SAC Segmentation)
//   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//   pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//   double threshold = 0.015; // 평면 두께 임계값 (1.5cm)
//   seg.setOptimizeCoefficients(true);
//   seg.setModelType(pcl::SACMODEL_PLANE);
//   seg.setMethodType(pcl::SAC_RANSAC);
//   seg.setDistanceThreshold(threshold);
//   seg.setInputCloud(cloud_filtered);
//   seg.segment(*inliers, *coefficients);

//   if (inliers->indices.empty()) {
//     return;
//   }

//   // 5. 검출된 평면(바닥/책상) 제거 -> 물체 점군만 추출
//   pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//   extract.setInputCloud(cloud_filtered);
//   extract.setIndices(inliers);
//   extract.setNegative(true); // 바닥을 제외한 나머지 반환
//   extract.filter(*cloud_filtered);

//   if (cloud_filtered->points.size() < 20) {
//     return; // 클러스터링을 수행하기에 점이 너무 적으면 건너뜀
//   }

//   // 6. KdTree 기반 유클리디안 클러스터링 (물체 개별 분할)
//   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//   tree->setInputCloud(cloud_filtered);

//   std::vector<pcl::PointIndices> cluster_indices;
//   pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
//   ece.setClusterTolerance(0.03); // 같은 물체로 묶을 최대 점 간격 (3cm)
//   ece.setMinClusterSize(30);     // 최소 점 개수
//   ece.setMaxClusterSize(10000);  // 최대 점 개수
//   ece.setSearchMethod(tree);
//   ece.setInputCloud(cloud_filtered);
//   ece.extract(cluster_indices);

//   // 7. 각 클러스터(물체)마다 고유 색상 부여
//   int j = 0;
//   float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);

//   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
//     for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
//       cloud_cluster->points[*pit].r = colors[j % 6][0];
//       cloud_cluster->points[*pit].g = colors[j % 6][1];
//       cloud_cluster->points[*pit].b = colors[j % 6][2];
//     }
//     j++;
//   }

//   // 8. 결과 토픽 발행
//   sensor_msgs::msg::PointCloud2 sensor_msg;
//   pcl::toROSMsg(*cloud_cluster, sensor_msg);
//   sensor_msg.header.frame_id = cloud_msg->header.frame_id; // 원본 프레임 ID 계승
//   publisher_->publish(sensor_msg);
// }

#include "ros2_rs_pcl/clustering/rspcl_clustering_component.hpp"
#include <rclcpp/qos.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

RspclClusteringComponent::RspclClusteringComponent() : Node("pcl_clustering_node")
{
  // [개선 3] ROS 2 Parameter Server 연동
  this->declare_parameter("z_min", 0.2);
  this->declare_parameter("z_max", 1.0);

  this->declare_parameter("leaf_size", 0.02);
  this->declare_parameter("ransac_threshold", 0.015);
  this->declare_parameter("cluster_tolerance", 0.03);
  this->declare_parameter("min_cluster_size", 30);
  this->declare_parameter("max_cluster_size", 10000);

  // [개선 2] SensorDataQoS 적용 (네트워크 지연 방지 및 실시간성 확보)
  auto qos = rclcpp::SensorDataQoS();

  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/pointcloud", qos,
    std::bind(&RspclClusteringComponent::timer_callback, this, std::placeholders::_1)
  );

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", qos);
}

void RspclClusteringComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  // 파라미터 동적 로드
  double z_min = this->get_parameter("z_min").as_double();
  double z_max = this->get_parameter("z_max").as_double();
  double leaf_size = this->get_parameter("leaf_size").as_double();
  double ransac_thresh = this->get_parameter("ransac_threshold").as_double();
  double cluster_tol = this->get_parameter("cluster_tolerance").as_double();
  int min_cluster = this->get_parameter("min_cluster_size").as_int();
  int max_cluster = this->get_parameter("max_cluster_size").as_int();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // [개선 1] In-place 메모리 최적화 (포인터 하나로 필터 체인 덮어쓰기)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered, nan_indices);
  if (cloud_filtered->points.empty()) return;

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*cloud_filtered);
  if (cloud_filtered->points.empty()) return;

  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_filtered);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*cloud_filtered);
  if (cloud_filtered->points.empty()) return;

  // [개선 4] 평면 추출 강건성 증대 (Z축 방향에 수직인 바닥면만 타겟팅)
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);

  // seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  // [수정] 범용 평면 모델로 변경하고, 제약 조건(setAxis, setEpsAngle)을 삭제합니다.
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ransac_thresh);

  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // 카메라 좌표계 기준 Z축(전방) 평면
  seg.setEpsAngle(15.0f * (M_PI / 180.0f));    // 허용 오차 15도
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  if (!inliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
  }

  if (cloud_filtered->points.size() < (size_t)min_cluster) return;

  // KdTree 기반 클러스터링
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
  ece.setClusterTolerance(cluster_tol);
  ece.setMinClusterSize(min_cluster);
  ece.setMaxClusterSize(max_cluster);
  ece.setSearchMethod(tree);
  ece.setInputCloud(cloud_filtered);
  ece.extract(cluster_indices);

  int j = 0;
  float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud_filtered, *cloud_cluster);

  for (const auto& cluster : cluster_indices) {
    for (const auto& idx : cluster.indices) {
      cloud_cluster->points[idx].r = colors[j % 6][0];
      cloud_cluster->points[idx].g = colors[j % 6][1];
      cloud_cluster->points[idx].b = colors[j % 6][2];
    }
    j++;
  }

  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_cluster, sensor_msg);
  sensor_msg.header = cloud_msg->header; // 프레임 및 타임스탬프 동기화
  publisher_->publish(sensor_msg);
}
