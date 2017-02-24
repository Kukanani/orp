#include "orp/core/region_monitor.h"

/**
 * Start the segmentation node and all ROS publishers
 * @param  argc num args
 * @param  argv args
 * @return      1 if all is well.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "region_monitor");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  ROS_INFO("Starting Region Monitor");
  RegionMonitor r;
  r.run();
  return 1;
} //main

RegionMonitor::RegionMonitor() :
  node("region_monitor"),
  transformToFrame(""),
  listener(),
  spinner(4)
{
  ros::NodeHandle privateNode("~");
  if(!privateNode.getParam("clippingFrame", transformToFrame)) {
    transformToFrame = "world";
  }

  inputCloud = PCP(new PC());
  processCloud = PCP(new PC());

  boundedScenePublisher = privateNode.advertise<sensor_msgs::PointCloud2>("bounded_scene",1);
  
  regionSub = privateNode.subscribe("set_region", 1000, &RegionMonitor::cb_params, this);
  pointCloudSub = node.subscribe("/rosarnl_node/S3Series_1_pointcloud", 1000, &RegionMonitor::cb_pointCloud, this);

  monitorServer = privateNode.advertiseService("monitor", &RegionMonitor::cb_monitor, this);
}

void RegionMonitor::run() {
  ROS_INFO("Region Monitor running...");
  spinner.start();
  ros::waitForShutdown();
}


void RegionMonitor::cb_params(const orp::Region::ConstPtr& region)
{
  minX = region->min_x;
  maxX = region->max_x;
  minY = region->min_y;
  maxY = region->max_y;
  minZ = region->min_z;
  maxZ = region->max_z;
  ROS_INFO("[region_monitor] Monitoring region set.");
}

bool RegionMonitor::cb_monitor(orp::MonitorRequest& req, orp::MonitorResponse& res)
{
  // ROS_INFO("clipping by distance.");
  PCP blah = clipByDistance(inputCloud);

  // ROS_INFO("checking occupation");
  res.occupied = !(blah->empty());

  // ROS_INFO("publishing point cloud");
  sensor_msgs::PointCloud2 outgoing;
  pcl::toROSMsg(*blah, outgoing);
  boundedScenePublisher.publish(outgoing);
  // ROS_INFO("done");
  return true;
}

void RegionMonitor::cb_pointCloud(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  // ROS_INFO("received input cloud");
  sensor_msgs::PointCloud2 interimPC2, transformedPC2;

  // ROS_INFO("converting to point cloud 2");
  sensor_msgs::convertPointCloudToPointCloud2 (*cloud, interimPC2);
  // ROS_INFO("transforming into other frame");
  pcl_ros::transformPointCloud (transformToFrame, interimPC2, transformedPC2, listener);
  
  // ROS_INFO("converting from ros message to PCL point cloud");
  inputCloud->points.clear();
  pcl::fromROSMsg(transformedPC2, *inputCloud);
}

bool compareClusterSize(const sensor_msgs::PointCloud2& a, const sensor_msgs::PointCloud2& b) { return a.width > b.width; }

PCP& RegionMonitor::clipByDistance(PCP &unclipped) {

  processCloud->resize(0);

  // We must build a condition.
  // And "And" condition requires all tests to check true. "Or" conditions also available.
  // Checks available: GT, GE, LT, LE, EQ.
  pcl::ConditionAnd<ORPPoint>::Ptr condition(new pcl::ConditionAnd<ORPPoint>);
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("x", pcl::ComparisonOps::GT, minX)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("x", pcl::ComparisonOps::LT, maxX)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("y", pcl::ComparisonOps::GT, minY)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("y", pcl::ComparisonOps::LT, maxY)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("z", pcl::ComparisonOps::GT, minZ)));
  condition->addComparison(pcl::FieldComparison<ORPPoint>::ConstPtr(
    new pcl::FieldComparison<ORPPoint>("z", pcl::ComparisonOps::LT, maxZ)));
 
  // Filter object.
  pcl::ConditionalRemoval<ORPPoint> filter(condition);
  filter.setInputCloud(unclipped);
  // If true, points that do not pass the filter will be set to a certain value (default NaN).
  // If false, they will be just removed, but that could break the structure of the cloud
  // (organized clouds are clouds taken from camera-like sensors that return a matrix-like image).
  filter.setKeepOrganized(false);
  // If keep organized was set true, points that failed the test will have their Z value set to this.
  filter.setUserFilterValue(0.0);
 
  filter.filter(*processCloud);
  return processCloud;
}