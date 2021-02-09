#include "choose_filter.h"

CChoose_Filter::CChoose_Filter() : cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
{
}

CChoose_Filter::~CChoose_Filter()
{
}

void CChoose_Filter::passthrough(int min, int max)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min, max);
  pass.filter (*cloud_filtered);
  this->update_cloud();
}

void CChoose_Filter::voxel_grid(float leafSize)
{
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (leafSize, leafSize, leafSize);
  vg.filter (*cloud_filtered);
  this->update_cloud();
}

void CChoose_Filter::outlier_remove()
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
  this->update_cloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CChoose_Filter::get_cloud_filtered()
{
  return cloud_filtered;
}

void CChoose_Filter::set_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  this->cloud = cloud;
}

void CChoose_Filter::update_cloud()
{
  this->cloud = cloud_filtered;
}
