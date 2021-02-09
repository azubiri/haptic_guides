#ifndef _SEGMENTATION_H
#define _SEGMENTATION_H

#include <iostream>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

class CSegmentation
{
  private:

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    pcl::search::Search<pcl::PointXYZ>::Ptr tree;

    pcl::PointCloud <pcl::Normal>::Ptr normals;

    pcl::IndicesPtr indices ;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;

  public:

    CSegmentation();

    ~CSegmentation(); 

    void normal_estimation(int k);

    void region_growing(int min_cluster_size, int max_cluster_size, int neighbour_number, float theta);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters();

    void set_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

};

#endif
