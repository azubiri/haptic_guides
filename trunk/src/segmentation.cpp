#include "segmentation.h"


CSegmentation::CSegmentation() : normals(new pcl::PointCloud <pcl::Normal>), cloud(new pcl::PointCloud<pcl::PointXYZ>), indices(new std::vector <int>)
{
  this->tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
}

CSegmentation::~CSegmentation()
{
}

void CSegmentation::normal_estimation(int k)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (k);
  normal_estimator.compute (*normals);
}

void CSegmentation::region_growing(int min_cluster_size, int max_cluster_size, int neighbour_number, float theta)
{
  reg.setMinClusterSize (min_cluster_size);
  reg.setMaxClusterSize (max_cluster_size);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (neighbour_number);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (theta / 180.0 * M_PI);
  //reg.setCurvatureThreshold (1.0);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > CSegmentation::clusters()
{
  std::vector <pcl::PointIndices> cluster_indices;
  reg.extract (cluster_indices);

  int j = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_clusters(cluster_indices.size());
  for (int ii = 0; ii < cloud_clusters.size(); ++ii)
  {
    cloud_clusters[ii].reset(new pcl::PointCloud<pcl::PointXYZ>);
  }

  //pcl::PointCloud<pcl::PointXYZ> cloud_temp;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);


  //cout << "Nº clusters: " << cluster_indices.size() << endl;

  for (std::vector<pcl::PointIndices>::const_iterator i_cluster = cluster_indices.begin (); i_cluster != cluster_indices.end (); ++i_cluster)
  {

    for (std::vector<int>::const_iterator pi_cluster = i_cluster->indices.begin (); pi_cluster != i_cluster->indices.end (); ++pi_cluster)
    {
      cloud_temp->points.push_back (cloud->points[*pi_cluster]); //*
    }

    cloud_temp->width = cloud_temp->points.size ();
    cloud_temp->height = 1;
    cloud_temp->is_dense = true;

    *cloud_clusters[j] = *cloud_temp;

    cloud_temp->clear();
    j++;
  }

  return (cloud_clusters);
}

void CSegmentation::set_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  this->cloud = cloud;
}
