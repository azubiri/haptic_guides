#ifndef _SIMPLEOPENNIPROCESSOR_H
#define _SIMPLEOPENNIPROCESSOR_H

#include <iostream>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/Vertices.h>

#include "choose_filter.h"
#include "segmentation.h"

class SimpleOpenNIProcessor
{
  private:

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    CChoose_Filter filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, cloud_filtered, borders;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_clusters;
    std::vector<std::vector<float>> listPoints;
    bool isSegmented, isShown, isCreated;
    int selected_cluster, v1, v2;
    int k_normal, min_cluster_size, max_cluster_size, neighbour_number;
    float theta;
    float min, max, leafSize;
    float alpha;
    

  public:

    SimpleOpenNIProcessor();
    ~SimpleOpenNIProcessor();

    void getEdges(std::vector<std::vector<float>> &list);
    void numberPointsEdge(int &nPoints);
    void setSegParameters(int k_normal, int min_cluster_size, int max_cluster_size, int neighbour_number, float theta);
    void setPassParameters(float min, float max);
    void setVoxParameters(float leafSize);
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
    void computeBorders(float alpha);
    void setAlpha(float alpha);
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer();
    void run ();

};

#endif
