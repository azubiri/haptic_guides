#ifndef _CHOOSE_FILTER_H
#define _CHOOSE_FILTER_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>



/** \class CChoose_Filter
 *  \brief A point cloud filtering
 *  
 *  This class filters a point cloud with the selected filter.
 **/
class CChoose_Filter
{
  private:

    // Point cloud before the filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    // Point cloud after the filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

  public:

    /** \brief Contructor
     **/
    CChoose_Filter();

    /** \brief Destructor
     **/
    ~CChoose_Filter();

    /** \brief A filter passthrough
     *
     *  \param min Minimum distance which the filter allows to pass points
     *  \param max Maximum distance which the filter allows to pass points
     **/
    void passthrough(int min, int max);

    /** \brief A voxel grid filter
     *
     *  Assembles a local 3D grid over a given PointCloud, and downsamples + filters the data
     **/
    void voxel_grid(float leafSize);

    /** \brief A filter noise
     *
     *  \param leafSize The voxel grid leaf size
     **/
    void outlier_remove();

    /** \brief Get the point cloud filtered
     *
     *  This method allows to obtain the point cloud when it has been filtered.
     **/
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_filtered();

    /** \brief Set the captured point cloud to filter
     *
     *  This method allows what point cloud will filter.
     **/
    void set_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void update_cloud();
};

#endif
