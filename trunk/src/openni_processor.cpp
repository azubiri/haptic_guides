#include "openni_processor.h"


SimpleOpenNIProcessor::SimpleOpenNIProcessor() : cloud2(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), borders(new pcl::PointCloud<pcl::PointXYZ>), v1(0), v2(0)
{
  this->isSegmented = false;
  this->isShown = false;
  this->selected_cluster = 0;
  this->isCreated = false;
}

SimpleOpenNIProcessor::~SimpleOpenNIProcessor()
{
}

void SimpleOpenNIProcessor::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  if (! viewer->wasStopped() && !isSegmented)
  {
    pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cloud, *cloud2);
    filter.set_cloud(cloud2);
    filter.passthrough(this->min, this->max);
    filter.voxel_grid(this->leafSize);
    //filter.voxel_grid(0.01f);
    //filter.passthrough(0.5, 1);
    cloud_filtered = filter.get_cloud_filtered();

    CSegmentation seg;
    seg.set_cloud(cloud_filtered);
    seg.normal_estimation(this->k_normal);
    seg.region_growing(this->min_cluster_size, this->max_cluster_size, this->neighbour_number, this->theta);
    //seg.normal_estimation(50);
    //seg.region_growing(50, 1000, 30, 10.0);

    cloud_clusters = seg.clusters();
//std::cout << "Cloudcallback: Number of clusters: " << cloud_clusters.size() << std::end << std::flush;
    if(false)
    {
      cout << "Number of clusters: "<< cloud_clusters.size() << endl;
      cout << selected_cluster << endl;
      cout << cloud_clusters[selected_cluster]->size() << endl;
    }

    //this->computeBorders(0.01);
    this->computeBorders(this->alpha);
//==================================================================================================================================
//------------------Provisional-----------------------------------------------------------------------------------------------------
//==================================================================================================================================

//==================================================================================================================================
//==================================================================================================================================

    //if(true)
    //{
    //  static unsigned count = 0;
    //  static double last = pcl::getTime ();
    //  if (++count == 30)
    //  {
    //    double now = pcl::getTime ();
    //    std::cout << "Average framerate: " << double(count)/double(now - last) << " fps" <<  std::endl;
    //    count = 0;
    //    last = now;
    //  }
    //}

    if(!isShown)
    {
      viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "filtered point cloud", v1);
      viewer->addPointCloud<pcl::PointXYZ>(borders, "borders", v2);
      //viewer->addPointCloud<pcl::PointXYZ>(cloud_clusters[selected_cluster], "borders", v2);
      //viewer->addCoordinateSystem(0.5, v2);
      isShown = true;
    }
    viewer->updatePointCloud(cloud_clusters[selected_cluster], "borders");
    viewer->updatePointCloud(borders, "borders");
    //viewer->updatePointCloud(cloud_filtered, "filtered point cloud");

    if(false)
    {
      isSegmented = true;
    }
  }
}

void SimpleOpenNIProcessor::computeBorders(float alpha)
{
  // Object for retrieving the concave hull.
  pcl::ConcaveHull<pcl::PointXYZ> hull;
  pcl::PointCloud<pcl::PointXYZ>::iterator it;

  hull.setInputCloud(cloud_clusters[selected_cluster]);
  // Set alpha, which is the maximum length from a vertex to the center of the voronoi cell
  // (the smaller, the greater the resolution of the hull).
  //hull.setAlpha(0.01);
  hull.setAlpha(alpha);
  hull.reconstruct(*borders);

  listPoints.resize(borders->size());

  int jj = 0;
  bool write = false;

  if(write)
  {
    cout << "----------"<< "Borders: " << borders->size() << "------------" << endl;
  }
  for(it = borders->points.begin(); it < borders->points.end(); it++, jj++)
  {
    listPoints[jj].resize(3);
    listPoints[jj][0] = it->x;
    listPoints[jj][1] = it->y;
    listPoints[jj][2] = it->z;

    if(write)
    {
      cout << "Point " << jj << ": ";
      cout << "(" << listPoints[jj][0] << "," << listPoints[jj][1] << "," << listPoints[jj][2] << ")" << endl;
    }
  }
}

void SimpleOpenNIProcessor::setAlpha(float alpha)
{
  this->alpha = alpha;
}

void SimpleOpenNIProcessor::getEdges(std::vector<std::vector<float>> &list)
{
  list = listPoints;
}

void SimpleOpenNIProcessor::numberPointsEdge(int &nPoints)
{
   nPoints = listPoints.size();
}

void SimpleOpenNIProcessor::setSegParameters(int k_normal, int min_cluster_size, int max_cluster_size, int neighbour_number, float theta)
{
  this->k_normal = k_normal;
  this->min_cluster_size = min_cluster_size;
  this->max_cluster_size = max_cluster_size;
  this->neighbour_number = neighbour_number;
  this->theta = theta;
}

void SimpleOpenNIProcessor::setPassParameters(float min, float max)
{
  this->min = min;
  this->max = max;
}

void SimpleOpenNIProcessor::setVoxParameters(float leafSize)
{
  this->leafSize = leafSize;
}

void SimpleOpenNIProcessor::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event)
{

  if (event.keyDown())
  {
    if(event.getKeySym() == "space")
    {
      isSegmented = !isSegmented;
      this->selected_cluster = 0;
      viewer->updatePointCloud(cloud_clusters[selected_cluster], "cloud cluster");
    }

    else if((event.getKeySym() == "Down" || event.getKeySym() == "Left") && this->selected_cluster > 0)
    {
      this->selected_cluster--;
      viewer->updatePointCloud(cloud_clusters[selected_cluster], "cloud cluster");
    }

    else if((event.getKeySym() == "Up" || event.getKeySym() == "Right") && this->selected_cluster < cloud_clusters.size())
    {
      this->selected_cluster++;
      viewer->updatePointCloud(cloud_clusters[selected_cluster], "cloud cluster");
    }

    else if(event.getKeyCode() >= 48 && (event.getKeyCode() <= cloud_clusters.size() + 48))
    {
      this->selected_cluster = event.getKeyCode() - 48;
      viewer->updatePointCloud(cloud_clusters[selected_cluster], "cloud cluster");
    }

    else
    {
      cout << "Please, select another cluster" << endl;
    }
//
//        std::string ss = event.getKeySym();
//        int i = atoi(ss.c_str());
//        cout << ss << endl;
//
//        int num = event.getKeyCode();
//        cout << num << endl;

    if(false)
    {
      cout << "Selected cluster: "<< selected_cluster << endl;
    }
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> SimpleOpenNIProcessor::createViewer()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> v(new pcl::visualization::PCLVisualizer("OpenNI viewer"));
  v->initCameraParameters();

  v->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  v->setBackgroundColor(0, 0, 0, v1);
  v->addText("Filtered point cloud", 10, 10, "v1 text", v1);

  v->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  v->setBackgroundColor(0, 0, 0, v2);
  v->addText("Border of the cluster", 10, 10, "v2 text", v2);

  boost::function<void(const pcl::visualization::KeyboardEvent&)> kk;
  kk = boost::bind(&SimpleOpenNIProcessor::keyboardEventOccurred, this, _1); //
  v->registerKeyboardCallback(kk);

  return (v);
}


void SimpleOpenNIProcessor::run ()
{
  
  std::cout << "Run: " << " MIN: "<< this->min << "  MAX: " << this->max << "  LeafSize: " << this->leafSize << std::endl << std::flush;

  // create a new grabber for OpenNI devices
  std::cout << "SimpleOpenNIProcessor::run: INI" << std::endl << std::flush;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();

  // make callback function from member function
  boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
    boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

  // connect callback function for desired signal. In this case its a point cloud with color values
  boost::signals2::connection c = interface->registerCallback (f);

  viewer = createViewer();

  // start receiving point clouds
  interface->start ();

  // wait until user quits program with "q", but no busy-waiting -> sleep (1);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(1);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  // stop the grabber
  interface->stop ();
  delete interface;
  std::cout << "SimpleOpenNIProcessor::run: OUT" << std::endl << std::flush;
}
