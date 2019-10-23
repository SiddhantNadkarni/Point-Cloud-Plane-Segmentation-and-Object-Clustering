#include "SegmentationandClustering.h"
#include "planeSegmentation.cpp"

 
pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::string s)
{

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (s));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  // viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}




int
main (int argc, char** argv)
{
	if (argc<5){
		std::cout << "[Executable] [inputData] [planeSegmentationThreshold] [clusterThreshold1] [clusterThreshold2]" << std::endl;
		exit(1);
	}

	//Initialize all point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented2 (new pcl::PointCloud<pcl::PointXYZRGB>);

  planeSegmentationAndClustering<pcl::PointXYZRGB> pc(cloud_filtered, cloud_blob, cloud_p, cloud_segmented, cloud_filtered_z, cloud_f, cloud_segmented2);

  // Fill in the cloud data
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read (argv[1], *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  cloud_filtered = pc.getDownSampledCloud(cloud_blob);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pc.writeToDisk(cloud_filtered);

  //threshold for plane segmentation
  double distanceThreshold = atof(argv[2]);

  //plane segmentation using RANSAC
  pc.getSegmentedOutputs(cloud_filtered, distanceThreshold);

  //Save the output after the
  reader.read("outputs/Plane_Segmented_Output_0.pcd", *cloud_segmented);



  //Get and Write Clusters of plane segmented object
  double clusterThreshold1 = atof(argv[3]);
  pc.getClusters(cloud_segmented, clusterThreshold1); 

  //Point Clouds to Visualize clusters
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr2 (new pcl::PointCloud<pcl::PointXYZRGB>);

  reader.read("outputs/cloud_cluster_0.pcd", *basic_cloud_ptr1);
  reader.read("outputs/cloud_cluster_1.pcd", *basic_cloud_ptr2);

  //visualization objects
  pcl::visualization::PCLVisualizer::Ptr viewer5;
  pcl::visualization::PCLVisualizer::Ptr viewer6;

  viewer5 = simpleVis(basic_cloud_ptr2,"cloud_cluster_1");
  viewer6 = simpleVis(basic_cloud_ptr1,"cloud_cluster_0");

  int condition(0); //This condition is used to recluster the zeroth cluster; default value is fault
  int clusterNum(0); //default cluster to clusterized is 0

  while (!viewer6->wasStopped ())
  {

    viewer5->spinOnce (100);
    viewer6->spinOnce (100);
    std::cout << "Do you want to recluster the zeroth cluster? Enter 1 if you want to." << std::endl;
    std::cin >> condition;
    


    if(condition == 0 || 1)
    	break;

   }

   //if condition is set to True, the zeroth cluster is reclustered
  if(condition)
  {
  	  //view the first two clusters
  	

  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr3 (new pcl::PointCloud<pcl::PointXYZRGB>);
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr4 (new pcl::PointCloud<pcl::PointXYZRGB>);
  	std::stringstream ss;

  	std::cout << "Enter the Cluster Number that you wish to recluster i.e. either 0 or 1" << std::endl;
  	std::cin >> clusterNum;

  	ss << "outputs/cloud_cluster_" << clusterNum << ".pcd";
  	std::cout << ss.str() << std::endl;



  	reader.read(ss.str(), *cloud_segmented2);
  	double clusterThreshold2 = atof(argv[4]);
  	// pc.getClusters(cloud_segmented2, clusterThreshold2);


  	reader.read("outputs/cloud_cluster_0.pcd", *basic_cloud_ptr1);
  	reader.read("outputs/cloud_cluster_1.pcd", *basic_cloud_ptr2);
  	reader.read("outputs/cloud_cluster_2.pcd", *basic_cloud_ptr3);
  	reader.read("outputs/Plane_Segmented_Output_0.pcd", *basic_cloud_ptr4);


  	pcl::visualization::PCLVisualizer::Ptr viewer1;
  	pcl::visualization::PCLVisualizer::Ptr viewer2;
  	pcl::visualization::PCLVisualizer::Ptr viewer3;
  	pcl::visualization::PCLVisualizer::Ptr viewer4;


    //visualization objects
  	viewer1 = simpleVis(basic_cloud_ptr4, "Plane_Segmented_Output_0");
  	viewer2 = simpleVis(basic_cloud_ptr3, "cloud_cluster_2");
  	viewer3 = simpleVis(basic_cloud_ptr2,"cloud_cluster_1");
  	viewer4 = simpleVis(basic_cloud_ptr1,"cloud_cluster_0");


  	while (!viewer4->wasStopped ())
  	{
  		viewer4->spinOnce (100);
  		viewer3->spinOnce (100);
  		viewer2->spinOnce (100);
  		viewer1->spinOnce (100);


    //std::this_thread::sleep_for(100);
  	}

  }

  return (0);
}
