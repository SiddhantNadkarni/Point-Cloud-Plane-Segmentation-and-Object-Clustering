#include "SegmentationandClustering.hpp"





pcl::PointCloud<pcl::PointXYZ>::Ptr planeSegmentationAndClustering::getDownSampledCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob){ 
  
  sor.setInputCloud (cloud_blob);
  //set Voxels with side in cms
  sor.setLeafSize (voxelLeafSize, voxelLeafSize, voxelLeafSize);
  sor.filter (*cloud_filtered);
  return cloud_filtered;
}



void planeSegmentationAndClustering::writeToDisk(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  writer.write<pcl::PointXYZ> ("outputs/downsampled_scene_cloud.pcd", *cloud, false);

}

void planeSegmentationAndClustering::getSegmentedOutputs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, double distanceThreshold){
  

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  
  //RANSAC Model type
  seg.setModelType (pcl::SACMODEL_PLANE);

  //RANSAC Method type
  seg.setMethodType (pcl::SAC_MSAC);

  //RANSAC Max Iterations
  seg.setMaxIterations (maxIterations);

  //determines how close a point must be to the model in order to be considered an inlier
  seg.setDistanceThreshold (distanceThreshold);

  
 

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While fractionOfCloud % of the original cloud is still there
  while (cloud_filtered->points.size () > fractionOfCloud * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::stringstream ss;

    
    ss << "outputs/Plane_Segmented_Output_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;

  }
}

void planeSegmentationAndClustering::getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented, double clusterThreshold){
  
  //create a KDtree for searching
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_segmented);
  *cloud_filtered = *cloud_segmented;

  //The indices of each detected cluster are saved here
  std::vector<pcl::PointIndices> cluster_indices;

  //Threshold to determine how close the points should be to cluster as a single Object
  ec.setClusterTolerance (clusterThreshold); //2cms

  //Min cluster size
  ec.setMinClusterSize (minClusterSize);

  //Max Cluster size
  ec.setMaxClusterSize (maxClusterSize);

  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  //loop through each vector of cluster pointIndices and save them as clouds
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "outputs/cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
}




  