#pragma once
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <thread>



template<typename PointT>
class planeSegmentationAndClustering
{
private:
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered; 
  typename pcl::PointCloud<PointT>::Ptr cloud_f; 
  typename pcl::PointCloud<PointT>::Ptr cloud_p; 
  typename pcl::PointCloud<PointT>::Ptr cloud_blob; 
  typename pcl::PointCloud<PointT>::Ptr cloud_segmented; 
  typename pcl::PointCloud<PointT>::Ptr cloud_segmented2; 
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered_z; 
  double voxelLeafSize;
  int maxIterations;
  double fractionOfCloud;
  double minClusterSize;
  double maxClusterSize;
  

public:
  planeSegmentationAndClustering(typename pcl::PointCloud<PointT>::Ptr cloud_filtered, typename pcl::PointCloud<PointT>::Ptr cloud_blob, typename pcl::PointCloud<PointT>::Ptr cloud_p, typename pcl::PointCloud<PointT>::Ptr cloud_segmented,
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_z, typename pcl::PointCloud<PointT>::Ptr cloud_f, typename pcl::PointCloud<PointT>::Ptr cloud_segmented2){
    this->cloud_filtered = cloud_filtered;
    this->cloud_filtered_z = cloud_filtered_z;
    this->cloud_f = cloud_f;
    this->cloud_blob = cloud_blob;
    this->cloud_segmented = cloud_segmented;
    this->cloud_p = cloud_p;
    this->cloud_segmented2 = cloud_segmented2;
    this->voxelLeafSize = 0.01; //Voxel side 1cm
    this->maxIterations = 1000; //RANSAC Max Iterations
    this->fractionOfCloud = 0.3; //continue segmenting until 30% of the cloud remains
    this->minClusterSize = 100; //smallest cluster size of 100 points
    this->maxClusterSize = 25000; //largest cluster size of 25000
 


    
  }
    
  ~planeSegmentationAndClustering()
  {
  }
  //Voxel Grid Downsampling object
  pcl::VoxelGrid<PointT> sor;

  //Object to write PCD file to disk
  pcl::PCDWriter writer;

  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  //Object for Euclidean Cluster Extraction
  pcl::EuclideanClusterExtraction<PointT> ec;

  //method to get down sampled clouds
  typename pcl::PointCloud<PointT>::Ptr getDownSampledCloud(typename pcl::PointCloud<PointT>::Ptr cloud_blob);

  // //method to write PCDs on disk
  void writeToDisk(typename pcl::PointCloud<PointT>::Ptr cloud);

  // //method to get Plane segmented outputs
  void getSegmentedOutputs(typename pcl::PointCloud<PointT>::Ptr cloud_filtered, double distanceThreshold);

  // //method to get clusters
  void getClusters(typename pcl::PointCloud<PointT>::Ptr cloud_segmented,double clusterThreshold);

};