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




class planeSegmentationAndClustering
{
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented2; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z; 
  double voxelLeafSize;
  int maxIterations;
  double fractionOfCloud;
  double minClusterSize;
  double maxClusterSize;
  

public:
  planeSegmentationAndClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented2){
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
    
  ~planeSegmentationAndClustering(){
  }
  //Voxel Grid Downsampling object
  pcl::VoxelGrid<pcl::PointXYZ> sor;

  //Object to write PCD file to disk
  pcl::PCDWriter writer;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  //Object for Euclidean Cluster Extraction
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  //method to get down sampled clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr getDownSampledCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob);

  //method to write PCDs on disk
  void writeToDisk(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  //method to get Plane segmented outputs
  void getSegmentedOutputs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, double distanceThreshold);

  //method to get clusters
  void getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented,double clusterThreshold);

};