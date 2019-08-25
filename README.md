# Point-Cloud-Plane-Segmentation-and-Object-Clustering
**Used to Segment Objects and obtain multiple clusters of objects kept on a table captured by a Microsoft kinect.**

# Introduction 

#### Below is the PCD file obtained through Kinect OpenNI Grabber. We can see multiple objects kept on a table top.

![Original Scene Point Cloud](https://user-images.githubusercontent.com/19183728/57191500-6604e200-6edb-11e9-91ec-31873147368b.png)

#### Below is the Point Clouds obtained after 1st round of Euclidean Clustering and Plane Segmentation

![Clustered Scene](https://user-images.githubusercontent.com/19183728/57191593-32768780-6edc-11e9-8299-e6fc67c7dfe2.png)

#### We can see above that since the Objects are within 2cms of each other, they are clustered out together. To get them into two different clusters, we run the Clustering algorithm second time with 1cm as the distance threshold.

![Dome Clustered Out](https://user-images.githubusercontent.com/19183728/57191653-d6603300-6edc-11e9-9a91-08e5e31720ff.png)

![Heliblade Clustered Out](https://user-images.githubusercontent.com/19183728/57191661-f4c62e80-6edc-11e9-958b-a4d39770324b.png)

## Installation Instructions

```javascript
git clone https://github.com/SiddhantNadkarni/PointCloudPlaneSegmentationAndObjectClustering.git
cd PointCloudPlaneSegmentationAndObjectClustering && mkdir build && cd build
cmake ..
./planar_segmentation inputDataMarch2019/inputCloud20.pcd 0.02 0.02 0.01
[Executable] [inputData] [planeSegmentationThreshold] [FirstRoundClusterThreshold1] [SecondRoundClusterThreshold]
```

