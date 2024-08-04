# What's this?
ROS 2 node that processes LIDAR point cloud data by filtering and segmenting it to detect and visualize objects. It applies pass-through filters and voxel grid downsampling to preprocess the point cloud, segments out planar surfaces using RANSAC, and then clusters the remaining points to identify distinct objects. Bounding boxes are computed and visualized around these objects, and the processed point cloud is published along with the visualization markers for display.

I tried to cluster the trunks of oil palm trees. Played around with different parameters and thresholds to get better results, but unfortunately, the outcome wasn’t as good as I hoped. Need further enhancement or try different method...

Video link: https://www.youtube.com/watch?v=k151XyvtdmQ
[![240804 - Robotics Weekend - Point cloud clustering with PCL Library](https://img.youtube.com/vi/k151XyvtdmQ/0.jpg)](https://www.youtube.com/watch?v=k151XyvtdmQ)


# Changelog
|Date|Description|
|-|-|
|24-08-04|New|