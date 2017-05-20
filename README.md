# A Particle Filter filter for position tracking using LIDAR and RADAR processed measurements

## Brief Description of the ALgorithm
This is a standard particle filter for localization with a known map (i.e., a list of landmarks) that uses a bicycle model for transitioning. Each measurements comprises a set of observed distances to (unmatched) landmarks. These landmarks are transformed to the global coordinate frame and nearest - neighbor matching aligns them with the most suitable correspondences in the map. The measurement is simply a quadratic penalty on the distance to each nearest neighbor in the map. It is assumed that all measurements have correspondences in the map, so no measurements are discarded and further checks are made for spatial consistency with the map, based on the pose of the particle.  

## Compiling and Running
```
cd build
cmake ..
make
./ExtendedKF sample-laser-radar-measurement-data-1.txt output1.txt
./ExtendedKF sample-laser-radar-measurement-data-2.txt output2.txt
```
## Fast Search in the Map
Except for the standard resampling and weight update steps which are pretty straightforward, the filter makes use of a map that contains a **KD-tree** that holds the coordinates of the map landmarks. This way, nearest neighbor matching happens blazingly fast. I am very confident that a **quad-tree** whould probably have a marginally better performance, but I really enjoyed implemented the 2D KD-tree structure.
