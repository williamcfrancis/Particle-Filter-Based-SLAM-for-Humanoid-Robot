# SLAM with a Particle Filter

Simultaneous Localization and Mapping (SLAM) problem is a well-known problem in robotics, where a robot has to localize itself and map its environment simultaneously. Particle filter (PF) is one of the most adapted estimation algorithms for SLAM apart from Kalman filter (KF) and Extended Kalman Filter (EKF).\
Here I implement SLAM using a particle filter on data collected from a humanoid named THOR that was built at Penn and UCLA.

## Hardware Setup of Thor

The humanoid has a [Hokuyo LiDAR sensor](https://hokuyo-usa.com/products/lidar-obstacle-detection) on its head. This LiDAR is a planar LiDAR sensor and returns 1080 readings at each instant, each reading being the distance of some physical object along a ray that shoots off at an angle between (-135, 135) degrees with discretization of 0.25 degrees in an horizontal plane.

![image](https://user-images.githubusercontent.com/38180831/205529594-cda1e25e-a384-4bf2-b0ca-0a23c0719735.png)

Here I use the position and orientation of the head of the robot to calculate the orientation of the LiDAR in the body frame. The second kind of observations I used pertain to the location of the robot. I  directly used the (x,y,θ) pose of the robot in the world coordinates (θ denotes yaw). These poses were created presumably on the robot by running a filter on the IMU data (such estimates are called odometry estimates), and these poses will not be extremely accurate.

You can read more about the hardware in this paper - [THOR-OP humanoid robot for DARPA Robotics Challenge Trials 2013](https://ieeexplore.ieee.org/document/7057369)

## Coordinate frames
