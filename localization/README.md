# Localization Package

##  Launch

### kalman_localization

    ros2 launch localization kalman_localization.launch.py

This launch file will run localization.kalman_localization with kalman_localization.yaml

This package subscribe GPS(NavSatFix), Encoder(SerialFeedBack), Imu(Imu) and then publish /odometry/kalman(Odometry)

#### Params

1. topic

    * topic name of odometry

2. is_publish_tf

    * determine whether publish tf message between "frame_id" and "child_frame_id"

3. frame_id & child_frame_id

    * odometry and tf header

4. gps

    1. topic

        * topic name of gps

    2. use_covariance

        * determine use covariance from gps sensor(false) or fixed covariance(true, gps_covariance)

5. encoder & imu

    * same as gps


### map_odom_tf_publisher

    ros2 launch localization map_odom_tf_publisher.launch.py

This launch file will run localization.map_odom_tf_publisher with map_odom_tf_publisher.yaml

This package subscribe pcl_pose(PoseWithCovarianceStamped), and /odometry/kalman(Odometry)

#### Params

1. map_topic & odom_topic

    * topic name of pcl_pose(map_frame) and odometry/kalman(odom_frame)
    * Datatype is unchangeable.

2. Threshold Values

    * Threshold Values to decide update TF
    * If all values are under threshold, tf will updated
    * All Parameters must be double type