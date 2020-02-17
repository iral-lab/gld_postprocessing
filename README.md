# gld_postprocessing

Converting the ROS bag files to pointclouds.
There is one PCD (Point Cloud Data) file per frame.
 
rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
Where:
    <input_file.bag> is the bag file name to read.
    <topic> is the topic in the bag file containing messages to save. (For GLD this was 'points2')
    <output_directory> is the directory on disk in which to create PCD files from the point cloud messages. 
