#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()

    outlier_filter.set_mean_k(20)
    outlier_filter.set_std_dev_mul_thresh(0.2)
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()

    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    # Filter out objects on both sides of the table
    passthrough_y = cloud_filtered.make_passthrough_filter()

    filter_axis = 'y'
    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -0.45
    axis_max = 0.45
    passthrough_y.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough_y.filter()

    # Filter out objects beneath the desktop
    passthrough = cloud_filtered.make_passthrough_filter()

    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.3
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()

    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers       
    inliers, coefficients = seg.segment()

    extracted_inliers_table = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers_objects = cloud_filtered.extract(inliers, negative=True)
    
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_outliers_objects)
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(1500)

    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Convert PCL data to ROS messages
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters,each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(extracted_inliers_table)
    ros_cloud_objects = pcl_to_ros(extracted_outliers_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster_pub.publish(ros_cluster_cloud)


# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = extracted_outliers_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extrct histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)

        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)

        # Compute the associated feature vector
        # Concatenate features
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
    
    rospy.loginfo('Detected {} object: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()

    # Obtain rarget label
    object_list_param = rospy.get_param('/object_list')
    target_label = []
    for object in object_list_param:
        target_label.append(object['name'])
    
    # Check if recognition works
    flag = 1
    for label in target_label:
        if label in detected_objects_labels:
            continue
        else:
            flag = 0
            break
    
    # PR2 should execute when recognition works
    if flag == 1:
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass
    else:
        print("Wrong Recognition")
    

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    output_list = [] # list of dictionaries containing all ROS service request messages

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Check if input object list corresponds to current scenery
    if not len(object_list) == len(object_list_param):
        rospy.loginfo("List of detected objects does not match pick list.")
        return

    # Define test case
    test_scene_num = Int32()

    scene_dict = {'3':1, '5':2, '8':3}  # For extracting test_scene_num data
    test_scene_num.data = scene_dict[str(len(object_list_param))]

    # TODO: Parse parameters into individual variables
    
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for i in range(len(object_list_param)):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
        
        ros_object_name = String()
        ros_object_name.data = object_name
        
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        flag_recog = 0
        for object_recog in object_list:
            if object_recog.label == object_name:
                points_arr = ros_to_pcl(object_recog.cloud).to_array()
                object_list.remove(object_recog)
                flag_recog = 1
                break

        if flag_recog == 0:
            print("Wrong Recognition of %s"%(object_name))
            return

        centroid_np = np.mean(points_arr, axis=0)[:3]
        centroid = [np.asscalar(x) for x in centroid_np]
        # Assign pick pose
        pick_pose = Pose()
        pick_pose.position.x = centroid[0]
        pick_pose.position.y = centroid[1]
        pick_pose.position.z = centroid[2]

        # TODO: Create 'place_pose' for the object
        dropbox_dict = {'red': 0, 'green': 1}
        dropbox_id = dropbox_dict[object_group]
        dropbox_pos = dropbox_param[dropbox_id]['position']

        place_pose = Pose()
        place_pose.position.x = dropbox_pos[0]
        place_pose.position.y = dropbox_pos[1]
        place_pose.position.z = dropbox_pos[2]

        # TODO: Assign the arm to be used for pick_place
        arm_name = String()
        arm_name.data = dropbox_param[dropbox_id]['name']

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        object_yaml = make_yaml_dict(test_scene_num, arm_name, ros_object_name, pick_pose, place_pose)
        output_list.append(object_yaml)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, ros_object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    yaml_filename = 'output_' + str(test_scene_num.data) + '.yaml'
    send_to_yaml(yaml_filename, output_list)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)


    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
