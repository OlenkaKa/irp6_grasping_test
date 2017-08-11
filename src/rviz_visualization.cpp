//
// Created by akarbarc on 09.08.17.
//

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <irp6_grasping/object_maker_publisher.h>
#include <irp6_grasping/transform_util.h>

std::string world_frame_id = "/tl_base";
irp6_grasping::ObjectMarkerPublisher *publisher;

void callback(const object_recognition_msgs::RecognizedObject &sensor_object) {
    static tf::TransformListener tf_listener;
    static tf::StampedTransform world_sensor_tf;
    static tf::TransformBroadcaster br;

    try {
        tf_listener.lookupTransform(world_frame_id, sensor_object.pose.header.frame_id,
                /*sensor_object.pose.header.stamp*/ ros::Time(0), world_sensor_tf);

        geometry_msgs::Pose sensor_object_world_pose = irp6_grasping::transformPose(sensor_object.pose.pose.pose, world_sensor_tf);
        object_recognition_msgs::RecognizedObject msg(sensor_object);
        msg.pose.pose.pose = sensor_object_world_pose;
        msg.header.frame_id = world_frame_id;

        tf::Transform transform;
        tf::poseMsgToTF(msg.pose.pose.pose, transform);

        publisher->deleteMarkers();
        tf::StampedTransform stamped_transform(transform, msg.pose.header.stamp,
                                               msg.header.frame_id, msg.type.key);
        br.sendTransform(stamped_transform);

        publisher->createMarkers(msg);
    }
    catch (tf::TransformException &e) {
        ROS_ERROR("%s %s", e.what(), world_frame_id.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_visualization_test");
    ros::NodeHandle nh;

    publisher = new irp6_grasping::ObjectMarkerPublisher(nh, world_frame_id);
    ros::Subscriber recognized_object_sub = nh.subscribe("/recognized_objects", 1000, &callback);
    ros::spin();

    delete publisher;
    return 0;
}
