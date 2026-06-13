#ifndef DYN_OBJ_FILTER_ROS_H
#define DYN_OBJ_FILTER_ROS_H

#include <m_detector/DynObjFilter.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>

DynObjConfig loadDynObjConfigFromRos(ros::NodeHandle& nh);

void publishDynObjFilterClouds(DynObjFilter& filter,
                               const ros::Publisher& pub_point_out,
                               const ros::Publisher& pub_frame_out,
                               const ros::Publisher& pub_steady_points,
                               const ros::Time& scan_end_time);

#endif
