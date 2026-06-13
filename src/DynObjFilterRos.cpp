#include <m_detector/DynObjFilterRos.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

DynObjConfig loadDynObjConfigFromRos(ros::NodeHandle& nh)
{
    DynObjConfig c;

    nh.param<double>("dyn_obj/buffer_delay", c.buffer_delay, c.buffer_delay);
    nh.param<int>("dyn_obj/buffer_size", c.buffer_size, c.buffer_size);
    nh.param<int>("dyn_obj/points_num_perframe", c.points_num_perframe, c.points_num_perframe);
    nh.param<double>("dyn_obj/depth_map_dur", c.depth_map_dur, c.depth_map_dur);
    nh.param<int>("dyn_obj/max_depth_map_num", c.max_depth_map_num, c.max_depth_map_num);
    nh.param<int>("dyn_obj/max_pixel_points", c.max_pixel_points, c.max_pixel_points);
    nh.param<double>("dyn_obj/frame_dur", c.frame_dur, c.frame_dur);
    nh.param<int>("dyn_obj/dataset", c.dataset, c.dataset);

    nh.param<float>("dyn_obj/self_x_f", c.self_x_f, c.self_x_f);
    nh.param<float>("dyn_obj/self_x_b", c.self_x_b, c.self_x_b);
    nh.param<float>("dyn_obj/self_y_l", c.self_y_l, c.self_y_l);
    nh.param<float>("dyn_obj/self_y_r", c.self_y_r, c.self_y_r);
    nh.param<float>("dyn_obj/blind_dis", c.blind_dis, c.blind_dis);

    nh.param<float>("dyn_obj/fov_up", c.fov_up, c.fov_up);
    nh.param<float>("dyn_obj/fov_down", c.fov_down, c.fov_down);
    nh.param<float>("dyn_obj/fov_cut", c.fov_cut, c.fov_cut);
    nh.param<float>("dyn_obj/fov_left", c.fov_left, c.fov_left);
    nh.param<float>("dyn_obj/fov_right", c.fov_right, c.fov_right);

    nh.param<int>("dyn_obj/checkneighbor_range", c.checkneighbor_range, c.checkneighbor_range);
    nh.param<bool>("dyn_obj/stop_object_detect", c.stop_object_detect, c.stop_object_detect);

    nh.param<float>("dyn_obj/depth_thr1", c.depth_thr1, c.depth_thr1);
    nh.param<float>("dyn_obj/enter_min_thr1", c.enter_min_thr1, c.enter_min_thr1);
    nh.param<float>("dyn_obj/enter_max_thr1", c.enter_max_thr1, c.enter_max_thr1);

    nh.param<float>("dyn_obj/map_cons_depth_thr1", c.map_cons_depth_thr1, c.map_cons_depth_thr1);
    nh.param<float>("dyn_obj/map_cons_hor_thr1", c.map_cons_hor_thr1, c.map_cons_hor_thr1);
    nh.param<float>("dyn_obj/map_cons_ver_thr1", c.map_cons_ver_thr1, c.map_cons_ver_thr1);
    nh.param<float>("dyn_obj/map_cons_hor_dis1", c.map_cons_hor_dis1, c.map_cons_hor_dis1);
    nh.param<float>("dyn_obj/map_cons_ver_dis1", c.map_cons_ver_dis1, c.map_cons_ver_dis1);

    nh.param<float>("dyn_obj/depth_cons_depth_thr1", c.depth_cons_depth_thr1, c.depth_cons_depth_thr1);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr1", c.depth_cons_depth_max_thr1, c.depth_cons_depth_max_thr1);
    nh.param<float>("dyn_obj/depth_cons_hor_thr1", c.depth_cons_hor_thr1, c.depth_cons_hor_thr1);
    nh.param<float>("dyn_obj/depth_cons_ver_thr1", c.depth_cons_ver_thr1, c.depth_cons_ver_thr1);

    nh.param<float>("dyn_obj/enlarge_z_thr1", c.enlarge_z_thr1, c.enlarge_z_thr1);
    nh.param<float>("dyn_obj/enlarge_angle", c.enlarge_angle, c.enlarge_angle);
    nh.param<float>("dyn_obj/enlarge_depth", c.enlarge_depth, c.enlarge_depth);

    nh.param<int>("dyn_obj/occluded_map_thr1", c.occluded_map_thr1, c.occluded_map_thr1);
    nh.param<bool>("dyn_obj/case1_interp_en", c.case1_interp_en, c.case1_interp_en);

    nh.param<float>("dyn_obj/k_depth_min_thr1", c.k_depth_min_thr1, c.k_depth_min_thr1);
    nh.param<float>("dyn_obj/d_depth_min_thr1", c.d_depth_min_thr1, c.d_depth_min_thr1);
    nh.param<float>("dyn_obj/k_depth_max_thr1", c.k_depth_max_thr1, c.k_depth_max_thr1);
    nh.param<float>("dyn_obj/d_depth_max_thr1", c.d_depth_max_thr1, c.d_depth_max_thr1);

    nh.param<float>("dyn_obj/v_min_thr2", c.v_min_thr2, c.v_min_thr2);
    nh.param<float>("dyn_obj/acc_thr2", c.acc_thr2, c.acc_thr2);

    nh.param<float>("dyn_obj/map_cons_depth_thr2", c.map_cons_depth_thr2, c.map_cons_depth_thr2);
    nh.param<float>("dyn_obj/map_cons_hor_thr2", c.map_cons_hor_thr2, c.map_cons_hor_thr2);
    nh.param<float>("dyn_obj/map_cons_ver_thr2", c.map_cons_ver_thr2, c.map_cons_ver_thr2);

    nh.param<float>("dyn_obj/occ_depth_thr2", c.occ_depth_thr2, c.occ_depth_thr2);
    nh.param<float>("dyn_obj/occ_hor_thr2", c.occ_hor_thr2, c.occ_hor_thr2);
    nh.param<float>("dyn_obj/occ_ver_thr2", c.occ_ver_thr2, c.occ_ver_thr2);

    nh.param<float>("dyn_obj/depth_cons_depth_thr2", c.depth_cons_depth_thr2, c.depth_cons_depth_thr2);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr2", c.depth_cons_depth_max_thr2, c.depth_cons_depth_max_thr2);
    nh.param<float>("dyn_obj/depth_cons_hor_thr2", c.depth_cons_hor_thr2, c.depth_cons_hor_thr2);
    nh.param<float>("dyn_obj/depth_cons_ver_thr2", c.depth_cons_ver_thr2, c.depth_cons_ver_thr2);

    nh.param<float>("dyn_obj/k_depth2", c.k_depth2, c.k_depth2);
    nh.param<int>("dyn_obj/occluded_times_thr2", c.occluded_times_thr2, c.occluded_times_thr2);
    nh.param<bool>("dyn_obj/case2_interp_en", c.case2_interp_en, c.case2_interp_en);
    nh.param<float>("dyn_obj/k_depth_max_thr2", c.k_depth_max_thr2, c.k_depth_max_thr2);
    nh.param<float>("dyn_obj/d_depth_max_thr2", c.d_depth_max_thr2, c.d_depth_max_thr2);

    nh.param<float>("dyn_obj/v_min_thr3", c.v_min_thr3, c.v_min_thr3);
    nh.param<float>("dyn_obj/acc_thr3", c.acc_thr3, c.acc_thr3);

    nh.param<float>("dyn_obj/map_cons_depth_thr3", c.map_cons_depth_thr3, c.map_cons_depth_thr3);
    nh.param<float>("dyn_obj/map_cons_hor_thr3", c.map_cons_hor_thr3, c.map_cons_hor_thr3);
    nh.param<float>("dyn_obj/map_cons_ver_thr3", c.map_cons_ver_thr3, c.map_cons_ver_thr3);

    nh.param<float>("dyn_obj/occ_depth_thr3", c.occ_depth_thr3, c.occ_depth_thr3);
    nh.param<float>("dyn_obj/occ_hor_thr3", c.occ_hor_thr3, c.occ_hor_thr3);
    nh.param<float>("dyn_obj/occ_ver_thr3", c.occ_ver_thr3, c.occ_ver_thr3);

    nh.param<float>("dyn_obj/depth_cons_depth_thr3", c.depth_cons_depth_thr3, c.depth_cons_depth_thr3);
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr3", c.depth_cons_depth_max_thr3, c.depth_cons_depth_max_thr3);
    nh.param<float>("dyn_obj/depth_cons_hor_thr3", c.depth_cons_hor_thr3, c.depth_cons_hor_thr3);
    nh.param<float>("dyn_obj/depth_cons_ver_thr3", c.depth_cons_ver_thr3, c.depth_cons_ver_thr3);

    nh.param<float>("dyn_obj/k_depth3", c.k_depth3, c.k_depth3);
    nh.param<int>("dyn_obj/occluding_times_thr3", c.occluding_times_thr3, c.occluding_times_thr3);
    nh.param<bool>("dyn_obj/case3_interp_en", c.case3_interp_en, c.case3_interp_en);
    nh.param<float>("dyn_obj/k_depth_max_thr3", c.k_depth_max_thr3, c.k_depth_max_thr3);
    nh.param<float>("dyn_obj/d_depth_max_thr3", c.d_depth_max_thr3, c.d_depth_max_thr3);

    nh.param<float>("dyn_obj/interp_hor_thr", c.interp_hor_thr, c.interp_hor_thr);
    nh.param<float>("dyn_obj/interp_ver_thr", c.interp_ver_thr, c.interp_ver_thr);
    nh.param<float>("dyn_obj/interp_thr1", c.interp_thr1, c.interp_thr1);
    nh.param<float>("dyn_obj/interp_static_max", c.interp_static_max, c.interp_static_max);
    nh.param<float>("dyn_obj/interp_start_depth1", c.interp_start_depth1, c.interp_start_depth1);
    nh.param<float>("dyn_obj/interp_kp1", c.interp_kp1, c.interp_kp1);
    nh.param<float>("dyn_obj/interp_kd1", c.interp_kd1, c.interp_kd1);
    nh.param<float>("dyn_obj/interp_thr2", c.interp_thr2, c.interp_thr2);
    nh.param<float>("dyn_obj/interp_thr3", c.interp_thr3, c.interp_thr3);

    nh.param<bool>("dyn_obj/dyn_filter_en", c.dyn_filter_en, c.dyn_filter_en);
    nh.param<bool>("dyn_obj/debug_publish", c.debug_publish, c.debug_publish);

    nh.param<int>("dyn_obj/laserCloudSteadObj_accu_limit", c.laserCloudSteadObj_accu_limit, c.laserCloudSteadObj_accu_limit);
    nh.param<float>("dyn_obj/voxel_filter_size", c.voxel_filter_size, c.voxel_filter_size);

    nh.param<bool>("dyn_obj/cluster_coupled", c.cluster_coupled, c.cluster_coupled);
    nh.param<bool>("dyn_obj/cluster_future", c.cluster_future, c.cluster_future);
    nh.param<int>("dyn_obj/cluster_extend_pixel", c.cluster_extend_pixel, c.cluster_extend_pixel);
    nh.param<int>("dyn_obj/cluster_min_pixel_number", c.cluster_min_pixel_number, c.cluster_min_pixel_number);
    nh.param<float>("dyn_obj/cluster_thrustable_thresold", c.cluster_thrustable_thresold, c.cluster_thrustable_thresold);
    nh.param<float>("dyn_obj/cluster_Voxel_revolusion", c.cluster_Voxel_revolusion, c.cluster_Voxel_revolusion);
    nh.param<bool>("dyn_obj/cluster_debug_en", c.cluster_debug_en, c.cluster_debug_en);
    nh.param<std::string>("dyn_obj/cluster_out_file", c.cluster_out_file, c.cluster_out_file);

    nh.param<float>("dyn_obj/ver_resolution_max", c.ver_resolution_max, c.ver_resolution_max);
    nh.param<float>("dyn_obj/hor_resolution_max", c.hor_resolution_max, c.hor_resolution_max);

    nh.param<float>("dyn_obj/buffer_dur", c.buffer_dur, c.buffer_dur);
    nh.param<int>("dyn_obj/point_index", c.point_index, c.point_index);
    nh.param<std::string>("dyn_obj/frame_id", c.frame_id, c.frame_id);
    nh.param<std::string>("dyn_obj/time_file", c.time_file, c.time_file);
    nh.param<std::string>("dyn_obj/time_breakdown_file", c.time_breakdown_file, c.time_breakdown_file);

    return c;
}

void publishDynObjFilterClouds(DynObjFilter& filter,
                               const ros::Publisher& pub_point_out,
                               const ros::Publisher& pub_frame_out,
                               const ros::Publisher& pub_steady_points,
                               const ros::Time& scan_end_time)
{
    if(filter.cluster_coupled) {
        std::cout << "Found Dynamic Objects, numbers: " << filter.laserCloudDynObj_clus->points.size()
                  << " Total time: " << filter.time_total
                  << " Average total time: " << filter.time_total_avr << std::endl;
    } else {
        std::cout << "Found Dynamic Objects, numbers: " << filter.laserCloudDynObj->points.size()
                  << " Total time: " << filter.time_total
                  << " Average total time: " << filter.time_total_avr << std::endl;
    }
    filter.case1_num = 0;
    filter.case2_num = 0;
    filter.case3_num = 0;

    sensor_msgs::PointCloud2 dynamic_msg;
    pcl::toROSMsg(*filter.laserCloudDynObj_world, dynamic_msg);
    dynamic_msg.header.stamp = scan_end_time;
    dynamic_msg.header.frame_id = filter.frame_id;
    pub_point_out.publish(dynamic_msg);

    if(filter.cluster_coupled || filter.cluster_future) {
        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*filter.laserCloudDynObj_clus, cluster_msg);
        cluster_msg.header.stamp = scan_end_time;
        cluster_msg.header.frame_id = filter.frame_id;
        pub_frame_out.publish(cluster_msg);
    }

    sensor_msgs::PointCloud2 static_msg;
    PointCloudXYZI::Ptr static_cloud(new PointCloudXYZI);
    if(filter.cluster_coupled) {
        if(filter.laserCloudSteadObj_accu_times < filter.laserCloudSteadObj_accu_limit) {
            filter.laserCloudSteadObj_accu_times++;
            filter.laserCloudSteadObj_accu.push_back(filter.laserCloudSteadObj_clus);
        } else {
            filter.laserCloudSteadObj_accu.pop_front();
            filter.laserCloudSteadObj_accu.push_back(filter.laserCloudSteadObj_clus);
        }
        for(const auto& cloud : filter.laserCloudSteadObj_accu) {
            *static_cloud += *cloud;
        }
        pcl::VoxelGrid<PointType> down_size_filter;
        down_size_filter.setLeafSize(filter.voxel_filter_size, filter.voxel_filter_size, filter.voxel_filter_size);
        down_size_filter.setInputCloud(static_cloud);
        PointCloudXYZI static_down;
        down_size_filter.filter(static_down);
        pcl::toROSMsg(static_down, static_msg);
    } else {
        std::cout << "Found Steady Objects, numbers: " << filter.laserCloudSteadObj->points.size() << std::endl;
        if(filter.laserCloudSteadObj_accu_times < filter.laserCloudSteadObj_accu_limit) {
            filter.laserCloudSteadObj_accu_times++;
            filter.laserCloudSteadObj_accu.push_back(filter.laserCloudSteadObj);
        } else {
            filter.laserCloudSteadObj_accu.pop_front();
            filter.laserCloudSteadObj_accu.push_back(filter.laserCloudSteadObj);
        }
        for(const auto& cloud : filter.laserCloudSteadObj_accu) {
            *static_cloud += *cloud;
        }
        pcl::toROSMsg(*static_cloud, static_msg);
    }
    static_msg.header.stamp = scan_end_time;
    static_msg.header.frame_id = filter.frame_id;
    pub_steady_points.publish(static_msg);
}
