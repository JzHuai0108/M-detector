// reader.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <filesystem>
#include <regex>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>

namespace fs = std::filesystem;

// ----------------- utilities -----------------
static double safe_stod(const std::string& s, bool& ok) {
  try { ok = true; return std::stod(s); }
  catch (...) { ok = false; return 0.0; }
}

std::vector<std::string> findAndSortPcFiles(const std::string& folder) {
  std::vector<std::string> pcd_files;
  std::vector<std::string> ply_files;

  for (const auto& entry : fs::directory_iterator(folder)) {
    if (!entry.is_regular_file()) continue;
    auto ext = entry.path().extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".pcd")      pcd_files.push_back(entry.path().string());
    else if (ext == ".ply") ply_files.push_back(entry.path().string());
  }

  // pick the format with the most files (tie -> .pcd)
  auto& pc_files = (pcd_files.size() >= ply_files.size()) ? pcd_files : ply_files;

  // Extract numeric key from stem
  auto extractNumber = [](const std::string &filepath) -> double {
    std::string stem = fs::path(filepath).stem().string();
    // If the stem itself is a number, use it
    bool ok = false;
    double v = safe_stod(stem, ok);
    if (ok) return v;

    // Else, grab the last numeric substring (supports sec.nsec or floats)
    std::regex re(R"((\d+(?:\.\d+)?))");
    std::sregex_iterator it(stem.begin(), stem.end(), re), end;
    if (it == end) return -1.0;
    std::sregex_iterator last = it;
    while (++it != end) last = it;
    return std::stod(last->str());
  };

  std::sort(pc_files.begin(), pc_files.end(),
            [&](const std::string &a, const std::string &b) {
              return extractNumber(a) < extractNumber(b);
            });

  // print a preview
  size_t print_count = std::min(pc_files.size(), static_cast<size_t>(13));
  std::cout << "First " << print_count << " sorted point cloud files:\n";
  for (size_t i = 0; i < print_count; ++i) {
    std::cout << std::setw(5) << i << ": " << pc_files[i] << "\n";
  }
  return pc_files;
}

struct Pose {
  ros::Time stamp;
  Eigen::Vector3d t{0,0,0};
  Eigen::Quaterniond q{1,0,0,0}; // w,x,y,z
};

static inline bool validQuat(const Eigen::Quaterniond& q) {
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z()) && (q.norm() > 1e-6);
}

// Load TUM format: timestamp x y z qx qy qz qw ...
// Returns poses sorted by time (ascending).
std::vector<Pose> loadTUMPoseFile(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    ROS_ERROR_STREAM("Failed to open pose file: " << path);
    return {};
  }

  std::vector<Pose> poses;
  poses.reserve(4096);

  std::string line;
  size_t ok_cnt = 0, bad_cnt = 0;

  while (std::getline(ifs, line)) {
    // trim leading space
    auto it = std::find_if(line.begin(), line.end(),
                           [](unsigned char c){ return !std::isspace(c); });
    if (it == line.end() || *it == '#') continue;

    std::istringstream iss(line);
    std::string ts_s;
    double x, y, z, qx, qy, qz, qw;

    if (!(iss >> ts_s >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      ++bad_cnt; continue;
    }

    // ---- exact sec.nsec parsing ----
    std::string sec_part, nsec_part;
    auto dot_pos = ts_s.find('.');
    if (dot_pos != std::string::npos) {
      sec_part  = ts_s.substr(0, dot_pos);
      nsec_part = ts_s.substr(dot_pos + 1);
    } else {
      sec_part  = ts_s;
      nsec_part.clear();
    }

    // normalize to 9 digits nanoseconds
    if (nsec_part.size() > 9)
      nsec_part = nsec_part.substr(0, 9);
    else if (nsec_part.size() < 9)
      nsec_part.append(9 - nsec_part.size(), '0');

    int32_t sec = 0;
    uint32_t nsec = 0;
    try {
      sec  = static_cast<int32_t>(std::stol(sec_part));
      nsec = static_cast<uint32_t>(std::stoul(nsec_part));
    } catch (...) {
      ++bad_cnt; continue;
    }

    Pose p;
    p.stamp = ros::Time(sec, nsec);
    p.t = Eigen::Vector3d(x, y, z);
    p.q = Eigen::Quaterniond(qw, qx, qy, qz);
    if (!validQuat(p.q)) { ++bad_cnt; continue; }
    p.q.normalize();

    poses.emplace_back(std::move(p));
    ++ok_cnt;
  }

  // sort by time ascending
  std::sort(poses.begin(), poses.end(),
            [](const Pose& a, const Pose& b){ return a.stamp < b.stamp; });

  // dedup very close timestamps
  std::vector<Pose> dedup;
  dedup.reserve(poses.size());
  const double eps = 1e-9;
  for (const auto& p : poses) {
    if (dedup.empty() ||
        std::abs((p.stamp - dedup.back().stamp).toSec()) > eps)
      dedup.emplace_back(p);
  }

  ROS_INFO_STREAM("Loaded poses: " << ok_cnt << " (skipped " << bad_cnt
                                   << ") from " << path
                                   << "; kept " << dedup.size() << " after sort/dedup");
  return dedup;
}


// Try to derive timestamp from filename stem.
// Supports "sec.nsec", "123.456", "scans_000123.456" (last numeric wins).
bool timestampFromFilename(const std::string& file, ros::Time& t_out) {
  const std::string stem = fs::path(file).stem().string();
  // Prefer full "sec.nsec" if two integers separated by '.'
  std::regex sec_nsec_re(R"((\d+)\.(\d{1,9}))");
  std::smatch m;
  if (std::regex_search(stem, m, sec_nsec_re)) {
    // Normalize nsec to 9 digits by padding/truncating
    std::string nsec_str = m[2].str();
    if (nsec_str.size() < 9) nsec_str.append(9 - nsec_str.size(), '0');
    else if (nsec_str.size() > 9) nsec_str = nsec_str.substr(0,9);
    uint32_t nsec = static_cast<uint32_t>(std::stoul(nsec_str));
    uint32_t sec  = static_cast<uint32_t>(std::stoul(m[1].str()));
    t_out.sec = sec;
    t_out.nsec = nsec;
    return true;
  }
  // Else use last floating number
  std::regex float_re(R"((\d+(?:\.\d+)?))");
  std::sregex_iterator it(stem.begin(), stem.end(), float_re), end;
  if (it == end) return false;
  std::sregex_iterator last = it;
  while (++it != end) last = it;
  double ts = std::stod(last->str());
  t_out = ros::Time().fromSec(ts);
  return true;
}

template <typename PointT>
bool loadCloud(const std::string& path, typename pcl::PointCloud<PointT>::Ptr& out) {
  out.reset(new pcl::PointCloud<PointT>());
  auto ext = fs::path(path).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  int ret = -1;
  if (ext == ".pcd") {
    ret = pcl::io::loadPCDFile(path, *out);
  } else if (ext == ".ply") {
    ret = pcl::io::loadPLYFile(path, *out);
  } else {
    ROS_ERROR_STREAM("Unsupported file extension: " << ext);
    return false;
  }
  if (ret < 0) {
    ROS_ERROR_STREAM("Failed to load cloud: " << path);
    return false;
  }
  return true;
}

nav_msgs::Odometry makeOdomMsg(const Pose& pose,
                               const std::string& odom_frame,
                               const std::string& child_frame) {
  nav_msgs::Odometry odom;
  odom.header.stamp = pose.stamp;
  odom.header.frame_id = odom_frame;      // world/map frame
  odom.child_frame_id  = child_frame;     // sensor/base frame

  odom.pose.pose.position.x = pose.t.x();
  odom.pose.pose.position.y = pose.t.y();
  odom.pose.pose.position.z = pose.t.z();
  odom.pose.pose.orientation.w = pose.q.w();
  odom.pose.pose.orientation.x = pose.q.x();
  odom.pose.pose.orientation.y = pose.q.y();
  odom.pose.pose.orientation.z = pose.q.z();

  // leave covariances zero or paramize later if needed
  return odom;
}


static bool hasField(const pcl::PCLPointCloud2& pc2, const std::string& name) {
  for (const auto& f : pc2.fields) if (f.name == name) return true;
  return false;
}

bool loadCloudSmart(const std::string& path,
                    sensor_msgs::PointCloud2& msg_out,
                    std::string& detected_type /* "XYZ"|"XYZI"|"XYZINormal" */)
{
  pcl::PCLPointCloud2 pc2;
  auto ext = std::filesystem::path(path).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

  int ret = -1;
  if (ext == ".pcd")      ret = pcl::io::loadPCDFile(path, pc2);
  else if (ext == ".ply") ret = pcl::io::loadPLYFile(path, pc2);
  else { ROS_ERROR_STREAM("Unsupported ext: " << ext); return false; }

  if (ret < 0) { ROS_ERROR_STREAM("Failed to load: " << path); return false; }

  const bool has_i  = hasField(pc2, "intensity");
  const bool has_nx = hasField(pc2, "normal_x");
  const bool has_ny = hasField(pc2, "normal_y");
  const bool has_nz = hasField(pc2, "normal_z");
  const bool has_normals = has_nx && has_ny && has_nz; // curvature optional

  if (has_normals && has_i) {
    detected_type = "XYZINormal";
    pcl_conversions::fromPCL(pc2, msg_out); // keep as-is; no reinterpretation needed
  } else if (has_i) {
    detected_type = "XYZI";
    // Convert to PointXYZI to be explicit (optional)
    pcl::PointCloud<pcl::PointXYZI> tmp;
    pcl::fromPCLPointCloud2(pc2, tmp);
    pcl::toROSMsg(tmp, msg_out);
  } else {
    detected_type = "XYZ";
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromPCLPointCloud2(pc2, tmp);
    pcl::toROSMsg(tmp, msg_out);
  }
  return true;
}

// ----------------- main -----------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "dynfilter_reader");
  ros::NodeHandle nh;

  // CLI args
  std::string pc_folder, pose_file;
  if (argc >= 3) {
    pc_folder = argv[1];
    pose_file = argv[2];
  } else {
    // also allow parameters if no CLI args
    nh.param<std::string>("pc_folder", pc_folder, std::string(""));
    nh.param<std::string>("pose_file", pose_file, std::string(""));
  }

  // Topics and frames
  std::string points_topic, odom_topic, points_frame, odom_frame, child_frame;
  nh.param<std::string>("dyn_obj/points_topic", points_topic, std::string("/dyn_obj/points"));
  nh.param<std::string>("dyn_obj/odom_topic",   odom_topic,   std::string("/dyn_obj/odom"));
  nh.param<std::string>("points_frame", points_frame, std::string("lidar"));
  nh.param<std::string>("odom_frame",   odom_frame,   std::string("camera_init"));
  nh.param<std::string>("child_frame",  child_frame,  std::string("lidar"));

  std::string registered_topic;
  nh.param<std::string>("registered_topic", registered_topic, std::string("/cloud_registered"));
  ros::Publisher pub_registered = nh.advertise<sensor_msgs::PointCloud2>(registered_topic, 2);

  // Playback controls
  double publish_rate_hz = 5.0;   nh.param("publish_rate_hz", publish_rate_hz, 5.0);
  double match_tolerance = 0.050; nh.param("match_tolerance", match_tolerance, 0.050); // seconds
  bool   use_file_time_as_msg_stamp = true;
  nh.param("use_file_time_as_msg_stamp", use_file_time_as_msg_stamp, true);

  if (pc_folder.empty() || pose_file.empty()) {
    ROS_ERROR("Usage: rosrun <pkg> reader <pc_folder> <pose_file>\n"
              "Or set private params: ~pc_folder, ~pose_file");
    return 1;
  }

  // Load inputs
  const auto files = findAndSortPcFiles(pc_folder);
  if (files.empty()) {
    ROS_ERROR_STREAM("No .pcd/.ply files found under: " << pc_folder);
    return 1;
  }
  auto poses = loadTUMPoseFile(pose_file);
  if (poses.empty()) {
    ROS_ERROR_STREAM("No poses loaded from: " << pose_file);
    return 1;
  }
  if (poses.size() != files.size()) {
    ROS_ERROR_STREAM("Inconsistent poses " << poses.size() << " and pc files " << files.size());
    return 1;
  }

  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(points_topic, 2);
  ros::Publisher pub_odom  = nh.advertise<nav_msgs::Odometry>(odom_topic,  2);

  static tf2_ros::TransformBroadcaster br;
  ros::Rate rate(publish_rate_hz);

  auto waitFor = [&](ros::Publisher& pub, const std::string& name, size_t min_subs=1){
    ros::Rate r(10);
    while (ros::ok() && pub.getNumSubscribers() < min_subs) {
      ROS_INFO_THROTTLE(2.0, "Waiting for subscriber on %s...", name.c_str());
      r.sleep();
      ros::spinOnce();
    }
  };

  std::cout << "Waiting for subscribers ... " << std::endl;
  waitFor(pub_cloud, points_topic, 1);
  waitFor(pub_odom,  odom_topic,   1);
  std::cout << "Playing pc and pose ... " << std::endl;

  size_t fi = 0;
  for (const auto& path : files) {
    // 1) derive timestamp from filename
    ros::Time file_stamp;
    timestampFromFilename(path, file_stamp);
    Pose pose_match = poses.at(fi);
    if (pose_match.stamp != file_stamp) {
      ROS_ERROR_STREAM("Inconsistent pose time " << pose_match.stamp << " and filename " << file_stamp);
      return 1;
    }

    // 3) load cloud (XYZ/I if present)
    sensor_msgs::PointCloud2 cloud_msg;
    std::string dtype;
    if (!loadCloudSmart(path, cloud_msg, dtype)) {
      ROS_WARN_STREAM("Skip unreadable cloud: " << path);
      break;
    }

    cloud_msg.header.stamp = pose_match.stamp;
    cloud_msg.header.frame_id = points_frame;
    pub_cloud.publish(cloud_msg);

    // -----------------------------------------------------------------------------
    // Transform cloud to world frame (map) for /cloud_registered
    // -----------------------------------------------------------------------------
    Eigen::Isometry3d T_w_l = Eigen::Isometry3d::Identity();
    T_w_l.linear() = pose_match.q.toRotationMatrix();
    T_w_l.translation() = pose_match.t;
    Eigen::Matrix4f T_4f = T_w_l.matrix().cast<float>();

    sensor_msgs::PointCloud2 msg_registered;

    // Convert cloud_msg -> typed -> transform -> toROSMsg
    if (dtype == "XYZINormal") {
      pcl::PointCloud<pcl::PointXYZINormal> c, c_reg;
      pcl::fromROSMsg(cloud_msg, c);
      pcl::transformPointCloud(c, c_reg, T_4f);
      pcl::toROSMsg(c_reg, msg_registered);
    }
    else if (dtype == "XYZI") {
      pcl::PointCloud<pcl::PointXYZI> c, c_reg;
      pcl::fromROSMsg(cloud_msg, c);
      pcl::transformPointCloud(c, c_reg, T_4f);
      pcl::toROSMsg(c_reg, msg_registered);
    }
    else { // dtype == "XYZ"
      pcl::PointCloud<pcl::PointXYZ> c, c_reg;
      pcl::fromROSMsg(cloud_msg, c);
      pcl::transformPointCloud(c, c_reg, T_4f);
      pcl::toROSMsg(c_reg, msg_registered);
    }

    msg_registered.header.stamp = pose_match.stamp;
    msg_registered.header.frame_id = odom_frame; // world/map frame
    pub_registered.publish(msg_registered);

    nav_msgs::Odometry odom = makeOdomMsg(pose_match, odom_frame, child_frame);
    odom.header.stamp = pose_match.stamp;
    pub_odom.publish(odom);

    geometry_msgs::TransformStamped T;
    T.header.stamp = pose_match.stamp;
    T.header.frame_id = odom_frame;
    T.child_frame_id  = child_frame;
    T.transform.translation.x = pose_match.t.x();
    T.transform.translation.y = pose_match.t.y();
    T.transform.translation.z = pose_match.t.z();
    T.transform.rotation.w = pose_match.q.w();
    T.transform.rotation.x = pose_match.q.x();
    T.transform.rotation.y = pose_match.q.y();
    T.transform.rotation.z = pose_match.q.z();
    br.sendTransform(T);

    ros::spinOnce();
    rate.sleep();
    ++fi;
    if (!ros::ok()) break;
  }

  ROS_INFO_STREAM("Publishing finished. Sent: " << fi);
  return 0;
}
