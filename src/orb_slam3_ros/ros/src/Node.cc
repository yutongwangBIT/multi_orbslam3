#include "Node.h"

#include <iostream>

Node::Node (ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) :  image_transport_(image_transport) {
  name_of_node_ = ros::this_node::getName();
  node_handle_ = node_handle;
  min_observations_per_point_ = 2;
  sensor_ = sensor;
  std::cout<<"node started"<<std::endl;
}


Node::~Node () {
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  //orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}

void Node::Init () {
  //static parameters
  node_handle_.param(name_of_node_+ "/publish_pointcloud", publish_pointcloud_param_, true);
  node_handle_.param(name_of_node_+ "/publish_pose", publish_pose_param_, true);
  node_handle_.param(name_of_node_+ "/publish_tf", publish_tf_param_, true);
  node_handle_.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
  node_handle_.param<std::string>(name_of_node_+ "/camera_frame_id", camera_frame_id_param_, "camera_link");
  node_handle_.param<std::string>(name_of_node_ + "/map_file", map_file_name_param_, "map.bin");
  node_handle_.param<std::string>(name_of_node_ + "/voc_file", voc_file_name_param_, "file_not_set");
  node_handle_.param(name_of_node_ + "/load_map", load_map_param_, false);

   // Create a parameters object to pass to the Tracking system
   ORB_SLAM3::ORBParameters parameters;
   LoadOrbParameters (parameters);

  orb_slam_ = new ORB_SLAM3::System (voc_file_name_param_, parameters, sensor_, false);

 // service_server_ = node_handle_.advertiseService(name_of_node_+"/save_map", &Node::SaveMapSrv, this);

  //Setup dynamic reconfigure
  //dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
  //dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1, _2);
  //dynamic_param_server_.setCallback(dynamic_param_callback);
  rendered_image_publisher_ = image_transport_.advertise (name_of_node_+"/debug_image", 1);
  if (publish_pointcloud_param_) {
    map_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points", 1);
    atlas_points_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2> (name_of_node_+"/map_points_atlas", 1);
    atlas_maps_publisher_=node_handle_.advertise<visualization_msgs::MarkerArray> (name_of_node_+"/atlas", 1);
    keyframes_publisher_ = node_handle_.advertise<orb_slam3_ros::KFs> (name_of_node_+"/keyframes", 10);
    //KeyFramesFromMsg_pub = node_handle_.advertise<visualization_msgs::MarkerArray> (name_of_node_+"/test", 1);
  }
  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> (name_of_node_+"/pose", 1);
  }
}


void Node::Update () {
  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    PublishPositionAsTransform (position);

    if (publish_pose_param_) {
      PublishPositionAsPoseStamped (position);
    }
  }

  /*PublishKeyFrames(orb_slam_->GetAllKeyFrames());*/
  PublishKeyFrames(orb_slam_->GetKeyFramesBuffer());

  //PublishRenderedImage (orb_slam_->DrawCurrentFrame());

  if (publish_pointcloud_param_) {
    PublishMapPoints (orb_slam_->GetAllMapPoints());
    PublishAllMaps(orb_slam_->GetAllMaps());
  }

}

void Node::PublishKeyFrames(orb_slam3_ros::KFs mKFs){
  keyframes_publisher_.publish(mKFs);

  //KEYFRAMES
    /*visualization_msgs::Marker tmp_marker_kf;
    tmp_marker_kf.header.frame_id = map_frame_id_param_;
    tmp_marker_kf.header.stamp = current_frame_time_;
    tmp_marker_kf.ns = "marker_kf";
    tmp_marker_kf.type = visualization_msgs::Marker::ARROW;
    tmp_marker_kf.type = visualization_msgs::Marker::ADD;
    tmp_marker_kf.color.r = 0;
    tmp_marker_kf.color.g = 255;
    tmp_marker_kf.color.b = 0;
    tmp_marker_kf.color.a = 1;
    tmp_marker_kf.scale.x = 0.05;
    tmp_marker_kf.scale.y = 0.01;
    tmp_marker_kf.scale.z = 0.01;
      for(size_t k=0; k<mKFs.KFs.size(); ++k){
        orb_slam3_ros::KF mkf = mKFs.KFs[k];
        cv::Mat Twc = cv::Mat(4, 4, 5);
        ORB_SLAM3::Converter::MsgArrayFixedSizeToCvMat<orb_slam3_ros::KF::_mTwc_type, float>(Twc, mkf.mTwc);
        g2o::Sim3 g2oTwc(ORB_SLAM3::Converter::toMatrix3d(Twc.rowRange(0,3).colRange(0,3)), ORB_SLAM3::Converter::toVector3d(Twc.rowRange(0,3).col(3)),1.0);
        tmp_marker_kf.pose.position.x = g2oTwc.translation().z();
        tmp_marker_kf.pose.position.y = -1.0*g2oTwc.translation().x();
        tmp_marker_kf.pose.position.z = -1.0*g2oTwc.translation().y();
        tmp_marker_kf.pose.orientation.x = g2oTwc.rotation().x();
        tmp_marker_kf.pose.orientation.y = g2oTwc.rotation().y();
        tmp_marker_kf.pose.orientation.z = g2oTwc.rotation().z();
        tmp_marker_kf.pose.orientation.w = g2oTwc.rotation().w();
        tmp_marker_kf.id = k;
        KeyFramesFromMsg.markers.push_back(tmp_marker_kf);
      }
  KeyFramesFromMsg_pub.publish(KeyFramesFromMsg); */
}
/*void Node::PublishKeyFrames(std::vector<ORB_SLAM3::KeyFrame*> KFs){
  if(!KFs.empty()){
    ORB_SLAM3::KeyFrame* currentKF = KFs[KFs.size()-1];
    orb_slam3_ros::KF kf_msg;
    kf_msg.mnId = currentKF->mnId;
    //kf_msg.dTimestamp = current_frame_time_;
    kf_msg.mnGridCols = currentKF->mnGridCols;
    kf_msg.mnGridRows = currentKF->mnGridRows;
    kf_msg.mfGridElementWidthInv = currentKF->mfGridElementWidthInv;
    kf_msg.mfGridElementHeightInv = currentKF->mfGridElementHeightInv;
    kf_msg.fx = currentKF->fx;
    kf_msg.fy = currentKF->fy;
    kf_msg.cx = currentKF->cx;
    kf_msg.cy = currentKF->cy;
    kf_msg.invfx = currentKF->invfx;
    kf_msg.invfy = currentKF->invfy;
    kf_msg.N = static_cast<int16_t>(currentKF->N);
    kf_msg.mnScaleLevels = static_cast<int8_t>(currentKF->mnScaleLevels);
    kf_msg.mfScaleFactor = currentKF->mfScaleFactor;
    kf_msg.mfLogScaleFactor = currentKF->mfLogScaleFactor;
    for(int idx=0;idx<currentKF->mvScaleFactors.size();++idx) kf_msg.mvScaleFactors[idx]=currentKF->mvScaleFactors[idx];
    for(int idx=0;idx<currentKF->mvLevelSigma2.size();++idx) kf_msg.mvLevelSigma2[idx]=currentKF->mvLevelSigma2[idx];
    for(int idx=0;idx<currentKF->mvInvLevelSigma2.size();++idx) kf_msg.mvInvLevelSigma2[idx]=currentKF->mvInvLevelSigma2[idx];
    kf_msg.mnMinX = static_cast<int16_t>(currentKF->mnMinX);
    kf_msg.mnMinY = static_cast<int16_t>(currentKF->mnMinY);
    kf_msg.mnMaxX = static_cast<int16_t>(currentKF->mnMaxX);
    kf_msg.mnMaxY = static_cast<int16_t>(currentKF->mnMaxY);
    keyframes_publisher_.publish(kf_msg);
  }
}*/
void Node::PublishMapPoints (std::vector<ORB_SLAM3::MapPoint*> map_points) {
  sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
  map_points_publisher_.publish (cloud);
}


void Node::PublishPositionAsTransform (cv::Mat position) {
  if(publish_tf_param_){
      tf::Transform transform = TransformFromMat (position);
      static tf::TransformBroadcaster tf_broadcaster;
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, current_frame_time_, map_frame_id_param_, camera_frame_id_param_));
  }
}

void Node::PublishPositionAsPoseStamped (cv::Mat position) {
  tf::Transform grasp_tf = TransformFromMat (position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, current_frame_time_, map_frame_id_param_);
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
  pose_publisher_.publish(pose_msg);
}


void Node::PublishRenderedImage (cv::Mat image) {
  std_msgs::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}


tf::Transform Node::TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);


  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}

void Node::PublishAllMaps(std::vector<ORB_SLAM3::Map*> vmaps){
  visualization_msgs::MarkerArray MapPoints, KeyFrames;
  int count = 0;
  for(std::vector<ORB_SLAM3::Map*>::iterator itmap=vmaps.begin(); itmap!=vmaps.end(); ++itmap){
    //MAP POINTS
    visualization_msgs::Marker tmp_marker_mp;
    tmp_marker_mp.header.frame_id = map_frame_id_param_;
    tmp_marker_mp.header.stamp = current_frame_time_;
    tmp_marker_mp.ns = "marker_map";
    tmp_marker_mp.id = count;
    tmp_marker_mp.color.r = mfFrameColors[count][0];
    tmp_marker_mp.color.g = mfFrameColors[count][1];
    tmp_marker_mp.color.b = mfFrameColors[count][2];
    tmp_marker_mp.color.a = 1;
    tmp_marker_mp.scale.x = 0.01;
    tmp_marker_mp.scale.y = 0.01;
    tmp_marker_mp.type = visualization_msgs::Marker::POINTS;
    tmp_marker_mp.action = visualization_msgs::Marker::ADD;
    std::vector<ORB_SLAM3::MapPoint*> tmp_map_points = (*itmap)->GetAllMapPoints();
    if(!tmp_map_points.empty()){
      for(size_t i=0; i<tmp_map_points.size(); ++i){
        geometry_msgs::Point p;
        p.x = tmp_map_points.at(i)->GetWorldPos().at<float> (2);
        p.y = -1.0*tmp_map_points.at(i)->GetWorldPos().at<float> (0);
        p.z = -1.0*tmp_map_points.at(i)->GetWorldPos().at<float> (1);
        tmp_marker_mp.points.push_back(p);
      }
      MapPoints.markers.push_back(tmp_marker_mp);
    }
    //KEYFRAMES
    visualization_msgs::Marker tmp_marker_kf;
    tmp_marker_kf.header.frame_id = map_frame_id_param_;
    tmp_marker_kf.header.stamp = current_frame_time_;
    tmp_marker_kf.ns = "marker_kf";
    tmp_marker_kf.type = visualization_msgs::Marker::ARROW;
    tmp_marker_kf.type = visualization_msgs::Marker::ADD;
    /*tmp_marker_kf.id = count;*/
    tmp_marker_kf.color.r = mfFrameColors[count][0];
    tmp_marker_kf.color.g = mfFrameColors[count][1];
    tmp_marker_kf.color.b = mfFrameColors[count][2];
    tmp_marker_kf.color.a = 0.7;
    tmp_marker_kf.scale.x = 0.05;
    tmp_marker_kf.scale.y = 0.01;
    tmp_marker_kf.scale.z = 0.01;
    //tmp_marker_kf.pose.orientation.w = 1.0;
    std::vector<ORB_SLAM3::KeyFrame*> tmp_keyframes = (*itmap)->GetAllKeyFrames();
    if(!tmp_keyframes.empty()){
      for(size_t k=0; k<tmp_keyframes.size(); ++k){
        cv::Mat Twc = tmp_keyframes.at(k)->GetPoseInverse();
        g2o::Sim3 g2oTwc(ORB_SLAM3::Converter::toMatrix3d(Twc.rowRange(0,3).colRange(0,3)), ORB_SLAM3::Converter::toVector3d(Twc.rowRange(0,3).col(3)),1.0);
        /*geometry_msgs::Point po;
        po.x = g2oTwc.translation().x();
        po.y = g2oTwc.translation().y();
        po.z = g2oTwc.translation().z();
        po.orientation.x = g2oTwc.rotation().x();
        po.orientation.y = g2oTwc.rotation().y();
        po.orientation.z = g2oTwc.rotation().z();
        po.orientation.w = g2oTwc.rotation().w();
        tmp_marker_kf.points.push_back(po);*/
        tmp_marker_kf.pose.position.x = g2oTwc.translation().z();
        tmp_marker_kf.pose.position.y = -1.0*g2oTwc.translation().x();
        tmp_marker_kf.pose.position.z = -1.0*g2oTwc.translation().y();
        tmp_marker_kf.pose.orientation.x = g2oTwc.rotation().x();
        tmp_marker_kf.pose.orientation.y = g2oTwc.rotation().y();
        tmp_marker_kf.pose.orientation.z = g2oTwc.rotation().z();
        tmp_marker_kf.pose.orientation.w = g2oTwc.rotation().w();
        tmp_marker_kf.id = k;
        KeyFrames.markers.push_back(tmp_marker_kf);
      }
    }
    count++;
  }
  atlas_maps_publisher_.publish(MapPoints);
  atlas_maps_publisher_.publish(KeyFrames);
}

/*void Node::PublishAllMaps(std::vector<ORB_SLAM3::Map*> vmaps){
  std::cout<<"size:"<<sizeof(vmaps)<<std::endl;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_all;
  size_t num_maps = vmaps.size();
  size_t max_maps_to_vis = num_maps-1;
  int count = 0;
  if(num_maps < 1){
    return;
  }
  else if(num_maps>3){  
    max_maps_to_vis = 3;
  }
 //or(std::vector<ORB_SLAM3::Map*>::iterator itmap=(vmaps.begin()+(max_maps_to_vis-1)); itmap!=vmaps.end()-1; ++itmap){
  for(std::vector<ORB_SLAM3::Map*>::iterator itmap=vmaps.begin(); itmap!=vmaps.end(); ++itmap){
    std::vector<ORB_SLAM3::MapPoint*> tmp_map_points = (*itmap)->GetAllMapPoints();
    Color tmp_color;
    tmp_color.red = 255 - 255/5*count;
    tmp_color.green = 0 + 255/5*count;
    tmp_color.blue = 0 + 255/5*count;
    pcl::PointCloud<pcl::PointXYZRGB> tmp_cloud = MapPointsToPointCloudWithColor(tmp_map_points, tmp_color);
    cloud_all += tmp_cloud;
    count++;
  }
  std::cout<<count<<std::endl;
  pcl::toROSMsg(cloud_all, cloud_msg);
  cloud_msg.header.stamp = current_frame_time_;
  cloud_msg.header.frame_id = map_frame_id_param_;
  atlas_points_publisher_.publish(cloud_msg);
}*/

pcl::PointCloud<pcl::PointXYZRGB> Node::MapPointsToPointCloudWithColor (std::vector<ORB_SLAM3::MapPoint*> map_points, Color color){
  //inspired by ethz-asl maplab visualization common-rviz-visualization.cc eigen3XdMatrixToPointCloud
  size_t num_points = map_points.size();
  //sensor_msgs::PointCloud2 point_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(num_points);
  for(size_t idx=0; idx<num_points; ++idx){
    pcl::PointXYZRGB point;
    point.x = map_points.at(idx)->GetWorldPos().at<float> (2);
    point.y = -1.0* map_points.at(idx)->GetWorldPos().at<float> (0);
    point.z = -1.0* map_points.at(idx)->GetWorldPos().at<float> (1);//cannot use double to instead float, strange...
    //std::cout<<point.z<<";";
    point.r = color.red;
    point.g = color.green;
    point.b = color.blue;
    cloud.push_back(point);
  }
  return cloud;
}

sensor_msgs::PointCloud2 Node::MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
  	cloud.fields[i].name = channel_id[i];
  	cloud.fields[i].offset = i * sizeof(float);
  	cloud.fields[i].count = 1;
  	cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

	unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i=0; i<cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
      //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    }
  }

  return cloud;
}


/*void Node::ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level) {
  orb_slam_->EnableLocalizationOnly (config.localize_only);
  min_observations_per_point_ = config.min_observations_for_ros_map;

  if (config.reset_map) {
    orb_slam_->Reset();
    config.reset_map = false;
  }

  orb_slam_->SetMinimumKeyFrames (config.min_num_kf_in_map);
}


bool Node::SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res) {
  res.success = orb_slam_->SaveMap(req.name);

  if (res.success) {
    ROS_INFO_STREAM ("Map was saved as " << req.name);
  } else {
    ROS_ERROR ("Map could not be saved.");
  }

  return res.success;
}*/


void Node::LoadOrbParameters (ORB_SLAM3::ORBParameters& parameters) {
  //ORB SLAM configuration parameters
  node_handle_.param(name_of_node_ + "/camera_fps", parameters.maxFrames, 30);
  node_handle_.param(name_of_node_ + "/camera_rgb_encoding", parameters.RGB, true);
  node_handle_.param(name_of_node_ + "/ORBextractor/nFeatures", parameters.nFeatures, 1200);
  node_handle_.param(name_of_node_ + "/ORBextractor/scaleFactor", parameters.scaleFactor, static_cast<float>(1.2));
  node_handle_.param(name_of_node_ + "/ORBextractor/nLevels", parameters.nLevels, 8);
  node_handle_.param(name_of_node_ + "/ORBextractor/iniThFAST", parameters.iniThFAST, 20);
  node_handle_.param(name_of_node_ + "/ORBextractor/minThFAST", parameters.minThFAST, 7);

  bool load_calibration_from_cam = false;
  node_handle_.param(name_of_node_ + "/load_calibration_from_cam", load_calibration_from_cam, false);

  //if (sensor_== ORB_SLAM3::System::STEREO || sensor_==ORB_SLAM3::System::RGBD) {
    node_handle_.param(name_of_node_ + "/ThDepth", parameters.thDepth, static_cast<float>(35.0));
    node_handle_.param(name_of_node_ + "/DepthMapFactor", parameters.depthMapFactor, static_cast<float>(1.0));
  //}

  if (load_calibration_from_cam) {
    ROS_INFO_STREAM ("Listening for camera info on topic " << node_handle_.resolveName(camera_info_topic_));
    sensor_msgs::CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, ros::Duration(1000.0));
    if(camera_info == nullptr){
        ROS_WARN("Did not receive camera info before timeout, defaulting to launch file params.");
    } else {
      parameters.fx = camera_info->K[0];
      parameters.fy = camera_info->K[4];
      parameters.cx = camera_info->K[2];
      parameters.cy = camera_info->K[5];

      parameters.baseline = camera_info->P[3];

      parameters.k1 = camera_info->D[0];
      parameters.k2 = camera_info->D[1];
      parameters.p1 = camera_info->D[2];
      parameters.p2 = camera_info->D[3];
      parameters.k3 = camera_info->D[4];
      return;
    }
  }

  bool got_cam_calibration = true;
  if (sensor_== ORB_SLAM3::System::STEREO || sensor_==ORB_SLAM3::System::RGBD) {
    got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_baseline", parameters.baseline);
  }

  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fx", parameters.fx);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_fy", parameters.fy);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cx", parameters.cx);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_cy", parameters.cy);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k1", parameters.k1);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k2", parameters.k2);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p1", parameters.p1);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_p2", parameters.p2);
  got_cam_calibration &= node_handle_.getParam(name_of_node_ + "/camera_k3", parameters.k3);

  if (!got_cam_calibration) {
    ROS_ERROR ("Failed to get camera calibration parameters from the launch file.");
    throw std::runtime_error("No cam calibration");
  }
}
