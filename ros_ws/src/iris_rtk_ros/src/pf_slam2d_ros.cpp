

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <rtk/structs/image.h>
#include <rtk/ros/pf_slam2d_ros.h>

namespace rtk {

PFSlam2DROS::PFSlam2DROS()
    : nh_(), pnh_("~")
{
    // Load parameters from the server.
    double tmp;

    pnh_.param("global_frame_id", global_frame_id_, std::string("/map"));
    pnh_.param("odom_frame_id",   odom_frame_id_,   std::string("/odom"));
    pnh_.param("base_frame_id",   base_frame_id_,   std::string("/base_link"));

    pnh_.param("scan_topic", scan_topic_, std::string("/scan"));

    pnh_.param("transform_tolerance", tmp, 0.1); transform_tolerance_.fromSec(tmp);

    Vector2d pos;
    pnh_.param("initial_pos_x", pos[0], 0.0);
    pnh_.param("initial_pos_y", pos[1], 0.0);
    pnh_.param("initial_pos_a", tmp, 0.0);
    rtk::geom::Pose2D prior(pos, tmp);

    rtk::slam::PFSlam2D::Options options;
    pnh_.param("d_thresh", options.trans_thresh, 0.5);
    pnh_.param("a_thresh", options.rot_thresh, 0.5);
    pnh_.param("l2_max",   options.l2_max, 0.5);
    pnh_.param("lgain",    options.meas_sigma_gain, 3.0);
    pnh_.param("resolution", options.resolution, 0.05);
    pnh_.param("use_compression", options.use_compression, false);
    pnh_.param("mrange",   max_range_, 16.0);
    pnh_.param("strategy", options.strategy, std::string("gn"));

    int itmp;
    pnh_.param("patch_size", itmp, 32); options.patch_size = itmp;
    pnh_.param("particles",  itmp, 30); options.particles = itmp;
    pnh_.param("cache_size", itmp, 100); options.cache_size = itmp;

    pnh_.param("map_publish_period", tmp, 5.0);
    pnh_.param("threads", options.threads, -1);
    map_publish_period_ = ros::WallDuration(tmp);

    slam2d_ = new rtk::slam::PFSlam2D(options);
    slam2d_->setPrior(prior);

    // Setup TF workers ...
    tf_ = new tf::TransformListener();
    tfb_= new tf::TransformBroadcaster();

    // Syncronized LaserScan messages with odometry transforms. This ensures that an odometry transformation
    // exists when the handler of a LaserScan message is called.
    laser_scan_sub_    = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
    laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 100);
    laser_scan_filter_->registerCallback(boost::bind(&PFSlam2DROS::onLaserScan, this, _1));

    // Setup publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 2);
    map_pub_  = nh_.advertise<nav_msgs::OccupancyGrid>("map",      1, true);
    dist_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("distance", 1, true);
    patch_pub_= nh_.advertise<nav_msgs::OccupancyGrid>("patch",    1, true);
    poses_pub_= nh_.advertise<geometry_msgs::PoseArray>("poses", 1, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);

    ros_occ_.header.frame_id = global_frame_id_;
    ros_cost_.header.frame_id = global_frame_id_;
    ros_patch_.header.frame_id = global_frame_id_;

    poses_.header.frame_id = global_frame_id_;

    // Setup service
    ss_ = nh_.advertiseService("dynamic_map", &PFSlam2DROS::onGetMap, this);

    ROS_INFO("node up and running");
}

PFSlam2DROS::~PFSlam2DROS()
{
    delete laser_scan_filter_;
    delete laser_scan_sub_;
    delete slam2d_;
}

void PFSlam2DROS::onLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    int laser_index = -1;

    // verify if it is from a known source
    if ( frame_to_laser_.find( laser_scan->header.frame_id ) == frame_to_laser_.end() ){

        laser_index = (int)frame_to_laser_.size();  // simple ID generator :)
        lasers_update_.push_back(false);            // do not update when a laser is added to the known list.
        frame_to_laser_[laser_scan->header.frame_id] = laser_index;

        // find the origin of the sensor
        tf::Stamped<tf::Pose> identity(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)),
                                       ros::Time(), laser_scan->header.frame_id);
        tf::Stamped<tf::Pose> laser_origin;
        try{ tf_->transformPose(base_frame_id_, identity, laser_origin); }
        catch(tf::TransformException& e)
        { ROS_ERROR("Could not find origin of %s", laser_scan->header.frame_id.c_str()); return; }

        rtk::geom::Pose3D lp(laser_origin.getOrigin().x(), laser_origin.getOrigin().y(), 0,
                             0, 0, tf::getYaw(laser_origin.getRotation()));

        lasers_origin_.push_back( lp );

        ROS_INFO("New laser configured (id=%d frame_id=%s)", laser_index, laser_scan->header.frame_id.c_str() );
    }else{
        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

    // Where was the robot at the time of the scan ?
    tf::Stamped<tf::Pose> identity(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)),
                                   laser_scan->header.stamp, base_frame_id_);
    tf::Stamped<tf::Pose> odom_tf;
    try{ tf_->transformPose(odom_frame_id_, identity, odom_tf); }
    catch(tf::TransformException& e)
    { ROS_WARN("Failed to compute odom pose, skipping scan %s", e.what() ); return; }

    rtk::geom::Pose2D odom(odom_tf.getOrigin().x(), odom_tf.getOrigin().y(),
                           tf::getYaw(odom_tf.getRotation()));

    bool update;

    size_t size = laser_scan->ranges.size();
    size_t beam_step = 1;

    float max_range = max_range_; //laser_scan->range_max;
    float min_range = laser_scan->range_min;
    float angle_min = laser_scan->angle_min;
    float angle_inc = laser_scan->angle_increment;

    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);

    cloud->sensor_origin_ = lasers_origin_[laser_index].xyz();
    cloud->sensor_orientation_ = Quaterniond(lasers_origin_[laser_index].se3().so3().matrix());

    cloud->points.reserve(laser_scan->ranges.size());
    for(size_t i = 0; i < size; i += beam_step ){

        if ( laser_scan->ranges[i] >= max_range || laser_scan->ranges[i] <= min_range )
            continue;

        if (std::isnan(laser_scan->ranges[i]) || std::isinf(laser_scan->ranges[i]))
            continue;

        Eigen::Vector3d point;
        point << laser_scan->ranges[i] * std::cos(angle_min+(i*angle_inc)),
                 laser_scan->ranges[i] * std::sin(angle_min+(i*angle_inc)),
                 0;

        cloud->points.push_back( point );
    }

    update = slam2d_->update(cloud, odom, laser_scan->header.stamp.toSec());

    if (update){
        rtk::geom::Pose2D pose = slam2d_->getPose();

        // subtracting base to odom from map to base and send map to odom instead
        tf::Stamped<tf::Pose> odom_to_map;
        try{
            tf::Transform tmp_tf(tf::createQuaternionFromYaw(pose.rotation()), tf::Vector3(pose.x(), pose.y(), 0));
            tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(), laser_scan->header.stamp, base_frame_id_);
            tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);

        }catch(tf::TransformException){
            ROS_WARN("Failed to subtract base to odom transform");
            return;
        }

        latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                   tf::Point(odom_to_map.getOrigin()));

        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                            transform_expiration,
                                            global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);

        //--
        nav_msgs::Path path;
        path.header.frame_id = global_frame_id_;
        path.header.stamp    = laser_scan->header.stamp;

        uint32_t idx = slam2d_->getBestParticleIdx();
        const size_t num_poses = slam2d_->getParticles()[idx].poses.size();
        for (size_t i = 0; i < num_poses; ++i){

            geometry_msgs::PoseStamped p;
            p.header.frame_id = global_frame_id_;
            p.header.stamp    = laser_scan->header.stamp;
            p.pose.position.x = slam2d_->getParticles()[idx].poses[i].x();
            p.pose.position.y = slam2d_->getParticles()[idx].poses[i].y();
            p.pose.position.z = 0.0;
            p.pose.orientation = tf::createQuaternionMsgFromYaw(slam2d_->getParticles()[idx].poses[i].rotation());

            path.poses.push_back(p);
        }
        path_pub_.publish(path);

        //--
        std::cout << "NEFF " << slam2d_->getNeff() << std::endl;

    } else {
        // Nothing has change, therefore, republish the last transform.
        ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration,
                                            global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
    } // end if (update)

    const size_t num_particles = slam2d_->getParticles().size();
    poses_.poses.resize(num_particles);
    for (size_t i = 0; i < num_particles; ++i){

        poses_.poses[i].position.x = slam2d_->getParticles()[i].pose.x();
        poses_.poses[i].position.y = slam2d_->getParticles()[i].pose.y();
        poses_.poses[i].position.z = 0.0;
        poses_.poses[i].orientation = tf::createQuaternionMsgFromYaw(slam2d_->getParticles()[i].pose.rotation());
    }

    poses_pub_.publish(poses_);

    ros::WallTime now = ros::WallTime::now();
    if ((map_publish_period_.toSec() > 0.0f ) &&
        (now - map_publish_last_time_) >= map_publish_period_ )
    {
        nav_msgs::OccupancyGrid ros_occ;
        ros_occ.header.frame_id = global_frame_id_;
        ros_occ.header.stamp = laser_scan->header.stamp;

        if (map_pub_.getNumSubscribers() > 0 ){
            OccupancyMsgFromOccupancyMap(ros_occ);
            ros_occ_.header.stamp = laser_scan->header.stamp;
            map_pub_.publish(ros_occ);

            map_publish_last_time_ = ros::WallTime::now();
        }

        if (dist_pub_.getNumSubscribers() > 0){
            DistanceMsgFromOccupancyMap(ros_cost_);
            ros_cost_.header.stamp = laser_scan->header.stamp;
            dist_pub_.publish(ros_cost_);

            map_publish_last_time_ = ros::WallTime::now();
        }

        if (patch_pub_.getNumSubscribers() > 0){
            PatchMsgFromOccupancyMap(ros_patch_);
            ros_patch_.header.stamp = laser_scan->header.stamp;
            patch_pub_.publish(ros_patch_);

            map_publish_last_time_ = ros::WallTime::now();
        }

    }

}

bool PFSlam2DROS::OccupancyMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg)
{
    const rtk::sdm::FrequencyOccupancyMap* map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if ( width*height > msg.data.size() )
        msg.data.resize(width*height);

    rtk::Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin](const Vector3ui& coords){
        Vector3ui adj_coords = coords - imin;

        if (map->isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else if (map->isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 100;
        else
            image(adj_coords(0), adj_coords(1)) = 0xff;
    });

    memcpy(&msg.data[0], image.data.get(), width*height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);

    return true;
}

bool PFSlam2DROS::DistanceMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg)
{
    const rtk::sdm::DynamicDistanceMap* map = slam2d_->getDistanceMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    double factor = 1.0 / map->maxDistance();

    if (width == 0 || height == 0)
        return false;

    if ( width*height > msg.data.size() )
        msg.data.resize(width*height);

    rtk::Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin, factor](const rtk::Vector3ui& coords){
        Vector3ui adj_coords = coords - imin;
        image(adj_coords(0), adj_coords(1)) = 100 - 100 * map->distance(coords) * factor;
    });

    memcpy(&msg.data[0], image.data.get(), width*height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);

    return true;
}

bool PFSlam2DROS::PatchMsgFromOccupancyMap(nav_msgs::OccupancyGrid& msg)
{
    const rtk::sdm::FrequencyOccupancyMap* map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3d min, max;
    map->bounds(min, max);

    Vector3ui imin = map->w2m(min);
    Vector3ui imax = map->w2m(max);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    msg.data.resize(width*height);

    for (unsigned int j = imin(1); j < imax(1); ++j)
        for (unsigned int i = imin(0); i < imax(0); ++i){
            if (map->patchAllocated(Vector3ui(i,j,0))){
                if (map->patchIsUnique(Vector3ui(i,j,0)))
                    msg.data[ (j-imin[1])*width + (i-imin[0]) ] = 100;
                else
                    msg.data[ (j-imin[1])*width + (i-imin[0]) ] = 50;
            }
            else
                msg.data[ (j-imin[1])*width + (i-imin[0]) ] = -1;
        }

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    msg.info.origin.position.x = min[0];
    msg.info.origin.position.y = min[1];
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);

    return true;
}

bool PFSlam2DROS::onGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{

    res.map.header.frame_id = global_frame_id_;
    res.map.header.stamp = ros::Time::now();

    OccupancyMsgFromOccupancyMap(res.map);

    return true;
}

void PFSlam2DROS::fromBag(const std::string bag_file)
{
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/tf"));
    topics.push_back("/scan");

    ros::Publisher scan_pub = nh_.advertise<sensor_msgs::LaserScan>("/scan", 5);

    rosbag::View viewall(bag, rosbag::TopicQuery(topics));

    // Store up to 5 messages and there error message (if they cannot be processed right away)
    std::queue<std::pair<sensor_msgs::LaserScan::ConstPtr, std::string> > s_queue;
    foreach(rosbag::MessageInstance const m, viewall)
    {
        if (not ros::ok()) return;

        tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
        if (cur_tf != NULL) {
            for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
            {
                geometry_msgs::TransformStamped transformStamped;
                tf::StampedTransform stampedTf;
                transformStamped = cur_tf->transforms[i];
                tf::transformStampedMsgToTF(transformStamped, stampedTf);
                tf_->setTransform(stampedTf);
            }
        }

        sensor_msgs::LaserScan::ConstPtr s = m.instantiate<sensor_msgs::LaserScan>();
        if (s != NULL) {
            if (!(ros::Time(s->header.stamp)).is_zero())
            {
                s_queue.push(std::make_pair(s, ""));
            }
            // Just like in live processing, only process the latest 5 scans
            if (s_queue.size() > 5) {
                ROS_WARN_STREAM("Dropping old scan: " << s_queue.front().second);
                s_queue.pop();
            }
            // ignoring un-timestamped tf data
        }

        // Only process a scan if it has tf data
        while (!s_queue.empty())
        {
            try
            {
                tf::StampedTransform t;
                tf_->lookupTransform(s_queue.front().first->header.frame_id, odom_frame_id_, s_queue.front().first->header.stamp, t);
                scan_pub.publish(s_queue.front().first);
                this->onLaserScan(s_queue.front().first);
                s_queue.pop();
            }
            // If tf does not have the data yet
            catch(tf2::TransformException& e)
            {
                // Store the error to display it if we cannot process the data after some time
                s_queue.front().second = std::string(e.what());
                break;
            }
        }
    }

    bag.close();
}

} /* rtk */

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pf_slam2d_ros");
    rtk::PFSlam2DROS slam2d_ros;

    if (argc > 1)
        slam2d_ros.fromBag( argv[1] );
    else
        ros::spin();

    return 0;
}

