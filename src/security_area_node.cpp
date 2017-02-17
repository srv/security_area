#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class SecurityArea {
 public:
  SecurityArea(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
      : nh_(nh), nhp_(nhp)  {
    // radius of the spherical security zone
    nhp_.param("radius", radius_, 1);
    // by default, the angular increment is 10deg = pi/18
    nhp_.param("increment_angle", increment_, 1);
    nhp_.param("frame_origin", origin_frame_id_, std::string("/sparus2"));
    nhp_.param("frame_destination", destination_frame_id_, std::string("/odom"));

    security_area_pub_ = nhp_.advertise<PointCloud>("/security_area_out", 10);

    ROS_INFO_STREAM("[SecurityArea]" << " Parameters:"
      << "\n\t* Radius            : " << radius_
      << "\n\t* Increment angle   : " << increment_
      << "\n\t* Frame origin      : " << origin_frame_id_
      << "\n\t* Frame destination : " << destination_frame_id_);

    timer_ = nh_.createTimer(ros::Duration(1.0),
                             &SecurityArea::timerCallback,
                             this);
  }

 private:

  void timerCallback(const ros::TimerEvent&) {
    tf::StampedTransform transform;
    try {
      listener_.lookupTransform(origin_frame_id_, destination_frame_id_,
                                ros::Time(0), transform);
    }
    catch (tf::TransformException ex){ // launches an exception if the listener returns an error
      ROS_ERROR("%s",ex.what()); // shows the error message
      ros::Duration(1.0).sleep();
    }

    /* RECUPERATE TRANSFORM DATA */

    double x,y,z, thetarad, phirad, px, py, pz;
    int i, cloud_size;
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    z = transform.getOrigin().z();

    /* GENERATE THE SPHERICAL POINT CLOUD centred at the origin.
     * Sphere equations (NED coordinate system)
     *   x= -r * sin(theta) * cos(phi)
     *   y=  r * sin(theta) * sin(phi)
     *   z= -r * cos(theta)
     * where 0 < theta < 180 degrees and  0 < phi < 360 degrees
     */

    PointCloud::Ptr point_cloud(new PointCloud());
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = origin_frame_id_;
    pcl_conversions::toPCL(header, point_cloud->header);

    cloud_size = ((180-0)/increment_)*((360-0)/increment_);
    point_cloud->width = cloud_size;
    point_cloud->height = 1;
    point_cloud->points.resize(cloud_size);
    i=0;
    for(int theta = 0; theta < 180; theta += increment_ ) {
      thetarad= theta * (M_PI /180);
      for(int phi = 0 ; phi < 360; phi+=increment_ ) {
        phirad= phi * (M_PI /180);
        px = x - (radius_)*((float) sin(thetarad))*((float)cos(phirad));
        py = y + (radius_)*((float) sin(thetarad))*((float)sin(phirad));
        pz = z - (radius_)*((float) cos(thetarad));
        point_cloud->points[i].x = px;
        point_cloud->points[i].y = py;
        point_cloud->points[i].z = pz;
        i++;
      }
    }
    // publish the point cloud
    security_area_pub_.publish(point_cloud);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher security_area_pub_;
  tf::TransformListener listener_;
  ros::Timer timer_;

  int radius_;
  int increment_;
  std::string origin_frame_id_;
  std::string destination_frame_id_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "security_area");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  SecurityArea sa(nh, nhp);
  ros::spin();
  return 0;
};

