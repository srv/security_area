#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // define a PointCloud as a pcl Pointcloud of points XYZ. 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char** argv){
  ros::init(argc, argv, "security_area");

  ros::NodeHandle node_; // declare new node_ handler
  ros::NodeHandle nhp_("~");

  int radius_, increment_;
  std::string origin_frame_id_, destination_frame_id_ , node_name_ = ros::this_node::getName();


  ros::Publisher security_area_pub_ = nhp_.advertise<PointCloud>("/security_area_out", 10); // new publisher: publish a PointCloud2 
  nhp_.param("radius", radius_, 1); // radious of the spherical security zone
  nhp_.param("increment_angle", increment_, 1); // by defaul, the angular increment is 10º = pi/18 
  nhp_.param("frame_origin", origin_frame_id_, std::string("/sparus2"));
  nhp_.param("frame_destination", destination_frame_id_, std::string("/odom"));
  
  ROS_WARN_STREAM("[" << node_name_ << "]" << ":params: " << radius_ << ", " << increment_ << ", " << destination_frame_id_ << ")");


  tf::TransformListener listener;

  ros::Rate rate(1.0); // define the loop frequency


  while (node_.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(origin_frame_id_, destination_frame_id_,  
                               ros::Time(0), transform); 
    }
    catch (tf::TransformException ex){ // launches an exception if the listener returns an error 
      ROS_ERROR("%s",ex.what()); // shows the error message
      ros::Duration(1.0).sleep();
    }

    /* RECUPERATE TRANSFORM DATA */
    
    double x,y,z, thetarad, phirad, Verticex, Verticey, Verticez; 
    int theta, phi,i, sizeof_point_cloud;
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    z = transform.getOrigin().z();

    ROS_WARN_STREAM("USBL Origin: (" << x << ", " << y << ", " << z << ")");

    /* GENERATE THE SPHERICAL POINT CLOUD centered at the origin */
    /* sphere equations: 
          taking into account that the origin is refered to a NED coordinate system, the sphere coordinates are: 
            x= -rsin(theta) cos(phi) 
            y= rsin(theta) sin(phi)
            z= -rcos(theta) , where      0<theta<180º and  0 < phi < 360 º

    */ 
    
    PointCloud::Ptr point_cloud(new PointCloud()); // create a pcl Pointcloud 
    std_msgs::Header header;
    ros::Time now = ros::Time::now();
    header.stamp = now;
    header.frame_id = origin_frame_id_;
    pcl_conversions::toPCL(header, point_cloud->header);


    sizeof_point_cloud = ((180-0)/increment_)*((360-0)/increment_);
    point_cloud->width = sizeof_point_cloud;
    point_cloud->height = 1;
    point_cloud->points.resize(sizeof_point_cloud);
    i=0;
    for( theta = 0 ; theta < 180; theta+=increment_ )
    {
             thetarad= theta * (M_PI /180);
             for(phi = 0 ; phi < 360; phi+=increment_ )
             {
                  phirad= phi * (M_PI /180); 
                  Verticex= x-(radius_)*((float) sin(thetarad))*((float)cos(phirad));
                  Verticey= y+(radius_)*((float) sin(thetarad))*((float)sin(phirad));
                  Verticez= z-(radius_)*((float) cos(thetarad));
                  point_cloud->points[i].x = Verticex;
                  point_cloud->points[i].y = Verticey;
                  point_cloud->points[i].z = Verticez;

                  i++;
                 
             }
    }
    ROS_WARN_STREAM("Publishing the Sphere of Security: " << i << "points");
    // publish the point cloud
    security_area_pub_.publish(point_cloud);

   
    rate.sleep();
  }
  return 0;
};

