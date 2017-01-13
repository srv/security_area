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

  ros::NodeHandle node; // declare new node handler
  ros::NodeHandle nhp("~");

  int radius_, increment_;
  std::string origin_frame_id_, destination_frame_id_ , node_name_ = ros::this_node::getName();
  //sensor_msgs::PointCloud2 area;


  ros::Publisher security_area_pub_ = nhp.advertise<PointCloud>("/security_area_out", 10); // new publisher: publish a PointCloud2 
  nhp.param("radius", radius_, 1); // radious of the spherical security zone
  nhp.param("increment_angle", increment_, 1); // by defaul, the angular increment is 10º = pi/18 
  nhp.param("frame_origin", origin_frame_id_, std::string("/sparus2"));
  nhp.param("frame_destination", destination_frame_id_, std::string("/odom"));
  
  ROS_WARN_STREAM("params: " << radius_ << ", " << increment_ << ", " << destination_frame_id_ << ")");


  tf::TransformListener listener;

  ros::Rate rate(1.0); // define the loop frequency


  while (node.ok()){
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
    // origin, position of the object (usbl, G500, etc...) with respect to /map 

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
    i=0;
    PointCloud::Ptr point_cloud(new PointCloud()); // create a pcl Pointcloud as defined in the typedef
    std_msgs::Header header;
    ros::Time now = ros::Time::now();
    header.stamp = now;
    header.frame_id = origin_frame_id_;
    pcl_conversions::toPCL(header, point_cloud->header);

    ROS_WARN_STREAM("");

    sizeof_point_cloud = ((180-0)/increment_)*((360-0)/increment_);
//    ROS_WARN_STREAM("sizeof_point_cloud: " << sizeof_point_cloud);
    point_cloud->width = sizeof_point_cloud;
    point_cloud->height = 1;
    point_cloud->points.resize(sizeof_point_cloud);

    for( theta = 0 ; theta < 180; theta+=increment_ )
    {
             thetarad= theta * (M_PI /180);
             for(phi = 0 ; phi < 360; phi+=increment_ )
             {
                  phirad= phi * (M_PI /180); 
                  Verticex= x-(radius_)*((float) sin(thetarad))*((float)cos(phirad));
                  Verticey= y+(radius_)*((float) sin(thetarad))*((float)sin(phirad));
                  Verticez= z-(radius_)*((float) cos(thetarad));
            //      ROS_WARN_STREAM("points: " << Verticex << ", " << Verticey << ", " << Verticez);

                  
            //      ROS_WARN_STREAM("point_cloud height: " << point_cloud->height);
            //      ROS_WARN_STREAM("point_cloud size: " << point_cloud->points.size());

                  point_cloud->points[i].x = Verticex;
                  point_cloud->points[i].y = Verticey;
                  point_cloud->points[i].z = Verticez;
            //      ROS_WARN_STREAM("point_cloud point: " << point_cloud->points[i].x << ", " << point_cloud->points[i].y << ", " << point_cloud->points[i].z);

                  i++;
                 
             }
    }
    ROS_WARN_STREAM("Publishing the Sphere of Security: " << i << "points");
    // publish the point cloud
    security_area_pub_.publish(point_cloud);

    /**/
   
    rate.sleep();
  }
  return 0;
};

