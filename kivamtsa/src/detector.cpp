#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h> 
#include "std_msgs/String.h"
#include "turtlebot_follower/FollowerConfig.h"
#include "dynamic_reconfigure/server.h"
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

//  pcl::visualization::CloudViewer viewer("newobjectdetector");   

namespace detector
{

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  //class ObjectDetector : public nodelet::Nodelet
  class ObjectDetector 
  {
  public:
    ObjectDetector() :  min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(0.8), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0)
    {
      ROS_INFO("Initialize object_detector");
      std::cout<<"Initialize object_detector"<<std::endl;
      private_nh.getParam("min_y", min_y_);
      private_nh.getParam("max_y", max_y_);
      private_nh.getParam("min_x", min_x_);
      private_nh.getParam("max_x", max_x_);
      private_nh.getParam("max_z", max_z_);
      private_nh.getParam("goal_z", goal_z_);
      private_nh.getParam("z_scale", z_scale_);
      private_nh.getParam("x_scale", x_scale_);
      private_nh.getParam("enabled", enabled_);
    }

    ~ObjectDetector()
    {
      //   delete config_srv_;
    }

    void run(){
      ROS_INFO("Function run()");
      cmd_pub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
      markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
      bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
      sub_= nh.subscribe<PointCloud>("depth/points", 1, &ObjectDetector::cloudcb, this);
      detector_pub = nh.advertise<std_msgs::String>("detect", 1000);
sub_for_cloudview = nh_for_cv.subscribe<PointCloud>("depth/points", 1, &ObjectDetector::getcloudview, this);
ros::Duration duration(1.0);
      //      duration.sleep();     
 ros::spin();
    }


  private:
    double min_y_; /**< The minimum y position of the points in the box. */
    double max_y_; /**< The maximum y position of the points in the box. */
    double min_x_; /**< The minimum x position of the points in the box. */
    double max_x_; /**< The maximum x position of the points in the box. */
    double max_z_; /**< The maximum z position of the points in the box. */
    double goal_z_; /**< The distance away from the robot to hold the centroid */
    double z_scale_; /**< The scaling factor for translational robot speed */
    double x_scale_; /**< The scaling factor for rotational robot speed */
    bool   enabled_; /**< Enable/disable following; just prevents motor commands */
    bool points_catched;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher cmd_pub_;
    ros::Publisher markerpub_;
    ros::Publisher bboxpub_;
    ros::Publisher detector_pub;
    ros::Subscriber sub_;
    ros::NodeHandle nh_for_cv;
    ros::Subscriber sub_for_cloudview;
    std_msgs::String detect;  


    void cloudcb(const PointCloud::ConstPtr& cloud)
    {
            ROS_INFO("Loading cloudcb");
      //X,Y,Z of the centroid
      float cent_x = 0.0;
      float cent_y = 0.0;
      float cent_z = 1e6;
      //Number of points observed
      unsigned int n = 0;
      ROS_INFO("Cloud points = %f", cloud->points.size());

      BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points)
	{


	  //ポイントの位置が適切かを確かめる
	  if(!std::isnan(cent_x) && !std::isnan(cent_y) && !std::isnan(cent_z))
	    {

	      //	  ROS_INFO("Boost foreach");
	      //pointがボックスの中に存在するかを確かめる。
	      //ボックスはmin_xとかで与えられている。
	      /*
		ここで飛行機を追いかける用に改造する。
		取ってきたポイントが果たして飛行機であるかを確認する機構を用いるか？
	      */
	      if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
		{
		  //このポイントを全体のポイントに加える
		  //	  ROS_INFO("get point");
		  cent_x += pt.x;
		  cent_y += pt.y;
		  cent_z = std::min(cent_z, pt.z);
		  n++; 
		}
	    }
	}

      //   ROS_INFO("The number of points is %f", n);

      //ポイントが
      if (n>4000)
	{
	  ROS_INFO("N > 4000");
	  std::cout<<"The number of points is "<<n<<std::endl;
	  detect.data = "d";
	  ros::Rate r(0.5);
	  r.sleep();
	  detector_pub.publish(detect);
	}
      else
	{
	  ROS_DEBUG("No points detected");
	  detect.data = "n";
	  ros::Rate r(0.5);
	  r.sleep();
	  detector_pub.publish(detect);
	}
    }


    void getcloudview(const PointCloud::ConstPtr& cloud)
    {
      ROS_INFO("cloud viewer # of Points = %f", cloud->points.size());

      if(cloud->points.empty())
	{
   ROS_INFO("Empty");
	}


      //      viewer.showCloud(cloud);
    }

  };

}

int main(int argc, char** argv){

  ros::init(argc, argv,"object_detector");
  detector::ObjectDetector detector;
  detector.run();




}
