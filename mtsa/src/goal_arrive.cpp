
#include "ros/ros.h"
/* gcc -lsocket -lnsl -o inet_server inet_server.c */
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>

#define ERROR(x) {				\
    fprintf(stderr, "client - ");		\
    perror(x);					\
    exit(1);					\
  }


namespace ARRIVE_to_MTSA{
  class ArriveToMTSA{
  private:
    ros::Publisher arrive_pub;
    ros::Subscriber odometry_subscriber;
    void arrive_callback(const nav_msgs::Odometry msg);
    // void arrive_callback(const geometry_msgs::Twist msg);
    void subscriber_init();
    void publisher_init();
  public:
    int sockfd, len, result;
    struct sockaddr_in address;
    int com,detect_arrive; 
    nav_msgs::Odometry c_odom,l_odom;
    geometry_msgs::Twist c_vel,l_vel;
    std_msgs::String arrive;
    std::stringstream ss;

    ArriveToMTSA(ros::NodeHandle node_handle,ros::NodeHandle private_node_handle )
      : nh(node_handle), private_nh(private_node_handle)
    {
      ROS_INFO("BumperToMTSA");
      
      detect_arrive = 0;
    }
    ~ArriveToMTSA(){}
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    void run();


  };

  void ArriveToMTSA::arrive_callback(const nav_msgs::Odometry msg){
    ROS_INFO("arrive_callback in");
    c_odom = msg;
    
    if(c_odom.pose.pose.position.x > 1.5 &&
       c_odom.pose.pose.position.x <  4.0){
				      //       c_odom.pose.pose.position.y > 1.0 && 
	 //       c_odom.pose.pose.position.y < 3.0){
      ROS_INFO("pos m");
      arrive.data = "m";
      arrive_pub.publish(arrive);
    }else if(c_odom.pose.pose.position.x >= 0.0 &&
	     c_odom.pose.pose.position.x <= 1.5){
	     //	     c_odom.pose.pose.position.y >= 0.0 && 
	     // c_odom.pose.pose.position.y <= 1.0){
      ROS_INFO("pos w");
      arrive.data = "w";
      arrive_pub.publish(arrive);
    }else if(c_odom.pose.pose.position.x >= 4.0 &&
	     c_odom.pose.pose.position.x <= 5.5 ){
	 //	     c_odom.pose.pose.position.y >= 3.0 && 
	 //  c_odom.pose.pose.position.y <= 4.5){
      ROS_INFO("pos e");
      arrive.data = "e";
      arrive_pub.publish(arrive);
    }
    ROS_INFO("arrive_callback_out");
  }
  void ArriveToMTSA::subscriber_init(){
    ROS_INFO("subscriber_init");
    std::string odometry_topic_name = "odom";
    odometry_subscriber = nh.subscribe<nav_msgs::Odometry>(odometry_topic_name, 1,
							   &ArriveToMTSA::arrive_callback,this);
  }
  void ArriveToMTSA::publisher_init(){
    ROS_INFO("publisher_init");
    arrive_pub = nh.advertise<std_msgs::String>("arrive",1000);

  }

  void ArriveToMTSA::run(){
    subscriber_init();
    publisher_init();

  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "arrive_to_MTSA");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ARRIVE_to_MTSA::ArriveToMTSA atm(nh, nh_private);
  atm.run();
  ros::spin();
}
