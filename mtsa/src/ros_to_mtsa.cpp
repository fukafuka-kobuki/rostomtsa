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
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include "kobuki_msgs/BumperEvent.h"
#include "std_msgs/String.h"
#include <sstream>

#define ERROR(x) {				\
    fprintf(stderr, "server - ");		\
    perror(x);					\
    exit(1);					\
  }



namespace Ros_to_Mtsa{
  class ROSToMTSA{
  private:
    ros::Publisher pickput_pub;
    ros::Publisher goal_pub;
    ros::Subscriber arrive_sub;
    ros::Subscriber bump_sub;
    ros::Subscriber vel_sub;

    void subscriber_init();
    void publisher_init();
    void arrive_callback(const std_msgs::String msg);
    void bump_callback(const std_msgs::String msg);
    void vel_callback(const geometry_msgs::Twist twist);
    void arrive_detect();
    void bump_detect();
    
  public:
    int sockfd, len, result, pos,p_detect,a_detect,com;
    struct sockaddr_in address;

    std_msgs::String msgp;
    std::stringstream ss;
    geometry_msgs::PoseStamped goal_e;
    geometry_msgs::PoseStamped goal_m;
    geometry_msgs::PoseStamped goal_w;
      

    ROSToMTSA(ros::NodeHandle node_handle,ros::NodeHandle private_node_handle)
      :nh(node_handle),private_nh(private_node_handle)
    {
      ROS_INFO("ROSToMTSA constructer in");

      //initialize variables
      com=3, p_detect= 0, a_detect = 0;

      /*create socket for client*/
      sockfd = socket(AF_INET, SOCK_STREAM, 0);
      
      /*name the scket same as server socket*/
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = inet_addr("136.187.81.230");
      address.sin_port = htons(9999);
      len = sizeof(address);
      
      /*connect client and server socket*/
      result = connect(sockfd, (struct sockaddr *)&address, len);
      if(result == -1) ERROR("oops : client1");

      ROS_INFO("goal initialize");
      //goal east
      goal_e.header.frame_id = "map";
      goal_e.pose.position.x = 5.0;
      goal_e.pose.position.y = 3.5;
      goal_e.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      
      //goal west
      goal_w.header.frame_id = "map";
      goal_w.pose.position.x = 1.0;
      goal_w.pose.position.y = 0.0;
      goal_w.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

      //goal middle
      goal_m.header.frame_id = "map";
      goal_m.pose.position.x = 3.0;
      goal_m.pose.position.y = 2.0;
      goal_m.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);



      ROS_INFO("ROSToMTSA constructer out");
    }
    ~ROSToMTSA(){}
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    
    void run();
    
  };
  
  
  void ROSToMTSA::arrive_callback(const std_msgs::String msg)
  {
    //localization reasoning the topic published by "goal_arrive node"

    if(msg.data == "m"){
      ROS_INFO("Kobuki arrives at middle!");
      pos = 1;
    }else if(msg.data == "e"){
      ROS_INFO("Kobuki arrives at east!");
      pos = 2;
    }else if(msg.data == "w"){
      ROS_INFO("Kobuki arrives at west!");
      pos = 0;
    }    
  }

  void ROSToMTSA::bump_callback(const std_msgs::String msg)
  {
    //detect bumper was kicked reasoning the topic pubrished by "bump_to_mtsa"

    ROS_INFO("bump_callback in!");
    if(msg.data == "b"){
      ROS_INFO("bump!");
      p_detect   = 1;
    }

        ROS_INFO("bump_callback out!");
  }

  void ROSToMTSA::vel_callback(const geometry_msgs::Twist twist)
  {
    //look the velocity of kobuki and detect its stopping

    //    ROS_INFO("vel_callback in!");
    if(twist.linear.x == 0.0 &&
       twist.linear.y == 0.0 &&
       twist.linear.z == 0.0 &&
       twist.angular.x == 0.0 &&
       twist.angular.y == 0.0 &&       
       twist.angular.z == 0.0        
       ){
      ROS_INFO("arrive!");
      a_detect = 1;
    }
    //    ROS_INFO("vel_callback out!");
  }


  void ROSToMTSA::subscriber_init(){
    ROS_INFO("subscriber_init");  
    arrive_sub = nh.subscribe("arrive",1000,&ROSToMTSA::arrive_callback,this);
    bump_sub = nh.subscribe("bump",1000,&ROSToMTSA::bump_callback,this);
    vel_sub = nh.subscribe<geometry_msgs::Twist>("/navigation_velocity_smoother/raw_cmd_vel",10,&ROSToMTSA::vel_callback,this);
  }

  void ROSToMTSA::publisher_init(){
    ROS_INFO("publisher_init");
    pickput_pub = nh.advertise<std_msgs::String>("chatter",1000);

    std::string goal_topic_name = "/move_base_simple/goal";
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_name.c_str(), 1);
  }

  void ROSToMTSA::arrive_detect(){
    ROS_INFO("arrive detect in!");
    if(pos == 0){
      ROS_INFO("arrive west");
      com = 3;
    }else if(pos == 1){
      ROS_INFO("arrive middle");
      com = 2;      
    }else if(pos == 2){
      ROS_INFO("arrive east");
      com = 1;
    }
  }



  void ROSToMTSA::run(){
    ROS_INFO("run");
    publisher_init();
    subscriber_init();

    pos = 0;

    ros::Rate r(1);
    r.sleep();

    //    com = 3;
    // write(sockfd, &com, 1);
      ROS_INFO("send: arrive w");
      write(sockfd, &com, 1);


  
    while(1){
      //   printf("Kobuki is at position %d !\n", pos);


      printf("server waiting\n");
      /*become being able to weite or read thorough client_sockfd*/
 
     read(sockfd, &com,1);

      printf("command [%d] is received!\n", com);
      com = com-48;
      printf("command [%d] is received!\n", com);

      /*
	received command number is related to Controllable action
	1:move w
	2:move e
	3:pickup
	4:putdown
       */

      if(com == 2){
	ROS_INFO("command move e is received!\n");
	//receive move e
	if(pos == 0){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_m);
	  ROS_INFO("move e(go to m) was published");
	  while(a_detect != 1){
	    ros::spinOnce();
	    //	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();
	}else if(pos ==1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_e);	    
	  ROS_INFO("move e  (go to e)was published");
	  while(a_detect != 1){
	    ros::spinOnce();
	    //	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();
	} else if(pos ==2){
	  ROS_INFO("kobuki is already at east");
	  com = 1;
	}
      }else if(com == 1){
	//西へ移動
	printf("command move w is received!\n");
	if(pos == 1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_m);
	  ROS_INFO("move w  was published");
	  while(a_detect != 1){
	    ros::spinOnce();
	    //	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();	     
	}else if(pos == 2){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_w);
	  ROS_INFO("move w  was published");
	  while(a_detect != 1){
	    ros::spinOnce();
	    //	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();
	} else if(pos ==0){
	  ROS_INFO("kobuki is already at east");
	  com = 3;
	}
      }else if(com == 3){
	//荷物を持つ
	ROS_INFO_ONCE("command pickup is received!\n");
	ros::Rate r(10);
	r.sleep();
	msgp.data = "pick";
	pickput_pub.publish(msgp);
	ROS_INFO_ONCE("waiting pick succ/fail");
	ros::spinOnce();
	if(p_detect == 1){
	  com = 4;
	  p_detect == 0;
	}else if(p_detect == 0){
	  com = 5;
	}
      }else if(com== 4){	  
	//荷物を置く
	ROS_INFO_ONCE("command putdown is received!\n");
	ros::Rate r(10);
	r.sleep();
	msgp.data = "put";
	pickput_pub.publish(msgp);
	ROS_INFO_ONCE("waiting pick succ/fail");
	ros::spinOnce();
	if(p_detect == 1){
	  com = 6;
	  p_detect == 0;
	}else if(p_detect == 0){
	  com = 7;
	}
      }

      //命令の書き込み
      write(sockfd, &com, 2);
      a_detect = 0;
      /*
	sent command number is related to Monitorable action
	1:arrive e
	2:arrive m
	3:arrive w
	4:picksuccess
	5:pickfail
	6:putsuccess
	7:putfail
       */


      ROS_INFO_ONCE("command number %d is sent to MTSA!\n",com);
    }
    //    close(sockfd); 
  }
}

  
int main(int argc,char** argv){
  ros::init(argc,argv,"ros_to_MTSA");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Ros_to_Mtsa::ROSToMTSA rtm(nh,nh_private);
  rtm.run();
  ros::spin();
}
