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
#include <time.h>
#include "kobuki_msgs/Sound.h"

#define ERROR(x) {				\
    fprintf(stderr, "server - ");		\
    perror(x);					\
    exit(1);					\
  }



namespace Kiva_to_Mtsa{
  class KIVAToMTSA{
  private:
    ros::Publisher pickput_pub;
    ros::Publisher goal_pub;
    ros::Publisher sound_pub;
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
    int sockfd, len, result, pos,pos_b,p_detect,a_detect,com,vel,shelfx,shelfy;
    clock_t time,time_n,c_time;
    struct sockaddr_in address;

    std_msgs::String msgp;
    std::stringstream ss;
    geometry_msgs::PoseStamped goal_shelf;
    geometry_msgs::PoseStamped goal_s0;
    geometry_msgs::PoseStamped goal_s1;
    geometry_msgs::PoseStamped goal_w;
    geometry_msgs::PoseStamped goal_d0; 
    geometry_msgs::PoseStamped goal_d1;
    kobuki_msgs::Sound sound;      

    KIVAToMTSA(ros::NodeHandle node_handle,ros::NodeHandle private_node_handle)
      :nh(node_handle),private_nh(private_node_handle)
    {
      ROS_INFO("ROSToMTSA constructer in");

      //initialize variables
      com=3, p_detect= 0, a_detect = 0;
      shelfx = 0;
      shelfy = 0;

      /*create socket for client*/
      sockfd = socket(AF_INET, SOCK_STREAM, 0);
      
      /*name the scket same as server socket*/
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = inet_addr("136.187.83.40");
      address.sin_port = htons(9999);
      len = sizeof(address);

      struct timeval tv;

      tv.tv_sec = 5;
      tv.tv_usec = 0;
      setsockopt(sockfd, SOL_SOCKET,SO_RCVTIMEO,(char *)&tv,sizeof(tv));

      /*connect client and server socket*/
      result = connect(sockfd, (struct sockaddr *)&address, len);
      if(result == -1) ERROR("oops : client1");

      ROS_INFO("goal initialize");

      //goal for demonstration
      //goal shelf
      goal_shelf.header.frame_id = "map";
      goal_shelf.pose.position.x = 0.0;
      goal_shelf.pose.position.y = -4.6;
      goal_shelf.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      
      //goal west
      goal_w.header.frame_id = "map";
      goal_w.pose.position.x = 0.0;
      goal_w.pose.position.y = 0.0;
      goal_w.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

      //goal middle
      goal_s0.header.frame_id = "map";
      goal_s0.pose.position.x = 0.0;
      goal_s0.pose.position.y = -2.3;
      goal_s0.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);      

      //goal middle
      goal_s1.header.frame_id = "map";
      goal_s1.pose.position.x = 0.0;
      goal_s1.pose.position.y = -2.3;
      goal_s1.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

      //goal dock no.0 仮
      goal_d0.header.frame_id = "map";
      goal_d0.pose.position.x = 0.0;
      goal_d0.pose.position.y = -2.3;
      goal_d0.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

     //goal dock no.1 仮
      goal_d1.header.frame_id = "map";
      goal_d1.pose.position.x = 0.0;
      goal_d1.pose.position.y = -2.3;
      goal_d1.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

      ROS_INFO("ROSToMTSA constructer out");
    }
    ~KIVAToMTSA(){}
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    
    void run();
  };
  
  
  void KIVAToMTSA::arrive_callback(const std_msgs::String msg)
  {
    //localization reasoning the topic published by "goal_arrive node"

    if(msg.data == "m"){
      //      ROS_INFO("Kobuki arrives at middle!");
      pos = 1;
    }else if(msg.data == "e"){
      //  ROS_INFO("Kobuki arrives at east!");
      pos = 2;
    }else if(msg.data == "w"){
      // ROS_INFO("Kobuki arrives at west!");
      pos = 0;
    }    
  }

  void KIVAToMTSA::bump_callback(const std_msgs::String msg)
  {
    //detect bumper was kicked reasoning the topic pubrished by "bump_to_mtsa"

    ROS_INFO("bump_callback in!");
    if(msg.data == "b"){
      ROS_INFO("bump!");
      ros::Rate r(1);
      r.sleep();
      sound.value= 6;
      sound_pub.publish(sound);
      p_detect   = 1;
      
    } 
        ROS_INFO("bump_callback out!");
  }

  void KIVAToMTSA::vel_callback(const geometry_msgs::Twist twist)
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
      if(vel == 0){
	vel = 1;
	ROS_INFO("1st stop");
      }else if (vel == 1){
	a_detect = 1;
	ROS_INFO("2nd stop");
      }
    }else{
      vel = 0;
    }
  }





  void KIVAToMTSA::subscriber_init(){
    ROS_INFO("subscriber_init");  
    arrive_sub = nh.subscribe("arrive",1000,&KIVAToMTSA::arrive_callback,this);
    bump_sub = nh.subscribe("bump",1000,&KIVAToMTSA::bump_callback,this);
    vel_sub = nh.subscribe<geometry_msgs::Twist>("/navigation_velocity_smoother/raw_cmd_vel",10,&KIVAToMTSA::vel_callback,this);
  }

  void KIVAToMTSA::publisher_init(){
    ROS_INFO("publisher_init");
    pickput_pub = nh.advertise<std_msgs::String>("chatter",1000);
    std::string goal_topic_name = "/move_base_simple/goal";
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_name.c_str(), 1);
    sound_pub = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound",1);
  }

  void KIVAToMTSA::arrive_detect(){
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



  void KIVAToMTSA::run(){
    ROS_INFO("run");
    publisher_init();
    subscriber_init();

    pos = 0;
    ros::Rate r(0.1);
    r.sleep();

      ROS_INFO("send: arrive w");
      write(sockfd, &com, 1);

      ROS_INFO("server waiting\n");
      /*become being able to weite or read thorough client_sockfd*/   
	read(sockfd, &com,1);
  
    while(1){
      a_detect = 0;p_detect = 0; vel = 0;
      ROS_INFO("command [%d] is received!", com);
      /*
	received command number is related to Controllable action
	1:move shelf
	2:move dock
	3;move station
	4:pick
	5:put
       */

      if(com == 2){
	ROS_INFO("command move dock is received!\n");
	// move dock;受け取ったら、行き先のdock番号をreceiveする
	read(sockfd, &com,1);
	//com: receive dock number
	if(com == 0){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_d0);
	  ROS_INFO("move dock no.0  was published");
	}else if(com == 1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_d1);
	  ROS_INFO("move dock no.1  was published");
	}
	  time = clock();
	  time_n = clock();
	  while(a_detect != 1){
	    ros::spinOnce();
	    time_n = clock();
	    c_time = time_n - time;
	    if(c_time >= 25000000){
	      break;
	    }
	    if(vel == 1){
	      ros::Rate r(0.5);
	      r.sleep();
	    }
	  }
	  arrive_detect();
      }else if(com == 1){
	//move shelf
	ROS_INFO("command move shelf is received!");
	read(sockfd, &shelfx,1);	
	read(sockfd, &shelfy,1);
	goal_shelf.pose.position.x = shelfx;
	goal_shelf.pose.position.y = shelfy;
	ros::Rate r(1); 
	r.sleep();
	goal_pub.publish(goal_shelf);
	ROS_INFO("move to shelf  was published");
	time = clock();
	time_n = clock();
	while(a_detect != 1){
	  ros::spinOnce();
	  time_n = clock();
	  c_time = time_n - time;
	  if(c_time >= 25000000){
	    ROS_INFO("detect pick fail");
	    break;
	  }
	  if(vel == 1){
	      ros::Rate r(0.5);
	      r.sleep();
	  }
	}
	arrive_detect();	     
      } else if(com == 3){
	ROS_INFO("command move Station is received!\n");
	// move station;受け取ったら、行き先のstation番号をreceiveする
	read(sockfd, &com,1);
	//com: receive station number
	if(com == 0){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_s0);
	  ROS_INFO("move station no.0  was published");
	}else if(com == 1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_d1);
	  ROS_INFO("move station no.1  was published");
	}
	  time = clock();
	  time_n = clock();
	  while(a_detect != 1){
	    ros::spinOnce();
	    time_n = clock();
	    c_time = time_n - time;
	    if(c_time >= 25000000){
	      break;
	    }
	    if(vel == 1){
	      ros::Rate r(0.5);
	      r.sleep();
	    }
	  }
	  arrive_detect();
      }else if(com == 4){
	//pickup
	ROS_INFO("command pick is received!");
	ros::Rate r(10);
	r.sleep();
	msgp.data = "pick";
	pickput_pub.publish(msgp);
	ROS_INFO("waiting pick succ/fail");
	time = clock();
	time_n = clock();
	while(p_detect == 0){
	  ros::spinOnce();
	  time_n = clock();
	  c_time = time_n - time;
	  if(c_time >= 10000000){
	    ROS_INFO("detect pick fail");
	    break;
	  }
	}
	if(p_detect == 1){
	  com = 4;
	  p_detect = 0;
	}else if(p_detect == 0){
	  com = 5;
	}
      }else if(com== 5){	  
	//putdown
	ROS_INFO("command putdown is received!");
	ros::Rate r(10);
	r.sleep();
	msgp.data = "put";
	pickput_pub.publish(msgp);
	ROS_INFO("waiting pick succ/fail;%d",p_detect);
	time = clock();
	time_n = clock();

	//for test
	//p_detect =1;
	  while(p_detect == 0){
	    ros::spinOnce();
	    time_n = clock();
	    c_time = time_n - time;
	    if(c_time >= 10000000){
	      ROS_INFO("detect pick fail");
	      break;
	    }
	  }
	if(p_detect == 1){
	  com = 6;
	  p_detect = 0;
	}else if(p_detect == 0){
	  com = 7;
	}       
      }
      
      if (com >= 1){
	ROS_INFO("send command to enactment:%d", com );
	//write command
	write(sockfd, &com, 1);
      }
      ROS_INFO("server waiting");
      /*become being able to write or read thorough client_sockfd*/ 

      com = 0;
      ROS_INFO("com b : %d",com);
      read(sockfd, &com,1);
      ROS_INFO("com a : %d",com);

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

    }
    ROS_INFO("closed");
    //    close(sockfd); 
  }
}

  
int main(int argc,char** argv){
  ros::init(argc,argv,"kiva_to_MTSA");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Kiva_to_Mtsa::KIVAToMTSA ktm(nh,nh_private);
  ktm.run();
  ros::spin();
}
