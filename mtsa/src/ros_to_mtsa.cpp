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



namespace Ros_to_Mtsa{
  class ROSToMTSA{
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
    int sockfd, len, result, pos,pos_b,p_detect,a_detect,com,vel;
    clock_t time,time_n,c_time;
    struct sockaddr_in address;

    std_msgs::String msgp;
    std::stringstream ss;
    geometry_msgs::PoseStamped goal_e;
    geometry_msgs::PoseStamped goal_m;
    geometry_msgs::PoseStamped goal_w;
    kobuki_msgs::Sound sound;      

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
      //goal east
      goal_e.header.frame_id = "map";
      goal_e.pose.position.x = 0.0;
      goal_e.pose.position.y = -4.6;
      goal_e.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      
      //goal west
      goal_w.header.frame_id = "map";
      goal_w.pose.position.x = 0.0;
      goal_w.pose.position.y = 0.0;
      goal_w.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

      //goal middle
      goal_m.header.frame_id = "map";
      goal_m.pose.position.x = 0.0;
      goal_m.pose.position.y = -2.3;
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

  void ROSToMTSA::bump_callback(const std_msgs::String msg)
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
    sound_pub = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound",1);
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
    ros::Rate r(0.1);
    r.sleep();

    //    com = 3;
    // write(sockfd, &com, 1);
      ROS_INFO("send: arrive w");
      write(sockfd, &com, 1);

      //     ros::Rate beg(0.15);
      // beg.sleep();

      ROS_INFO("server waiting\n");
      /*become being able to weite or read thorough client_sockfd*/ 
  
	read(sockfd, &com,1);


  
    while(1){
      //   printf("Kobuki is at position %d !\n", pos);
      a_detect = 0;
      p_detect = 0;
      vel = 0;
      ROS_INFO("command [%d] is received!", com);
      //      com = com-48;
      // ROS_INFO("command [%d] is received!", com);

      /*
	received command number is related to Controllable action
	1:move w
	2:move e
	3:pickup
	4:putdown
       */

      if(com == 2){
	ROS_INFO("command move e is received!\n");
	pos_b = pos;
	//receive move e
	if(pos == 0){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_m);
	  ROS_INFO("move e(go to m) was publishe");
	  ROS_INFO("a_detect = %d, pos = %d, pos_b = %d",a_detect, pos_b,pos);
	  //	  while(a_detect != 1 || pos_b == pos){
	  //ros::spinOnce();
	    //	    ROS_INFO("pos; %d", pos);
	  //}
	  time = clock();
	  time_n = clock();

	  //for offline test
	  //a_detect = 1;
	  //pos = 1;

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
	}else if(pos ==1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_e);	    
	  ROS_INFO("move e  (go to e)was pubished");
	  ROS_INFO("a_detect = %d, pos = %d, pos_b = %d",a_detect, pos_b,pos);
	  time = clock();
	  time_n = clock();

	  //for offline test
	  //a_detect = 1;
	  //pos = 2;


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
	} else if(pos ==2){
	  ROS_INFO("kobuki is already at east");
	  com = 1;
	  ros::Rate r(10);
	  r.sleep();
	}
      }else if(com == 1){
	//move w
	ROS_INFO("command move w is received!");
	pos_b = pos;
	if(pos == 1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_w);
	  ROS_INFO("move w  was published");
	  ROS_INFO("a_detect = %d, pos = %d, pos_b = %d",a_detect, pos_b,pos);
	  // while(a_detect != 1 || pos_b == pos){
	    // ros::spinOnce();
	    //	    ROS_INFO("pos; %d", pos);
	    //	  }
	  time = clock();
	  time_n = clock();

	  //for offline test
	  //a_detect = 1;
	  //pos = 0;


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
	}else if(pos == 2){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_m);
	  ROS_INFO("move w  was published");
	  ROS_INFO("a_detect = %d, pos = %d, pos_b = %d",a_detect, pos_b,pos);
	  //	  while(a_detect != 1 || pos_b == pos){
	  //  ros::spinOnce();
	    //	    ROS_INFO("pos; %d", pos);
	  // }
	  time = clock();
	  time_n = clock();

	  //for offline test
	  //a_detect = 1;
	  //pos = 1;


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
	} else if(pos ==0){
	  ROS_INFO("kobuki is already at west");
	  com = 3;
	  ros::Rate r(10);
	  r.sleep();
	}
      }else if(com == 3){
	//pickup
	ROS_INFO("command pickup is received!");
	ros::Rate r(10);
	r.sleep();
	msgp.data = "pick";
	pickput_pub.publish(msgp);
	ROS_INFO("waiting pick succ/fail");
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
	  com = 4;
	  p_detect = 0;
	}else if(p_detect == 0){
	  com = 5;
	}
      }else if(com== 4){	  
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
       
      }else if(com == 5){
	//pick red
	ROS_INFO("command pick red is received!");
	ros::Rate r(10);
	r.sleep();
	msgp.data = "red";
	pickput_pub.publish(msgp);
	ROS_INFO("waiting pick red succ/fail;%d",p_detect);
	time = clock();
	time_n = clock();

	//for test
	//p_detect =1;
	  while(p_detect == 0){
	    ros::spinOnce();
	    time_n = clock();
	    c_time = time_n - time;
	    if(c_time >= 10000000){
	      ROS_INFO("detect pick red  fail");
	      break;
	    }
	  }
	if(p_detect == 1){
	  com = 8;
	  p_detect = 0;
	}else if(p_detect == 0){
	  com = 9;
	}
      }else if(com == 6){
	//pick yellow
	ROS_INFO("command pick yellow is received!");
	ros::Rate r(10);
	r.sleep();
	msgp.data = "yellow";
	pickput_pub.publish(msgp);
	ROS_INFO("waiting pick yellow succ/fail;%d",p_detect);
	time = clock();
	time_n = clock();

	//for test
	//p_detect =1;
	  while(p_detect == 0){
	    ros::spinOnce();
	    time_n = clock();
	    c_time = time_n - time;
	    if(c_time >= 10000000){
	      ROS_INFO("detect pick yellow fail");
	      break;
	    }
	  }
	if(p_detect == 1){
	  com = 8;
	  p_detect = 0;
	}else if(p_detect == 0){
	  com = 9;
	}
      }

      //      com =2;
      if (com >= 1){
	ROS_INFO("send command to enactment:%d", com );
	//write command
	write(sockfd, &com, 1);
      }
      //write(sockfd, &com, 1);
      //ROS_INFO("command number %d is sent to MTSA!",com);
      ROS_INFO("server waiting");
      /*become being able to write or read thorough client_sockfd*/ 


      //ros::Rate r(1);
      //r.sleep();
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
  ros::init(argc,argv,"ros_to_MTSA");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Ros_to_Mtsa::ROSToMTSA rtm(nh,nh_private);
  rtm.run();
  ros::spin();
}
