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
    void subscriber_init();
    void publisher_init();
    void arrive_callback(const std_msgs::String msg);
    void bump_callback(const std_msgs::String msg);
    void arrive_detect();
    void bump_detect();
    
  public:
    int sockfd, len, result;
    struct sockaddr_in address;
    int com;
    std_msgs::String msgp;
    std::stringstream ss;
    //    char pos[2];
    geometry_msgs::PoseStamped goal_e;
    geometry_msgs::PoseStamped goal_m;
    geometry_msgs::PoseStamped goal_w;
    int pos,p_detect;    

    ROSToMTSA(ros::NodeHandle node_handle,ros::NodeHandle private_node_handle)
      :nh(node_handle),private_nh(private_node_handle)
    {
      ROS_INFO("ROSToMTSA constructer in");
      com=0, p_detect   = 0;

      printf("command [%d] is sent!\n",com);
      /*クライアント用ソケット作成*/
      sockfd = socket(AF_INET, SOCK_STREAM, 0);
      
      /*サーバ側と同じ名前でソケットの名前を指定*/
      address.sin_family = AF_INET;
      address.sin_addr.s_addr = inet_addr("136.187.81.230");
      address.sin_port = htons(9999);
      len = sizeof(address);
      
      /*クライアントのソケットとサーバのソケットの接続*/
      result = connect(sockfd, (struct sockaddr *)&address, len);
      if(result == -1) ERROR("oops : client1");


      //        ROS_INFO("goal initialize");
      //ゴール　東
      goal_e.header.frame_id = "map";
      goal_e.pose.position.x = 5.0;
      goal_e.pose.position.y = 3.5;
      goal_e.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      
      //ゴール　西
      goal_w.header.frame_id = "map";
      goal_w.pose.position.x = 1.0;
      goal_w.pose.position.y = 0.0;
      goal_w.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

      //ゴール　中間地点
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
    //     ROS_INFO("arrive_callback in!;%s ",msg);

    //    if(msg->data.c_str() == "m"){
    //   ROS_INFO("middle!");
    //  pos = 1;
    // }else if(msg -> data.c_str() == "e"){
    //  ROS_INFO("east!");
    // pos = 2;
    // }else if(msg -> data.c_str() == "w"|| msg.data == "w"){
    // ROS_INFO("west!");
    // pos = 0;
    //}
    //  ros::Rate r(1);
    //r.sleep();
    if(msg.data == "m"){
      ROS_INFO("middle!");
      pos = 1;
    }else if(msg.data == "e"){
      ROS_INFO("east!");
      pos = 2;
    }else if(msg.data == "w"){
      ROS_INFO("west!");
      pos = 0;
    }
    //  ros::Rate r(1);
    //  r.sleep();
    
  }
  void ROSToMTSA::bump_callback(const std_msgs::String msg)
  {
    //    ROS_INFO("bump_callback in!;%s ",msg);
    if(msg.data == "b"){
      ROS_INFO("bump!");
      p_detect   = 1;
    }
    ros::Rate r(1);
    r.sleep();
  }


  void ROSToMTSA::subscriber_init(){
    ROS_INFO("subscriber_init");  
    arrive_sub = nh.subscribe("arrive",1000,&ROSToMTSA::arrive_callback,this);
    bump_sub = nh.subscribe("bump",1000,&ROSToMTSA::bump_callback,this);

  }

  void ROSToMTSA::publisher_init(){
    ROS_INFO("publisher_init");
    pickput_pub = nh.advertise<std_msgs::String>("chatter",1000);

    //目的地の設定
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

    com = 3;
    write(sockfd, &com, 1);
  
    while(1){
      //   printf("Kobuki is at position %d !\n", pos);
      //  printf("server waiting\n");

      /*client_sockfdを介してクライアントに対する読み書きができるようになる*/
      read(sockfd, &com,1);
            printf("command [%d] is received!\n", com);
      com = com-47;
      printf("command [%d] is received!\n", com);

      if(com == 2){
	ROS_INFO("command move e is received!\n");
	//東へ移動
	if(pos == 0){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_m);
	  ROS_INFO("move e(go to m) was published");
	  while(pos != 1){
	    ros::spinOnce();
	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();
	}else if(pos ==1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_e);	    
	  ROS_INFO("move e  (go to e)was published");
	  while(pos != 2){
	    ros::spinOnce();
	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();
	} else if(pos ==2){
	  ROS_INFO("kobuki is already at east");
	  com = 1;}
      }else if(com == 1){
	//西へ移動
	printf("command move w is received!\n");
	if(pos == 1){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_m);
	  ROS_INFO("move w  was published");
	  while(pos != 0){
	    ros::spinOnce();
	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();	     
	}else if(pos == 2){
	  ros::Rate r(1);
	  r.sleep();
	  goal_pub.publish(goal_w);
	  ROS_INFO("move w  was published");
	  while(pos != 1){
	    ros::spinOnce();
	    ROS_INFO("pos; %d", pos);
	  }
	  arrive_detect();
	} else if(pos ==0){
	  ROS_INFO("kobuki is already at east");
	  com = 3;
	}
      }else if(com== 3){
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
      //        printf("Kobuki is at position %c\n",pos[0]);
      //	ROS_INFO_ONCE("Kobuki is at position %c", pos[0]);
      //	ROS_INFO_ONCE("command number %d is sent to MTSA!\n",com);
    }
    close(sockfd); 
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
