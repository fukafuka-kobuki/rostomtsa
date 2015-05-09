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

#define ERROR(x) {\
                  fprintf(stderr, "server - ");\
                  perror(x);\
                  exit(1);\
                 }

char pos[] = {'w'};
int arrive(void);
int pick_detect(void);
int put_detect(void);


void EventCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->state);
}



int main(int argc, char **argv)
{
  ros::init(argc,argv,"sample");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Publisher pickput_pub = nh.advertise<std_msgs::String>("chatter",1000);
  ros::Rate loop_rate(10);


  std_msgs::String msg;
  std::stringstream ss;

  int sockfd, len, result;
  struct sockaddr_in address;
  int com = 0;  /* argv[1][0]でも可能 */  /* argv[1][0]でも可能 */
  //       com[2] = 'c';  /* argv[1][0]でも可能 */
  
  printf("command [%d] is sent!\n",com);
  /*クライアント用ソケット作成*/
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  //         ros::spin();
  
  
  /*サーバ側と同じ名前でソケットの名前を指定*/
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = inet_addr("136.187.81.230");
    address.sin_port = htons(9999);
    len = sizeof(address);
    
    /*クライアントのソケットとサーバのソケットの接続*/
    result = connect(sockfd, (struct sockaddr *)&address, len);
    if(result == -1) ERROR("oops : client1");


    //目的地の設定
    ros::Publisher goal_pub;
    std::string goal_topic_name = "/move_base_simple/goal";
    goal_pub = n.advertise<geometry_msgs::PoseStamped>(goal_topic_name.c_str(), 1);
    
    //bumper
    ros::Subscriber bump;
    std::string bumper_topic_name = "/mobile_base/events/bumper";
    //    bump = n.subscribe(bumper_topic_name.c_str(),10,EventCallBack);

    //ゴール　東
    geometry_msgs::PoseStamped goal_e;
    goal_e.header.frame_id = "map";
    goal_e.pose.position.x = 0.1;
    goal_e.pose.position.y = 0.0;
    goal_e.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
 
    //ゴール　西
    geometry_msgs::PoseStamped goal_w;
    goal_w.header.frame_id = "map";
    goal_w.pose.position.x = 5.0;
    goal_w.pose.position.y = 0.0;
    goal_w.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    //ゴール　中間地点
    geometry_msgs::PoseStamped goal_m;
    goal_m.header.frame_id = "map";
    goal_m.pose.position.x = 3.0;
    goal_m.pose.position.y = 0.0;
    //    goal_m.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    goal_m.pose.orientation.x = 0.0;
    goal_m.pose.orientation.y = 0.0;  
    goal_m.pose.orientation.z = 0.0;
    goal_m.pose.orientation.w = 1.0;

    ros::Rate r(1);
    r.sleep();

    com = 3;
    write(sockfd, &com, 1);

 while(1){
   // 	int com =0;
        printf("Kobuki is at position %c!\n", pos[0]);
	printf("server waiting\n");


        /*接続を受け入れる*/
	//     client_sockfd =
	//  accept(server_sockfd, (struct sockaddr *)&client_address, &client_len);

        /*client_sockfdを介してクライアントに対する読み書きができるようになる*/
        read(sockfd, &com,1);
	printf("command [%d] is received!\n", com);
	com = com-47;
	printf("command [%d] is received!\n", com);



        if(com == 2){
	  ROS_INFO_ONCE("command move e is received!\n");
	  //東へ移動
	  if(pos[0] =='w'){
	      ros::Rate r(1);
	      r.sleep();

	      goal_pub.publish(goal_m);
	      ROS_INFO_ONCE("move e(go to m) was published");
	    //暫定的な到着の処理	    
	      //	    	    com = arrive();
	  }else if(pos[0] =='m'){
	      ros::Rate r(1);
	      r.sleep();
	      goal_pub.publish(goal_e);	    
	      ROS_INFO_ONCE("move e  (go to e)was published");
	    
	    //暫定的な到着処理
	  //	    	    com = arrive();
	  } 
	}else if(com == 1){
	  //西へ移動
	  printf("command move w is received!\n");
	  if(pos[0] == 'e'){
	      ros::Rate r(1);
	      r.sleep();
	      goal_pub.publish(goal_m);
	      ROS_INFO_ONCE("move w  was published");
	      
	    //暫定的な到着処理
	  }else if(pos[0] == 'm'){
	      ros::Rate r(1);
	      r.sleep();
	      goal_pub.publish(goal_w);
	      ROS_INFO_ONCE("move w  was published");
	      //暫定的な到着処理
	  }
	}else if(com== 3){
	  //荷物を持つ
	  ROS_INFO_ONCE("command pickup is received!\n");
	  ros::Rate r(10);
	  r.sleep();
	  ss<<"pick";
	  msg.data= ss.str();
	  pickput_pub.publish(msg);
	  ROS_INFO_ONCE("waiting pick succ/fail");
	}else if(com== 4){	  
	  //荷物を置く
	  ROS_INFO_ONCE("command putdown is received!\n");
	  ros::Rate r(10);
	  r.sleep();
	  ss<<"put";
	  msg.data= ss.str();
	  pickput_pub.publish(msg);
	  ROS_INFO_ONCE("waiting pick succ/fail");
	}

	//命令の書き込み
	//        write(sockfd, &com, 2);
	//        printf("Kobuki is at position %c\n",pos[0]);
	//	ROS_INFO_ONCE("Kobuki is at position %c", pos[0]);
	//	ROS_INFO_ONCE("command number %d is sent to MTSA!\n",com);
 }
        close(sockfd); 
}

char tmp;

int pick_detect(){
  ROS_INFO_ONCE("pick monitorable action fire!:");
  tmp = ' ';
  while(1){
    scanf("%c",&tmp);
    if(tmp == 's'){
      ROS_INFO_ONCE("pick up action was succeeded!\n");
      return 7;
    }else if(tmp == 'f'){
      ROS_INFO_ONCE("pickup action was failed!\n");
      return 8;
    }
  }
}

int put_detect(){
  ROS_INFO_ONCE("put monitorable action fire!:");
  tmp = ' ';
  while(1){
    scanf("%c",&tmp);
    if(tmp == 's'){
      ROS_INFO_ONCE("put up action was succeeded!\n");
      return 9;
    }else if(tmp == 'f'){
      ROS_INFO_ONCE("pickup action was failed!\n");
      return 10;
    }
  }
}


int arrive(){
  ROS_INFO_ONCE("Input Location of Kobuki!:");
  tmp = ' ';
  while(1){
    scanf("%c",&tmp);
    if(tmp == 'e'){
      pos[0] = 'e';
      ROS_INFO_ONCE("Kobuki is at position e!\n");
      return 4;
    }else if(tmp == 'm'){
      pos[0] = 'm';
      ROS_INFO_ONCE("Kobuki is at position m!\n");
      return 5;
    }else if(tmp == 'w'){
      pos[0] = 'w';
      ROS_INFO_ONCE("Kobuki is at position w!\n");
      return 6;
    }else if(tmp == '\n'||tmp =='\r'){
      continue;
    }else{
      break;
    }
  }
}
