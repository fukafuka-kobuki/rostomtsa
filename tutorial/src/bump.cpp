
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
#include "kobuki_msgs/Sound.h"
#include "std_msgs/String.h"

#define ERROR(x) {				\
    fprintf(stderr, "server - ");		\
    perror(x);					\
    exit(1);					\
  }


namespace ROS_to_MTSA{
  class BumperToMTSA{
  private:
    ros::Subscriber bumper_sub;
    ros::Subscriber pickput_sub;
    ros::Publisher sound_pub;
    ros::Publisher bump_pub;
    void bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr msg);
    void pickput_callback(const std_msgs::String msg);
    void subscriber_init();
    void publisher_init();
  public:
    int com;  /* argv[1][0]でも可能 */  /* argv[1][0]でも可能 */
    int pickput_detect;
    std_msgs::String bump;
    kobuki_msgs::Sound sound;

    BumperToMTSA(ros::NodeHandle node_handle,ros::NodeHandle private_node_handle )
      : nh(node_handle), private_nh(private_node_handle)
    {
      ROS_INFO("BumperToMTSA");
      pickput_detect = 0;
    }
    ~BumperToMTSA(){}
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    void run();


  };
  void BumperToMTSA::pickput_callback(const std_msgs::String msg)
  {
    if(msg.data == "pick"){
      ROS_INFO("pickup!");
      //sound publish for kobuki
      ros::Rate r(1);
      r.sleep();
      sound.value= 5;
      sound_pub.publish(sound);
      pickput_detect = 1;

    }else if(msg.data == "put"){
      ROS_INFO("putdown!");
      //sound publish for kobuki
      ros::Rate r(1);
      r.sleep();
      sound.value= 0;
      sound_pub.publish(sound);
      pickput_detect = 2;
    }
  }

  void BumperToMTSA::bumper_callback(const kobuki_msgs::BumperEvent::ConstPtr msg){
    ROS_INFO("bumper_callback in");
    //MTSA側に渡す処理
    //sound publish for kobuki
    ros::Rate r(1);
    r.sleep();
    sound.value= 6;
        sound_pub.publish(sound);
 
    //bumper happened is announced for ROS_TO_MTSA
    ROS_INFO("bumper happened is announced for ROS_TO_MTSA");
    bump.data = "b";
    bump_pub.publish(bump);
    
    ROS_INFO("bumper_callback out");
  }
  void BumperToMTSA::subscriber_init(){
    ROS_INFO("subscriber_init");
    std::string bumper_topic_name = "/mobile_base/events/bumper";
    bumper_sub = nh.subscribe<kobuki_msgs::BumperEvent::ConstPtr>(bumper_topic_name,10, &BumperToMTSA::bumper_callback, this);

    pickput_sub = nh.subscribe("chatter",1000,&BumperToMTSA::pickput_callback,this);
    //    ros::spin();

  }

  void BumperToMTSA::publisher_init(){
    ROS_INFO("publisher_init");
    sound_pub
      = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1);
    bump_pub = nh.advertise<std_msgs::String>("bump",1000);

    //    ros::Rate r(1);
    //r.sleep();
  }

  void BumperToMTSA::run(){
    ROS_INFO("run");
    subscriber_init();
    publisher_init();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "bumper_to_MTSA");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ROS_to_MTSA::BumperToMTSA btm(nh, nh_private);
  //  ROS_to_MTSA::BumperToMTSA publisher_init();
  btm.run();
  ros::spin();
  

}
