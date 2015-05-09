#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "kobuki_msgs/Sound.h"

namespace kobuki_control{
class KobukiSoundControl{
private:
  ros::Publisher kobuki_sound_pub;
  ros::Subscriber kobuki_sound_sub;
  int initial_sound_time;
    void subscriber_init();
    void publisher_init();
  public:
  KobukiSoundControl(ros::NodeHandle node_handle,ros::NodeHandle private_node_handle, int sound_time=0)
    : nh(node_handle), private_nh(private_node_handle), initial_sound_time(sound_time)
    {
      ROS_INFO("KobukiSoundControl");
      ROS_INFO("Initial Sound Time: %d", initial_sound_time);



    }
    ~KobukiSoundControl(){}
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    void run();




};

void KobukiSoundControl::subscriber_init(){

}

void KobukiSoundControl::publisher_init(){
 kobuki_sound_pub
   = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound",1);
 ros::Rate r(1);
 r.sleep();
}

  void KobukiSoundControl::run(){
    publisher_init();
   kobuki_msgs::Sound sound;
   sound.value = initial_sound_time;//これが音の種類を決める
   //詳細は以下
   // /opt/ros/hydro/share/kobuki_msgs/msg/Sound.msg
   kobuki_sound_pub.publish(sound);
  }

}
int main(int argc, char** argv){
  ros::init(argc, argv, "control_sound");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  int initial_time = atoi(argv[1]);
  kobuki_control::KobukiSoundControl ksc(nh, private_nh, initial_time);
  ksc.run();
  ros::spin();


}
