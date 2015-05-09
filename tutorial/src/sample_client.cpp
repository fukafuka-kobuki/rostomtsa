#include "ros/ros.h"
/* gcc -lsocket -lnsl -o inet_client inet_client.c */
#include  <sys/types.h>
#include  <sys/socket.h>
#include  <stdio.h>
#include  <netinet/in.h>
#include  <arpa/inet.h>
#include  <unistd.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>




int main(int argc, char **argv)
{
  ros::init(argc,argv,"sampleclient");
  ros::NodeHandle n;


   int server_sockfd, client_sockfd;
    char tmp = ' ';
   unsigned int server_len, client_len;
   struct sockaddr_in server_address;
      struct sockaddr_in client_address;

    /*サーバ用ソケット作成*/
    server_sockfd = socket(AF_INET, SOCK_STREAM, 0);

    /*ソケットに名前をつける(bind)*/
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_address.sin_port = 9999;
    server_len = sizeof(server_address);
    bind(server_sockfd, (struct sockaddr *) &server_address, server_len);

    /*接続キューを作成しクライアントからの接続を待つ*/
    while (1){
    listen(server_sockfd, 5);

     client_sockfd =
      accept(server_sockfd, (struct sockaddr *)&client_address, &client_len);  
     int com = 1;

    /*sockfdを介して読み書きができるようにする*/
    write(client_sockfd, &com, 2);
    read(client_sockfd, &com,2  );
    printf("char from server = %d\n", com);
    }
    close(client_sockfd);
    exit(0);
}
