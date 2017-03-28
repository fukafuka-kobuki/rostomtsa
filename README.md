# rostomtsa
# Turtlebot実装引き継ぎ資料
1.rosの導入と学習
基本的にはrosのホームページのtutorialを行えば良い。  
 http://wiki.ros.org/ja/hydro/Installation/Ubuntu  
OSはUbuntu12.04、rosバージョンはhydro推奨。

ROSの使い方はチュートリアル(http://wiki.ros.org/ja/ROS/Tutorials )の1, 2, 3, 4, 5, 6, 8, 9, 11,12, 13をやれば学習できると思います。それ以外は適宜必要になったらで良いと思います。

2. turtlebotの利用  

2.1turtlebotに必要な設定  
条件:1が終わっていること  

・turtlebotのドライバ設定  
$rosrun kobuki_ftdi create_udev_rules  
・turtlebotに必要なパッケージの導入  
$sudo apt-get install ros-hydro-turtlebot ros-hydro-turtlebot-apps  

2.2  親機と子機のネットワーク設定  
実験では、turtlebotに乗せるPC(子機)とそれを遠隔で操作するためのPC(親機)を使っていた。  
最悪必要ないが、これをしないとturtlebotにPCを載せたまま操作しなければならないので面倒。  

 Turtlebotを遠隔操作する側のPCを親機、Turtlebotに載せるPCを子機とする。  
・Wifiにつなぐ  
まずは両方共同じWifiにつなぐ。  
NII内ではSokenのみが使える。visitorはPortが開いていないため使えない。  
余力があるならローカルネットを自分で構築してしまうのが良いだろう。  
・IPを調べる  
親機と子機のIPを調べて、親機と子機の両方に  
/etc/hosts  
に以下のように書き加える。例えば、親機のIPが136.187.82.35、子機のIPが136.187.82.36であれば  
136.187.82.35 parentip  
136.187.82.36 childip  
のようにする。  
・ROSのネットワーク設定  
現状SSHで子機にログインすれば遠隔操作をすることができるが、それでは物足りない時が必ず来るはず。  
ROS上ではROSトピックなる情報のやり取りによって各ノード間で干渉し合うことができ、これをうまく使うことにより、  
親機の上でROSトピックを制御することで子機を動かすことができるようになる。そうするためには、多少の初期設定が  
必要となる。  
結論を言うと、以下のようにそれぞれの~/.bashrc（/home/katae/.bashrc）に書き加えるだけである  
親機側：　  
export ROS_MASTER_URI=http://parentip:11311  
export ROS_HOSTNAME=parentip   
子機側：  
export ROS_MASTER_URI=http://parentip:11311  
export ROS_HOSTNAME=childip  


3. rosの実装の導入  
実装したソースはgithub(https://github.com/fukafuka-kobuki)のrostomtsa内においてあるので、src内にこれを置けばmake可能。  
ただし、kivamtsaに関してはコンパイルできないのでrmして大丈夫  
（幾つかのPCで実装していた関係で別れたものなのでこれを残したままコンパイルするとコンフリクトします）  
 

4.実験方法  

4.1地図の作成  
1.roscoreを立ち上げる  

2.必要なノードを立ち上げる。以下のコマンドをそれぞれ別のターミナルで実行する  
・マップ表示rviz:   
　$rosrun rviz rviz  
・マップ取得gmap:   
　$roslaunch turtlebot_navigation gmapping_demo.launch  
・ turtlebot起動  
　$roslaunch turtlebot_bringup minimal.launch  
・turtlebot操作  
　$roslaunch turtlebot_teleop keyboard_teleop.launch  

3. teleopのターミナルをアクティブにし、turtlebotを動かす。  
rvizの画面上に動作する地図が描かれる。  

4.描画が完了したら、新しいターミナルを開いて  
$rosrun map_server map_saver –f /tmp/”my_map”  
とする(my_mapは任意の名前)  
これにより、地図情報が保存される。  

注意事項:  
　rvizで地図を作るとturtlebotの正面がx座標、横がy座標になる。  
　今回の実験ではy座標移動により倉庫内のエリアを移動するよう地図を作った。  

4.2座標の設定  
robotが目指すべき目的地座標が  
wエリア, mエリア,eエリアの三点決め、dest.txtを編集する  
座標に関しては、turtlebotが動いているときに別窓で  
$rostopic echo move_base  
とすれば速さや座標の情報を取れます。  

一行目:eエリアのy座標  
二行目:m エリアのy座標  
三行目:wエリアのy座標  

ここから、goal_arrive.cppとros_for_learnning.cppが目的地を決定する。  
ただ、これらの値は上で作ったmapに依存してy座標ベースでエリアを区切るかx座標ベースか変わったりするため、自分で中を書き換えたほうが楽に設定できるとも思う。  


4.3 実験  
1.roscoreを立ち上げる  
- ROSを導入したPCのターミナルで「roscore」とする  
2.必要なノードを立ち上げる  
roscoreを開いたのとは別にターミナルを立ち上げて、  
・turtlebotの起動  
　$roslaunch turtlebot_bringup minimal.launch  
・地図情報の呼び出し  
　＄roslaunch turtlebot_navigation amcl_demoo.launch map_file:=/home/usrname/map/”my_map”.yaml  
$rosrun mtsa ros_to_mtsa  
$rosrun mtsa bumper  
$rosrun mtsa goal_arrive  
とする  

3.MTSA側で利用したい制御器を生成  

4.[Enactment]->[Options]で利用するenactmentを選択  

5.[Enactment]->[Run model]を選択  

6.Controllerの遷移画面の出現、ROSとの通信開始  
※2において親機子機の設定を行っていた場合、mapファイルは親機側に保存されてしまった気がします。その場合はscpを使って親機から子機側にyamlファイルを転送する必要があります。  


5.dockerへの導入  
dockerを用いることで簡単に実験環境を整えることができないか試行錯誤した結果を簡単に。
$sudo docker run -it mkatae/turtleevalu:latest  
とすれば作ったdockerコンテナを持って来られます。  
ただ、turtlebotの実験を行うまでには至らず。  
理由：kinectにdockerコンテナ内からアクセスできなかった  
kinect for xboxは仮想環境からアクセスできないというのを見ました。  
真偽は定かではないが、自分ではこれは実現できなかった。  
もしかしたら方法はあるのかも。  

また、macのX11を使ってコンテナから窓を開こうとすると、rvizとの相性が悪いのか、エラーが出てしまう。  
結局ubuntuでしかrvizは開くことができなかったので、docker使うのも上の方法で環境整えるのも大した差はないのかと思う。  
仮に導入できても様々な障害が立ちふさがりそう。  






