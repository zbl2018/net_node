#include <sys/types.h>   
#include <sys/socket.h>   
#include <stdio.h>   
#include <stdlib.h>   
#include <string>   
#include <cstring>
#include <sys/ioctl.h>   
#include <unistd.h>   
#include <netdb.h>   
#include <netinet/in.h>     
#include <arpa/inet.h>     
#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <string>      

#include <ros/ros.h> 
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include <json/json.h>
#include <pthread.h>
#include <sstream>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <cmath>
#include <vector>  
#include <set> 
#include <list>
#include <algorithm>
#include <dirent.h>  
#include <pthread.h>
#include <loam_velodyne/pathPlan.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
//#include <loam_velodyne/GaussProjection.hpp>
#include <loam_velodyne/LocalGeographicCS.hpp>

using namespace std;
using namespace cv;

/// @brief      程序延时  
/// @param[in]  sec : 秒
struct Pointll
{
    double lat;
    double lon;
};

struct PointGS{
    double x;
    double y;
};

pthread_t id;
int i,ret;

pthread_t id2;
int i2,ret2;

pthread_t id3;
int i3,ret3;
//////////////////
int send_count = 0;
int stop_count = 0;
//////////////////
int gps_status = -1;

double last_lat,last_lon,last_height;

bool net_node_init = true;

double cmode,cangle,cspeed;
double cangle1,cangle2,cangle3,cangle4;
double ccur_m1,ccur_m2,ccur_m3,ccur_m4,cspeed_m1,cspeed_m2,cspeed_m3,cspeed_m4;
double cpul_m1,cpul_m2,cpul_m3,cpul_m4;
double cus1,cus2,cus3,cus4,cus5,cus6,cus7,cus8,cobstacle,cdis_keep;
double ccell[12];
double cmaxV,cminV,cmaxVP,cminVP,cVD,cAV,cTV,cCC,cDC,cSOC;
double cTem1,cTem2,cTem3,cTem4,cTem5,cTem6,cmaxTem,cminTem,cavrTem,cenvirTem;
double cRc,cWr;

bool is_stop = false;
int is_obstacle = 1;

bool is_last_car_state = true;//shenrk 网络节点中断重启后是否重新加载数据的标志位


ofstream path_log("/home/intel/Project/catkin_lidar/src/loam_velodyne/output/plan_log.txt");
ofstream record_sss("/home/intel/Project/catkin_lidar/src/loam_velodyne/output/record_sss.txt");
ofstream netnode_send_tast("/home/intel/Project/catkin_lidar/src/loam_velodyne/output/netnode_send_tast.txt");

static  double length_two_points(double x, double y, double xx,
        double yy) {
    double x_xx = x - xx;
    double y_yy = y - yy;

    return sqrt(x_xx * x_xx + y_yy * y_yy);
}

ros::Publisher Planer_net;
ros::Publisher Recv_net;
ros::Publisher Task_net;
ros::Publisher Move_net;
ros::Publisher stop_net;
ros::Publisher map_send;
ros::Publisher Charge_pub;

ros::Subscriber Location_net;
ros::Subscriber car_info_net;
ros::Subscriber gps_status_net;
ros::Subscriber sub_car_arrive;

ros::Subscriber sub_gps_rtk;

ros::Subscriber Obstacle_sub;

string lat,lon,heading;

// double lat_now = 5.1203;//x
// double lon_now = 2.8689;//y

double lat_now = 0;//x
double lon_now = 0;//y

double chargex = 0;
double chargey = 0.015;

// lat_now = 5.1203;//2.8689
// lon_now = 2.8689;//5.1203

double heading_now = 0;

double task_x  = 100,task_y = 100;
double last_task_x = 100,last_task_y = 100;
double net_speed = 0.2;

int is_recv,is_first = 0;
int length_read;

int is_car_arrvie = 0;

///////////////////////////////

//decoded from zbl on jan 21
//判断拐点 1:出现拐点
int is_car_arriveCrossing;
//订阅函数：订阅chmod话题：包含出现拐点信息
ros::Subscriber sub_car_cross;

//临时暂存拐点坐标
string cross_x,cross_y,cross_h;
//拐点计数
int count_CrossInfo=0;
bool charge_flag = false; //zqq 0203


///////////////////////////////

#define PORT 55556   
#define BUFFER_SIZE 2048  
#define NAME_SIZE 20  
const char * servInetAddr = "127.0.0.1";    
static int sockfd;
struct sockaddr_in serv_addr;

string plan_s;

char sendline[200]="";
string  serial = "";

vector<PointS> node;
vector<PointS> node2;
vector<PointS>cross_node2;
vector<PointS> tasknode;
vector<PointS> tasknode2;

String decodejson(char* s){ //decode the json from net by shenrks
    Json::Reader reader;
    Json::Value value;
    string js;
    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            js = value["action"].asString();
            serial = value["serial"].asString();
        }
    }
    return js;
}

String decodejson_serial(char* s){ //decode the json from net by shenrks
    Json::Reader reader;
    Json::Value value;
    string js;
    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            js = value["serial"].asString();
        }
    }
    return js;
}

void net_init(){
    /*创建socket*/   
 
    if ((sockfd = socket(AF_INET,SOCK_STREAM,0)) == -1)   
    {   
    perror("Socket failed!\n");   
    exit(1);   
    }   
    printf("Socket id = %d\n",sockfd);   
    /*设置sockaddr_in 结构体中相关参数*/   
    serv_addr.sin_family = AF_INET;   
    serv_addr.sin_port = htons(PORT);   
    inet_pton(AF_INET, servInetAddr, &serv_addr.sin_addr);    
    bzero(&(serv_addr.sin_zero), 8);   
    int  set = 1;
    setsockopt(sockfd, SOL_SOCKET,0, (void  *)&set, sizeof(int));
    /*调用connect 函数主动发起对服务器端的连接*/  
    cout<<"waiting for connect"<<endl; 
    if(connect(sockfd,(struct sockaddr *)&serv_addr, sizeof(serv_addr))== -1)   
    {   
    perror("Connect failed!\n");   
    exit(1);   
    }   
    printf("welcome\n"); 
}

//获取消息长度
int GetMsgLength(int sockfd){
    int recv_length = 0;
    unsigned char recvline[6] = {0};
    is_recv = recv(sockfd,recvline, 6, MSG_WAITALL);

    if(is_recv == -1)
    {
          cout<<"failed to recv msg"<<endl;
     }else{
          unsigned char recv_s[is_recv] = {0};
          int null_count = 0;
         
            for(int i=0;i<6;i++){
                if(recvline[i] != NULL){
                    //cout<<i<<" "<<recvline[i]<<endl;
                }
            }

            if(recvline[2] != NULL){
                recv_length += 256 * 256 * 256 * recvline[2];
                //cout<<"2:"<<recv_length<<endl;
            } 
            if(recvline[3] != NULL){
                recv_length += 256 * 256  * recvline[3];
                ///cout<<"3:"<<recv_length<<endl;
            }
            if(recvline[4] != NULL){
                recv_length += 256 * recvline[4];
                //cout<<"4:"<<recv_length<<endl;
            }
             if(recvline[5] != NULL){
                recv_length +=  recvline[5];
                //cout<<"5:"<<recv_length<<endl;
            }

      return recv_length;
     }
}

//转换为字节流，解决不同环境下编码方式的不同，产生乱码
unsigned char* intToBytes(int value) { //big end to little end
    unsigned char* src = new unsigned char[4];
    src[0] = (unsigned char) ((value >> 24) & 0xFF);
    src[1] = (unsigned char) ((value >> 16) & 0xFF);
    src[2] = (unsigned char) ((value >> 8) & 0xFF);
    src[3] = (unsigned char) (value & 0xFF);
    return src;
}
//decoded from zbl on jan 21
//deal with the information that prepares to send socket 
int deal_preSendInfo(string preInfo,char finInfo[], int &Infolen){
    
    int len = preInfo.length();
    unsigned char *s_len = (unsigned char*)&len;
   
    const char* ss_c = preInfo.c_str();
    char send_str[len+6] = {0};
    send_str[0] = NULL;
    send_str[1] = NULL;


    unsigned char* len_byte = intToBytes(len);
    for(int i=0;i<4;i++){
        send_str[i+2] = len_byte[i];
    }
    //复制转换数据
    for(int i=6;i<(len+6);i++){
        send_str[i] = ss_c[i-6];
    }
    send_str[len+6] = '\0';

    //转换完成
    finInfo=send_str;
    Infolen=len+6;
    return 1;
}
//解码move模式的json字符串
void decodeMove(char* s){
    Json::Reader reader;
    Json::Value value;
    //string js;
    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            //cout<<value["action"].asString()<<endl;
            time_t timep;  
            time(&timep);

            std_msgs::String msg;
            string msgs = value["mode"].asString().append(" ").append(value["angle"].asString()).append(" ").append(value["speed"].asString());
            msg.data = msgs;
            Move_net.publish(msg);
            cout<<msgs<<endl;
            printf("%s", asctime(gmtime(&timep))); 
        }
    }
} 
//解码任务点消息的json字符串
void decodeTask(char* s){
    Json::Reader reader;
    Json::Value value;

    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            //cout<<value["action"].asString()<<endl;
            std_msgs::String msg;

            cout<<value["x"].asString()<<" "<<value["y"].asString()<<endl;

            string msgs = value["mapNode"]["x"].asString().append(" ").append(value["mapNode"]["y"].asString()).append(" ").append(value["mapNode"]["speed"].asString());

            task_x = boost::lexical_cast<double>(value["mapNode"]["x"].asString());
            task_y = boost::lexical_cast<double>(value["mapNode"]["y"].asString());
            //net_speed = boost::lexical_cast<double>(value["mapNode"]["speed"].asString());//get the speed from the server

            serial = value["serial"].asString();

            string speed_rev = value["mapNode"]["speed"].asString();
            if(speed_rev.compare("") == 0)
            {
                net_speed = 0.4;
            }
            else
            {
                net_speed = boost::lexical_cast<double>(value["mapNode"]["speed"].asString());
            }

            msg.data = msgs;
            Task_net.publish(msg);
            cout<<msgs<<endl;


            //////////road plan//////////
            tasknode.clear();
            tasknode2.clear();
            
            PointS newpoint;//获取当前坐标加入规划
            if(net_node_init == true){
                newpoint.lat = 0;
                newpoint.lon = 0;
                tasknode.push_back(newpoint);

                net_node_init = false;
            }else{
                newpoint.lat = lat_now;
                newpoint.lon = lon_now;
                tasknode.push_back(newpoint);
            }

            newpoint.lat = task_x;
            newpoint.lon = task_y;
            tasknode.push_back(newpoint);

            pathPlan pa_tmp;
            if(is_obstacle)
            {
                tasknode2 = pa_tmp.calculate(tasknode);
            }else
            {
                tasknode2 = pa_tmp.obstacles(tasknode,heading_now);
                int cot = 5;
                while(cot--)
                {
                    cout << "obstacle replan!!!!!!!!!!!!!!" << endl;
                }
                is_obstacle = 1;
            }
            string path_send = pa_tmp.pathplan;//规划完成的导航路径
            //std_msgs::String msg;
            msg.data = path_send;
            map_send.publish(msg);

            /////////////////////////////

        }
    }
}

void decodeStop(int isstop){//发送stop指令
    std_msgs::Int8 msg;
    msg.data = isstop;
    stop_net.publish(msg);
}

void LocationHandler(const std_msgs::String::ConstPtr &msg){
    
    string moves = msg->data;
    stringstream stringin(moves);
    stringstream stringin2(moves);
    stringin>>lat>>lon>>heading;

    stringin2>>setprecision(12)>>lat_now>>lon_now>>heading_now;
   // cout<<"net location: "<<lat_now<<" "<<lon_now<<endl;
    //heading = heading * 180.0 / 3.14159265;
    
    //change by shenrk
    if(length_two_points(last_lat,last_lon,lat_now,lon_now) > 3.0 )
    {
        if(is_last_car_state == false)
        {
            lat_now = last_lat;
    	    lon_now = last_lon;
        }
    }

    last_lat = lat_now;
    last_lon = lon_now;
}   


//decoded from zbl on jan 21
//deal with subscription of "chmod" 
void Cross_Car_Handler(const std_msgs::String::ConstPtr &msg){
    string temp;//临时存储字符流中的mode部分
    stringstream datastream(msg->data);
    datastream>>cross_x>>cross_y>>cross_h>>temp;
    //将mode转换为int类型，下面用于判断是否为1：处于拐点位置
    is_car_arriveCrossing =boost::lexical_cast<int>(temp);
    cout<<"zbl:"<<cross_x<<" "<<cross_y<<" "<<cross_h<<" "<<temp<<" "<<is_car_arriveCrossing<<endl;
}

void Arrive_Car_Handler(const std_msgs::Int8::ConstPtr &msg){
	is_car_arrvie = msg->data;
	//cout<<"receive  arrived data from lidar_ctrl: "<<msg->data<<endl; 
	//cout<<"the car have arrived ang get the msg "<<is_car_arrvie<<endl;
}

void GPS_RTK_Handler(const sensor_msgs::NavSatFixConstPtr& gpslast){//will remove
    last_lat = gpslast->latitude;
    last_lon = gpslast->longitude;
    last_height = gpslast->altitude;
}

void GPS_STATUS_Handler(const std_msgs::Int64::ConstPtr &msg){//will remove
    gps_status = msg->data;
}

void InfoHandler(const std_msgs::String::ConstPtr &msg){
  string moves = msg->data;
  stringstream stringin(moves);
  //cout<<"mother fucker: "<<moves<<endl;
  //stringin>>o1>>o2>>o3>>o4>>a1>>a2>>a3>>a4>>sm1>>sm2>>sm3>>sm4;
  stringin>>cmode>>cangle>>cspeed;
  stringin>>cangle1>>cangle2>>cangle3>>cangle4;
  stringin>>ccur_m1>>ccur_m2>>ccur_m3>>ccur_m4>>cspeed_m1>>cspeed_m2>>cspeed_m3>>cspeed_m4;
  stringin>>cpul_m1>>cpul_m2>>cpul_m3>>cpul_m4;

  //cout<<cpul_m1<<" "<<cpul_m2<<" "<<cpul_m3<<" "<<cpul_m4<<endl;

  stringin>>cus1>>cus2>>cus3>>cus4>>cus5>>cus6>>cus7>>cus8>>cobstacle>>cdis_keep;
  stringin>>ccell[0]>>ccell[1]>>ccell[2]>>ccell[3]>>ccell[4]>>ccell[5]>>ccell[6]>>ccell[7]>>ccell[8]>>ccell[9]>>ccell[10]>>ccell[11];
  stringin>>cmaxV>>cminV>>cmaxVP>>cminVP>>cVD>>cAV>>cTV>>cCC>>cDC>>cSOC;
  stringin>>cTem1>>cTem2>>cTem3>>cTem4>>cTem5>>cTem6>>cmaxTem>>cminTem>>cavrTem>>cenvirTem;
  stringin>>cRc>>cWr;
  //cout<<"soc: "<<cSOC<<" "
} 

void Obstacle_plan_Handle(const std_msgs::Int8::ConstPtr &msg){
    is_obstacle = msg->data;
}//dyq obstacle

void Planner(char* s){

	is_car_arrvie = 0;

    Json::Reader reader;
    Json::Value value;
    string id,x,y,z;

    stringstream in_s;
    string s_,r;
    int is_one = 0;

    // lat_now = 5.1203;//2.8689
    // lon_now = 2.8689;//5.1203
    
    
    PointS newpoint;//获取当前坐标加入规划   zqq 
    if(net_node_init == true){
        newpoint.lat = 0;
        newpoint.lon = 0;
        node.push_back(newpoint);

        //net_node_init = false;
    }else{
        newpoint.lat = lat_now;
        newpoint.lon = lon_now;
        node.push_back(newpoint);
    }   

    cout<<"lat_now: "<<lat_now<<" lon_now: "<<lon_now<<endl;
    
    if(reader.parse(s,value)){

        int node_size = value["mapNodeList"].size();
        cout<<"node size:"<<node_size<<endl;
        for(int i=0;i<node_size;i++){
            id = value["mapNodeList"][i]["id"].asString();
            x = value["mapNodeList"][i]["x"].asString();
            y = value["mapNodeList"][i]["y"].asString();
            z = value["mapNodeList"][i]["z"].asString();

            if(is_one == 0){
                r.append(id);
                is_one = 1;
            }else{
                r.append(" ").append(id);
            }
            r.append(" ").append(x);
            r.append(" ").append(y);
            r.append(" ").append(z);

            double lat_ = strtod(x.c_str(),NULL);
            double lon_ = strtod(y.c_str(),NULL);

            newpoint.lat = lat_;
            newpoint.lon = lon_;
            node.push_back(newpoint);
            
        }


        //cout<<"task node:"<<r<<endl;
    }
}

void SendChargeMessage(string pathplan){
    std_msgs::Int8 charge_msg;
    charge_msg.data = 1;
    Charge_pub.publish(charge_msg);

    std_msgs::String charge_msg2;
    string tmp = (boost::lexical_cast<string>(chargex));
    tmp.append(" ").append(boost::lexical_cast<string>(chargey));
    tmp.append(" ").append(boost::lexical_cast<string>("-0.2"));
    charge_msg2.data = tmp;
    Task_net.publish(charge_msg2);

    tmp = pathplan;
    charge_msg2.data = tmp;
    map_send.publish(charge_msg2);

}

///////////////////////////////////////
//decoded from zbl on jan 21
//send CrossInfo to server
PointS search_cross_point_from_node2(double x,double y,vector<PointS> map,int &count){
//   double lat;
//  double lon;
//  int id;//start in 1
    vector<PointS>::iterator it;
    double min=99999;//初始较大
    double temp=0;
    int location=0;
    //查找一个与cross_x，cross_y相差最小的拐点
    for(it=map.begin(); it!=map.end(); it++){
        cout<<"node2:"<<it->lat<<" "<<it->lon<<endl;
       temp=fabs(x-it->lat)+fabs(y-it->lon);
      //temp=fabs((x+y)-(it->lat+it->lon));
    if(temp<min){
      min=temp;
      location=it-map.begin();
    }
   // cout<<*it<<" " ;
  }
  cout<<"result of searching :"<<(map.begin()+location)->lat<<endl;
  count=location+1;
  return *(map.begin()+location);
}
//decoded from zbl on jan 21
//send CrossInfo to server
void send_cross_info(int count){
    //构建json消息格式
    Json::FastWriter writer;
    Json::Value CrossInfo;
   // int len;
    char finInfo[1480];
    PointS crossPoint;
    double x=boost::lexical_cast<double>(cross_x);
    double y=boost::lexical_cast<double>(cross_y);
    //在node2中获取与当前接收到的拐点的位置cross_x,cross_y，最接近的拐点
    crossPoint=search_cross_point_from_node2(x,y,cross_node2,count);
    
    cout<<"search node2 successfully!"<<endl;
    CrossInfo["action"]="arriveCrossing";
    CrossInfo["serial"]="123456";
    //boost::lexical_cast
    CrossInfo["x"]=crossPoint.lat;
    CrossInfo["y"]=crossPoint.lon;
    CrossInfo["no"]=count;

    string preinfo =writer.write(CrossInfo);


    int len = preinfo.length();
    unsigned char *s_len = (unsigned char*)&len;
   
    const char* ss_c = preinfo.c_str();
    char send_str[len+6] = {0};
    send_str[0] = NULL;
    send_str[1] = NULL;


    unsigned char* len_byte = intToBytes(len);
    for(int i=0;i<4;i++){
        send_str[i+2] = len_byte[i];
    }
    for(int i=6;i<(len+6);i++){
        send_str[i] = ss_c[i-6];
    }
    send_str[len+6] = '\0';
    cout<<"send_str";
    for(int i=6;i<(len+6);i++){
        cout<<send_str[i];
    }
    cout<<endl;


    //  cout<<"finInfo:";
    // //将json转为socket可接受的数据格式，并且前6位为控制字段
    // //deal_preSendInfo(preinfo,finInfo,len);
    // cout<<"preinfo:"<<preinfo<<endl;
    // for(int i=0;i<len;i++)
    //     cout<<finInfo[i];
    // cout<<"deal info successfully!"<<endl;
    // //处理完json数据，加上前6位控制字段发送
   if(send(sockfd,send_str,len+6,0)<0)
     {  
        perror("send");  
        //exit(1);  
     } 
     else cout<<"have sent information of crosspoint successfully!"<<endl;
}

void sendTask_Arrived(){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;

    if(charge_flag)
    {
        map_data["action"] = "charge";
        map_data["result"] = "0000";
        map_data["serial"] = serial;
        map_data["msg"] = "charge";
        charge_flag = false;
    }
    else{
        map_data["action"] = "patrolArriveNode";
        map_data["result"] = "0000";
        map_data["serial"] = serial;
        map_data["msg"] = "patrolArriveNode";
    }

    string jstr = writer.write(map_data);

    //cout<<"josn:"<<jstr<<endl;

    int len = jstr.length();
    unsigned char *s_len = (unsigned char*)&len;
   
    const char* ss_c = jstr.c_str();
    char send_str[len+6] = {0};
    send_str[0] = NULL;
    send_str[1] = NULL;


    unsigned char* len_byte = intToBytes(len);
    for(int i=0;i<4;i++){
        send_str[i+2] = len_byte[i];
    }
    for(int i=6;i<(len+6);i++){
        send_str[i] = ss_c[i-6];
    }
    send_str[len+6] = '\0';
    cout<<"task:";
    for(int i=6;i<(len+6);i++){
        cout<<send_str[i];
    }
    cout<<endl;
    //send(sockfd,send_str,len+6,0);
    if(send(sockfd,send_str,len+6,0)<0)
     {  
        perror("send");  
        //exit(1);  
     } 
     else cout<<"have sent information of taskpoint successfully!"<<endl;

}

void send_Stop(){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;

    map_data["action"] = "stop";
    map_data["result"] = "0000";
    map_data["serial"] = serial;
    map_data["msg"] = "stop";

    string jstr = writer.write(map_data);

    //cout<<"josn:"<<jstr<<endl;

    int len = jstr.length();
    unsigned char *s_len = (unsigned char*)&len;
   
    const char* ss_c = jstr.c_str();
    char send_str[len+6] = {0};
    send_str[0] = NULL;
    send_str[1] = NULL;


    unsigned char* len_byte = intToBytes(len);
    for(int i=0;i<4;i++){
        send_str[i+2] = len_byte[i];
    }
    for(int i=6;i<(len+6);i++){
        send_str[i] = ss_c[i-6];
    }
    send_str[len+6] = '\0';
    send(sockfd,send_str,len+6,0);

    is_stop = false;

}

void sendPlanner(const char* s){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;

    Json::Reader reader;
    Json::Value value;

    cout<<"plan s:"<<s<<endl;




    if(reader.parse(s,value)){

        map_data["action"] = value["action"].asString();
        map_data["result"] = "0000";
        map_data["serial"] = serial;
        map_data["msg"] = value["action"].asString();

        //int node_size = value["mapNodeList"].size();
        

        int node_size = node.size();
        int node2_size = node2.size();
        int tag = 0;
        vector<int> task_visited;
        task_visited.resize(node_size);
        task_visited[0] = -1;
        for(int i = 1;i<node_size;i++)
        {
        	task_visited[i] = 1;
        }

        cout<<"node1: "<<node_size<<"  node2: "<<node2_size<<endl;
        for(int i=0;i<node2_size;i++)
        {
            for(int j=1;j<node.size();j++)
            {
                //if(length_two_points(node2[i].lat, node2[i].lon, node[j].lat, node[j].lon) <=0.3)
                //if(( node2[i].lat - node[j].lat < 0.00001 ) && ( node2[i].lon - node[j].lon < 0.00001 ) )
                if(( fabs(node2[i].lat - node[j].lat) < 0.1 ) && ( fabs(node2[i].lon - node[j].lon) < 0.1 ) && (task_visited[j] == 1) )
                {
                    tag = 1;
                    map_data["mapNodeList"][i]["id"] = value["mapNodeList"][j-1]["id"].asString();
                    map_data["mapNodeList"][i]["no"] = i+1;
                    map_data["mapNodeList"][i]["x"] = node2[i].lat;
                    map_data["mapNodeList"][i]["y"] = node2[i].lon;
                    task_visited[j] = -1;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
                    break;
                }
            }
            if(tag == 0)
            {
                map_data["mapNodeList"][i]["no"] = i+1;
                map_data["mapNodeList"][i]["x"] = node2[i].lat;
                map_data["mapNodeList"][i]["y"] = node2[i].lon; 
            }
            tag = 0;
        }

    }
    string jstr = writer.write(map_data);

    cout<<"plan josn:"<<jstr<<endl;

    path_log<<"plan josn:"<<jstr<<endl;
    path_log<<endl<<endl<<endl;

    int len = jstr.length();
    cout<<"len:"<<len<<endl;
    unsigned char *s_len = (unsigned char*)&len;
   
    const char* ss_c = jstr.c_str();
    char send_str[len+6] = {0};
    send_str[0] = NULL;
    send_str[1] = NULL;


    unsigned char* len_byte = intToBytes(len);
    for(int i=0;i<4;i++){
        send_str[i+2] = len_byte[i];
    }
   
    for(int i=6;i<(len+6);i++){
        send_str[i] = ss_c[i-6];
    }
    send_str[len+6] = '\0';

    send(sockfd,send_str,len+6,0);
    cout<<"plan json have send to server"<<endl;
}

void sendStatus(int sockfd){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;

    map_data["action"] = "RosRealState";
    map_data["content"]["direction"]["carDirection"] = heading;
    map_data["content"]["direction"]["flTire"] = cangle1;
    map_data["content"]["direction"]["frTire"] = cangle2;
    map_data["content"]["direction"]["rlTire"] = cangle3;
    map_data["content"]["direction"]["rrTire"] = cangle4;
    map_data["content"]["model"] = cmode;
    map_data["content"]["speed"]["carSpeed"] = cspeed;
    map_data["content"]["speed"]["flSpeed"] = cspeed_m1;
    map_data["content"]["speed"]["frSpeed"] = cspeed_m2;
    map_data["content"]["speed"]["rlSpeed"] = cspeed_m3;
    map_data["content"]["speed"]["rrSpeed"] = cspeed_m4;
    map_data["content"]["x"] = lat_now;
    map_data["content"]["y"] = lon_now;
    map_data["serial"] = "12345678";

    string jstr = writer.write(map_data);

    //cout<<jstr<<endl;

    int len = jstr.length();
    //cout<<jstr<<endl;
    //cout<<"len:"<<len<<endl;
    unsigned char *s_len = (unsigned char*)&len;
   
    const char* ss_c = jstr.c_str();
    char send_str[len+6] = {0};
    send_str[0] = NULL;
    send_str[1] = NULL;


    unsigned char* len_byte = intToBytes(len);
    for(int i=0;i<4;i++){
        send_str[i+2] = len_byte[i];
    }
   
    for(int i=6;i<(len+6);i++){
        send_str[i] = ss_c[i-6];
    }
    send_str[len+6] = '\0';

    send(sockfd,send_str,len+6,0);
}

void sendCarStatus(int sockfd){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;
    string powerTem;
    string powertmp;

    for(int i=0;i<12;i++)
    {
        if(i == 0)
        {
            powerTem.clear();
            powertmp.clear();
            powertmp = boost::lexical_cast<string>(ccell[0]);
            powerTem.append(powertmp);
        }
        else
        {
            powertmp.clear();
            powertmp = boost::lexical_cast<string>(ccell[i]);
            powerTem.append(",").append(powertmp);
        }
    }

    //int cSOC2 = boost::lexical_cast<int>(cSOC);
    map_data["action"] = "RosState";
    map_data["content"]["boxTemp"] = cenvirTem;
    map_data["content"]["infrared"] = 1;
    map_data["content"]["pavement"] = is_obstacle;
    map_data["content"]["power"] = cSOC;
    map_data["content"]["powerNode"] = powerTem;
    map_data["content"]["powerTemp"] = cmaxTem;
    map_data["content"]["stray"] = 1;
    map_data["content"]["ultrasonic"] = cobstacle;
    map_data["content"]["gpsStatus"] = gps_status;
    map_data["serial"] = "0986754";

    string jstr = writer.write(map_data);

    //cout<<jstr<<endl;

    int len = jstr.length();
    //cout<<"len:"<<len<<endl;
    //cout<<jstr<<endl;
    unsigned char *s_len = (unsigned char*)&len;
   
    const char* ss_c = jstr.c_str();
    char send_str[len+6] = {0};
    send_str[0] = NULL;
    send_str[1] = NULL;


    unsigned char* len_byte = intToBytes(len);
    for(int i=0;i<4;i++){
        send_str[i+2] = len_byte[i];
    }
   
    for(int i=6;i<(len+6);i++){
        send_str[i] = ss_c[i-6];
    }
    send_str[len+6] = '\0';

    if(cSOC > 5.0){
    	send(sockfd,send_str,len+6,0);
    }
}

void* getData(void* args){
    while(true){
        sendStatus(sockfd);
        //进程挂起进入就绪队列，以微妙为单位
        usleep(200000);
    }
}
void* sendData(void* args){
    while(true){
        sendCarStatus(sockfd);
        usleep(200000);
    }
}

void* recData(void* args){
    while(true){
        long recv_length = GetMsgLength(sockfd);//

                //cout<<recv_length<<endl;

                unsigned char recv_msg[recv_length+1] = {0};
                int msg = recv(sockfd,recv_msg,recv_length,MSG_WAITALL);
                char* sss = {0};
                recv_msg[recv_length+1] = '\0';
                sss = (char*)recv_msg;
                record_sss << ros::Time::now() << " " << sss << endl;

                //cout<<sss<<endl;
                serial.clear();

                string action_ = decodejson(sss);
                
                //cout<<"sss action:     "<<sss<<endl<<endl;

                if(action_.compare("buildMapStart") == 0){//构建地图
                    
                }else if(action_.compare("move") == 0){//遥控指令
                    decodeMove(sss);
                }else if(action_.compare("buildMapFinish") == 0){//构建完成

                }else if(action_.compare("buildMapSave") == 0){//上传地图数据

                    //sendFile(sockfd,"/home/shenrk/lidar_data/lidar_data_v3/path/path.txt","buildMapSave");

                }else if(action_.compare("deployRosParameter") == 0){//获取机器人坐标方位信息
                    cout<<"deployRosParameter"<<endl;
                    Json::Value root;
                    Json::FastWriter writer;
                    Json::Value map_data;

                    map_data["action"] = "deployRosParameter";
                    map_data["result"] = "0000";
                    map_data["serial"] = serial;
                    map_data["msg"] = "deployRosParameter";

                    double lx,ly;
                    //ReturnLocalPos(last_lat,last_lon,lx,ly);
                    // lx = lat_now;
                    // ly = lon_now;

                    //cout<<"now pos: "<<lx<<" "<<ly<<endl;
                    // map_data["x"] = lx;
                    // map_data["y"] = ly;

                    map_data["x"] = lat_now;
                    map_data["y"] = lon_now;

                    map_data["orientation"] = heading_now;


                    string jstr = writer.write(map_data);
                    cout<<jstr<<endl;
                    int len = jstr.length();
                    unsigned char *s_len = (unsigned char*)&len;
                    
                    const char* ss_c = jstr.c_str();
                    char send_str[len+6] = {0};
                    send_str[0] = NULL;
                    send_str[1] = NULL;

                    send_str[2] = NULL;
                    send_str[3] = NULL;

                    send_str[4] = NULL;
                    send_str[5] = (char)len;
                    //cout<<send_str[5]<<endl;
                    for(int i=6;i<(len+6);i++){
                        send_str[i] = ss_c[i-6];
                        //cout<<send_str;
                    }
                    cout<<endl;
                    send_str[len+6] = '\0';
                    
                    send(sockfd,send_str,len+6,0);

                }else if(action_.compare("patrolStartPathPlan") == 0){//开始路径规划
                    

                    send_count = 0;

                    //is_obstacle = 1;//dyy obstacle reset

                    cout<<"plan road "<<is_car_arrvie<<" "<<send_count<<endl;
                    
                    node.clear();
                    node2.clear();

                    pathPlan pa;
                    Planner(sss);
                    path_log<<"planner: "<<sss<<endl;
                    path_log<<"node: ";
                    for(int i=0;i<node.size();i++)
                    {
                        path_log<<setprecision(12)<<"("<<node[i].lat<<","<<node[i].lon<<");  ";
                    }
                    path_log<<endl;
                    if(is_obstacle)
                    {
                        node2 = pa.calculate(node);
                    }else
                    {
                        node2 = pa.obstacles(node,heading_now);
                    }

                    

                    cross_node2=node2;
                    //string path_send = pa.pathplan;//规划完成的导航路径


                    /////////////////////////
                    // std_msgs::String msg;
                    // msg.data = path_send;
                    // map_send.publish(msg);
                    /////////////////////////

                    path_log<<"node2: ";
                    for(int i=0;i<node2.size();i++)
                    {
                        path_log<<setprecision(12)<<"("<<node2[i].lat<<","<<node2[i].lon<<");  ";
                    }
                    path_log<<endl;
                    sendPlanner(sss);
                }else if(action_.compare("patrolArriveNode") == 0){//接收任务点并自动巡检
                    send_count = 0;
                    is_car_arrvie = 0;
                    stop_count = 0;

                    cout<<"patrolArriveNode "<<is_car_arrvie<<" "<<send_count<<endl<<endl;

                    decodeTask(sss);
                    decodeStop(0);

                    is_last_car_state = false;//shenrk change 3_20

                    if(length_two_points(task_x,task_y,lat_now,lon_now) < 0.4){//shenrk
                        is_car_arrvie = 1;
                    }
                    //usleep(500000);
                    //sendTask_Arrived();
                }else if(action_.compare("stop") == 0){//何工已确定json格式
                    decodeStop(1);
                    is_stop = true;
                    stop_count = 0;
                }else if(action_.compare("charge") == 0){

                    send_count = 0;
                    is_car_arrvie = 0;
                    stop_count = 0;

                    vector<PointS> ChargeNode;
                    PointS ChargeTask;
                    ChargeTask.lat = -0.213751;
                    ChargeTask.lon = 2.645041;
                    ChargeNode.push_back(ChargeTask);
                    ChargeTask.lat = chargex;
                    ChargeTask.lon = chargey;
                    ChargeNode.push_back(ChargeTask);

                    charge_flag = true;  //zqq

                    pathPlan pa;
                    vector<PointS> ChargeResult = pa.calculate(ChargeNode);
                    SendChargeMessage(pa.pathplan);
                }
    }
}

void sys_init(ros::NodeHandle nh){
    //Planer_net = nh.advertise<std_msgs::String>("/plan_net", 1000);
    //Recv_net = nh.advertise<std_msgs::String>("/recv_net",1000);
    Task_net = nh.advertise<std_msgs::String>("/task_net",1000);//任务点数据发布
    Move_net = nh.advertise<std_msgs::String>("/move_net",1000);//后台遥控指令发布
    stop_net = nh.advertise<std_msgs::Int8>("stop_net",100);

    Charge_pub = nh.advertise<std_msgs::Int8>("charge", 5);

    Location_net = nh.subscribe<std_msgs::String>("/final_data",1000,LocationHandler);
    car_info_net = nh.subscribe<std_msgs::String>("/car_info",100,InfoHandler);//接收底层数据

    map_send = nh.advertise<std_msgs::String>("/map_send",1000);//map send test

    //sub_gps_rtk = nh.subscribe<sensor_msgs::NavSatFix>("fix", 10,GPS_RTK_Handler);

    //gps_status_net = nh.subscribe<std_msgs::Int64>("gps_status",10,GPS_STATUS_Handler);

    sub_car_arrive = nh.subscribe<std_msgs::Int8>("car_arrive",5,Arrive_Car_Handler);

    Obstacle_sub = nh.subscribe<std_msgs::Int8>("/obstacle_plan",1,Obstacle_plan_Handle);

    ////////////////////////
    //decoded from zbl on jan 21
    sub_car_cross = nh.subscribe<std_msgs::String>("chmod",5,Cross_Car_Handler);
    /////////////////////////////
}
//change by shenrk 3_20
void Last_Car_State(double& last_x,double& last_y,bool& is_last){
    ifstream fin("/home/intel/Project/catkin_lidar/src/loam_velodyne/output/car_state_last.txt");
    string tmps;
    while(getline(fin,tmps))
    {
        double ltime,lx,ly,ntime,delta_time;
        if(tmps.compare("") == 0){
            stringstream ss(tmps);
            ss>>ltime>>lx>>ly;

            ntime = ros::Time::now();
            delta_time = fabs(ntime - ltime);

            if(delta_time > 6E5){
                last_x = lx;
                last_y = ly;

                is_last = true;
            }else{
                is_last = false;
            }
        }else{
            last_x = 0;
            last_y = 0;
            is_last = false;
        }
    }
    fin.close();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "Net_test");
    ros::NodeHandle nh;
    //cout<<"1"<<endl;
    //shenrk 3_20
    //节点启动时会读取该文件的位置信息，判断是否要采用上次的位置记录
    Last_Car_State(lat_now,lon_now,is_last_car_state);

    sys_init(nh);
    //cout<<"2"<<endl;
    net_init();

    ret = pthread_create(&id,NULL,getData,NULL);
    ret2 = pthread_create(&id2,NULL,sendData,NULL);
    ret3 = pthread_create(&id3,NULL,recData,NULL);

    
    


    ofstream car_state_last("/home/intel/Project/catkin_lidar/src/loam_velodyne/output/car_state_last.txt");//小车实时状态记录

    ros::Rate rate(150);
    bool status = ros::ok();
    int count_tmp = 0;
    while(status){
        ros::spinOnce();

        car_state_last<<setprecision(12)<<ros::Time::now()<<" "<<lat_now<<" "<<lon_now;

        if(is_stop == true && cspeed == 0 && stop_count < 1){
            cout<<"car have stop."<<endl;
            stop_count++;
            send_Stop();
            cout<<"car have stop. send"<<endl;
        }     

               /////////////////////////////////
        //decoded from zbl on jan 21
        //小车到达拐点,向服务后台发送数据
        if(is_car_arriveCrossing == 1)
        {
            //每次计数发送的拐点信息个数
            send_cross_info(count_CrossInfo);
            // if()
            //  cout<<"have sent information of crosspoint."<<endl;
            //  else cout<<"send crosspoint fail;"<<endl;
             //reset
             is_car_arriveCrossing=0;
        }
    /////////////////////////////////////////////   

        // if(count_tmp == 200)
        // {
        // 	cout<<"is car_arrive? "<<is_car_arrvie<<" send_count: "<<send_count <<" speed:"<<cspeed<<endl;
        // 	count_tmp = 0;
        // }
        // else
        // {
        // 	count_tmp++;
        // }//length_two_points(task_x,task_y,lat_now,lon_now) < 0.3

        //((is_car_arrvie == 1) && (cspeed < 0.01)) || (length_two_points(last_task_x,last_task_y,lat_now,lon_now) < 0.3)
        //((is_car_arrvie == 1) && (cspeed < 0.01))
        netnode_send_tast << ros::Time::now() << " is_car_arrvie: " << is_car_arrvie <<" send_count: " << send_count   //zqq 0319
        << " cmode: " << cmode << endl;


        if(((is_car_arrvie == 1) && (cspeed < 0.01))){//length_two_points(task_x,task_y,lat_now,lon_now) < 0.5
        	


            if(send_count < 1 && cmode == 0){

            	if(((is_car_arrvie == 1) && (cspeed < 0.01))){
	        		cout<<"send is_car_arrvie to net"<<endl<<endl;
	        	}

	        	// if((length_two_points(task_x,task_y,lat_now,lon_now) < 0.4)){
	        	// 	cout<<"length of two point less 0.2"<<endl<<endl;
	        	// }

            	cout<<"car have arrive the task point"<<endl;

	            send_count = 2;
	            
	            sendTask_Arrived();

	            cout<<"car have arrive the task point send "<<is_car_arrvie<<" "<<send_count<<endl;

	            last_task_x = task_x;
	            last_task_y = task_y;

	            task_x = 100;
	            task_y = 100;

            }
        }
        
        //cout<<"waiting msg......"<<endl;
        if(is_first == 0){
            string ss = "{\"action\":\"Regist\",\"deviceId\":\"ROS\"}";
            unsigned char s[120];
            memset(s,0,sizeof(unsigned char)*120);
            s[0] = NULL;
            s[1] = NULL;

            s[2] = NULL;
            s[3] = NULL;

            s[4] = NULL;
            s[5] = (char)ss.length();
            cout<<"REGIST:"<<ss.length()<<"  "<<s[5]<<endl;
            for(int i=0;i<=ss.length();i++){
                s[i+6] = ss[i];
            }
            cout<<s<<endl;
            send(sockfd,s,ss.length()+6,0);
            ss = "";
            is_first = 1; 
            cout<<"send successfully"<<endl;
        }
        status = ros::ok();
        rate.sleep();
    }

    car_state_last.close();
    return 0;  
}
