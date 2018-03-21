#include "ros/ros.h"
#include <lcm/lcm-cpp.hpp>
#include <string.h>
#include <vector>
#include "lcm_msg/task_msg/input_pos_t.hpp"
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <demo_test/teleop_ctrl.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>

boost::mutex task_mutex;
boost::mutex leader_mutex;
//boost::mutex client_mutex;
//using namespace std;
using namespace ros;
struct task_msg_t {
    int64_t    timestamp;
    double     position[3];
};

//ServiceClient client = n.serviceClient<demo_test::teleop_ctrl>(srv_name_channel);
std::vector<ServiceClient> client_vector;
//Subscriber pos1_sub;// = n.subscribe("/mavros1/mocap/pose",1,pos1_cb);
std::vector <task_msg_t> pos_input;
int global_index = 0;
//string srv_name_base = "teleop_ctrl_service";
float formation_type[3][2] = {{0.0,0.0},{1.20,-1.20},{0.4,0.4}};
//float formation_type[3][2] = {{0.6,-0.6},{-1.04,-1.04},{0.4,0.4}};
float uav1_pos[3];
//int last_index1 = 0;
//int last_index2 = 1;
void uav_thread_function(int id);
void quad1()
{
    int uav_id = 0;
    uav_thread_function(uav_id);
}

void pos1_cb(const geometry_msgs::PoseStamped::ConstPtr &mocap_pos1) {
    boost::mutex::scoped_lock leader_lock(leader_mutex);
    uav1_pos[0] = mocap_pos1->pose.position.x;
    uav1_pos[1] = mocap_pos1->pose.position.y;
    uav1_pos[2] = mocap_pos1->pose.position.z;
}

void uav_thread_function(int id) {
    //takeoff
    //srv name 
    /*char * srv_name;
    srv_name = new char[sizeof("/teleop_ctrl_service")];
    memset(srv_name, 0, sizeof("/teleop_ctrl_service"));
    strcpy(srv_name,"/teleop_ctrl_service");
    //char * srv_id = itoa(id);
    char * srv_id;
    //itoa(id, srv_id, 1);
    sprintf(srv_id, "%d", id);
    char * srv_name_channel = strcat(srv_name,srv_id);*/
    // takeoffing
    //ServiceClient client = n.serviceClient<demo_test::teleop_ctrl>(srv_name_channel);
    demo_test::teleop_ctrl hover_pos;

    hover_pos.request.teleop_ctrl_mask = hover_pos.request.MASK_ARM_DISARM;
    hover_pos.request.base_contrl = hover_pos.request.ARM_TAKEOFF;

    //boost::mutex::scoped_lock client_lock(client_mutex);//
    client_vector[id].call(hover_pos);
        std::cout<<hover_pos.response.success<<std::endl;

    //client_lock.unlock();//

    sleep(5);
    //rate 10Hz
    ros::Rate rate(10);
    while (ros::ok()) {

    // task func  target_function
        boost::mutex::scoped_lock task_lock(task_mutex);
        bool tasking_flag = false;
        if (global_index < pos_input.size()) {
            hover_pos.request.teleop_ctrl_mask = hover_pos.request.MASK_HOVER_POS;
            hover_pos.request.hover_pos_x = pos_input[global_index].position[0];
            hover_pos.request.hover_pos_y = pos_input[global_index].position[1];
            hover_pos.request.hover_pos_z = pos_input[global_index].position[2];
            hover_pos.request.hover_pos_yaw = -1.57;
            global_index ++;
            tasking_flag = true;
        }
        task_lock.unlock();

        if (tasking_flag) {
            // srv set task posd

            //boost::mutex::scoped_lock client_lock(client_mutex);//
            //client_lock.lock();
            if (client_vector[id].call(hover_pos)) 
                std::cout << "target1 position:" << std::endl;
            //client_lock.unlock();//
            sleep(10.0f);
        }

    // formation func

        // get uav1 pos
        float forma_posd[3];

        //Subscriber pos1_pub = n.subscribe("/mavros1/mocap/pose",1,pos1_cb);

        boost::mutex::scoped_lock leader_lock(leader_mutex);
        forma_posd[0] = uav1_pos[0] + formation_type[0][id];
        forma_posd[1] = uav1_pos[1] + formation_type[1][id];
        forma_posd[2] = uav1_pos[2] + formation_type[2][id];
        leader_lock.unlock();
        // srv posd
        hover_pos.request.teleop_ctrl_mask = hover_pos.request.MASK_HOVER_POS;
        hover_pos.request.hover_pos_x = forma_posd[0];
        hover_pos.request.hover_pos_y = forma_posd[1];
        hover_pos.request.hover_pos_z = forma_posd[2];
        hover_pos.request.hover_pos_yaw = -1.57;

        //boost::mutex::scoped_lock client_lock(client_mutex);
        //client_lock.lock();
        client_vector[id].call(hover_pos);

        //std::cout << "mmpmmp" << std::endl;
        //client_lock.unlock();

        rate.sleep();
    }

}

void quad2()
{
    int uav_id = 1;
    uav_thread_function(uav_id);

}

class Task_Handler
{
    public:
        ~Task_Handler() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const task_msg::input_pos_t* msg)
                {
                    /* valid this task*/
                    bool task_valid = true;
                    //int search_index = pos_input.size();
                    double pos_record[3];
                    double value_distance;
                    pos_record[0] = msg->position[0];
                    pos_record[1] = msg->position[1];
                    pos_record[2] = msg->position[2];
                    //std::cout << "uav1_pos: " << pos_record[0] << "| " << pos_record[1] << "| " << pos_record[2] << std::endl;

                    //search task vector
                    boost::mutex::scoped_lock task_lock(task_mutex);
                    for (int i = 0; i < pos_input.size(); i++) {
                        if (fabsf(pos_record[0] - pos_input[i].position[0]) < 1.5f 
                            && fabsf(pos_record[1] - pos_input[i].position[1]) < 1.5f) {
                            task_valid = false;
                            pos_input[i].position[0] = (pos_input[i].position[0] + pos_record[0])/2; 
                            pos_input[i].position[1] = (pos_input[i].position[1] + pos_record[1])/2; 
                            pos_input[i].position[2] = (pos_input[i].position[2] + pos_record[2])/2; 
                            break;
                        }
                    }
                    task_lock.unlock();
                    /*
                    if (pos_input.empty())
                        task_valid = true;
                    else{
                        //for (int i = 0; i < search_index; i++) {
                        //pos_input(i).position
                        //i = search_index - 1;
                        //value_distance = sqrt(pow(pos_record[0] - pos_input(i).position[0]) + pow(pos_record[1] - pos_input(i).position[1]) + pow(pos_record[2] - pos_input(i).position[2]));
                        if (fabsf(pos_record[0] - pos_input(search_index-1).position[0]) > 1.5f 
                            || (pos_record[1] - pos_input(search_index-1).position[1]) > 1.5f) {
                            task_value = true;
                        }
                        else{
                            pos_input(search_index-1).position[0] = (pos_input[search_index-1].position[0] + pos_record[0])/2; 
                            pos_input(search_index-1).position[1] = (pos_input[search_index-1].position[1] + pos_record[1])/2; 
                            pos_input(search_index-1).position[2] = (pos_input[search_index-1].position[2] + pos_record[2])/2; 
                        }
                    } */


                    /* record the task*/
                    if (task_valid) {
                        task_lock.lock();
                        task_msg_t temp_record;
                        temp_record.timestamp = msg->timestamp;
                        temp_record.position[0] = msg->position[0];
                        temp_record.position[1] = msg->position[1];
                        temp_record.position[2] = msg->position[2];
                        pos_input.push_back(temp_record);
                        task_lock.unlock();
                    }
                }
};
void ros_func() {
    spin();
}

int main(int argc, char **argv)
{
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
    if(!lcm.good())
        return 1;
    init(argc,argv,"task_main");
    NodeHandle n;


    //lcm
    Task_Handler task_handler;
    lcm.subscribe("target_pos", &Task_Handler::handleMessage, &task_handler);
    //client

    ServiceClient client = n.serviceClient<demo_test::teleop_ctrl>("/teleop_ctrl_service2");
    client_vector.push_back(client);
    client = n.serviceClient<demo_test::teleop_ctrl>("/teleop_ctrl_service3");
    client_vector.push_back(client);
    std::cout << "client_size" << ": " << client_vector.size() << std::endl;

    //sub_uav1_position
    Subscriber pos1_sub = n.subscribe("/mavros1/mocap/pose",1,pos1_cb);

    boost::thread t_quad1(&quad1);
    boost::thread t_quad2(&quad2);
    boost::thread t_roshandle(&ros_func);

    ros::Rate _rate_main(10);
    
    while(ros::ok()) {
        std::cout << "running" << std::endl;
        for (int i = 0; i < pos_input.size(); i++) {
            std::cout << "task[" << i << "]: " << pos_input[i].position[0] << "|" << pos_input[i].position[1] << "|" << pos_input[i].position[2] << std::endl;
        }
        //spinOnce();
        lcm.handleTimeout(1000);
        _rate_main.sleep();
    }

    for (int i = 0; i < pos_input.size(); i++) {
        std::cout << "vector[" << i << "]: " << pos_input[i].position[0] << "|" << pos_input[i].position[1] << "|" << pos_input[i].position[2] << std::endl;
    }


    t_quad1.join();
    t_quad2.join();
    t_roshandle.join();

    return 0;
}
