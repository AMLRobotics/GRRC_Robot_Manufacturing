#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <fstream> 
#include <time.h>
#include <pthread.h>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#define ALARM_INTERVAL_IN_S (1)
#define WRITE_WAIT_INFINITE (unsigned long)(-1)
#define READ_WAIT_INFINITE  (unsigned long)(-1)

static unsigned int msgCounter = 0;
static int willExit = 0;

canHandle hnd;
canStatus stat;

ros::Publisher pub_vel;
ros::Publisher pub_pose;;

geometry_msgs::Twist vel;
geometry_msgs::Pose pose;

static void check(char* id, canStatus stat)
{
    if (stat != canOK) {
        char buf[100];
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
    }
}

static void sighand(int sig)
{
    static unsigned int last;

    switch (sig) {
    case SIGINT:
        willExit = 1;
        break;
    case SIGALRM:
        if (msgCounter - last) {
            printf("msg/s = %d, total=%u\n",
                (msgCounter - last) / ALARM_INTERVAL_IN_S, msgCounter);
        }
        last = msgCounter;
        alarm(ALARM_INTERVAL_IN_S);
        break;
    }
}

//RMD Motor Command List
u_int8_t READ_PID = 0x30;
u_int8_t WRITE_PID_RAM = 0x31;
u_int8_t WRITE_PID_ROM = 0x32;
u_int8_t READ_ACCEL = 0x42;
u_int8_t WRITE_ACCEL = 0x43;
u_int8_t READ_MULTITURN_POS = 0x60;
u_int8_t READ_ORIGINAL_POS = 0x61;
u_int8_t READ_MULTITURN_OFFSET = 0x62;
u_int8_t WRITE_ENCODER_ZERO = 0x63;
u_int8_t WRITE_ENCODER_CURRENT_POS_AS_ZERO = 0x64;
u_int8_t READ_MULTITURN_ANGLE = 0x92;
u_int8_t READ_MOTOR_STATUS1 = 0x9A;
u_int8_t READ_MOTOR_STATUS2 = 0x9C;
u_int8_t READ_MOTOR_STATUS3 = 0x9D;
u_int8_t SHUTDOWN = 0x80;
u_int8_t STOP = 0x81;
u_int8_t TORQUE_COMMAND = 0xA1;
u_int8_t SPEED_COMMAND = 0xA2;
u_int8_t ABS_POS_COMMAND = 0xA4;
u_int8_t POS_TRK_COMMAND = 0xA5;
u_int8_t REL_POS_COMMAND = 0xA8;
u_int8_t READ_OPERATING_MODE = 0x70;
u_int8_t READ_MOTOR_POWER = 0x71;
u_int8_t RESET = 0x76;
u_int8_t BRAKE_RELEASE = 0x77;
u_int8_t BRAKE_LOCK = 0x78;
u_int8_t CAN_ID_SETUP = 0x79;
u_int8_t READ_RUNTIME = 0xB1;
u_int8_t READ_SOFTWARE_VERSION = 0xB2;
u_int8_t COMM_INTERRUPT_TIMEOUT = 0xB3;

// VehicleCmd -----------------------------
struct CommandData {
    double linear_vel;
    double steering_ang;

    float motor_1;
    float motor_2;
    float motor_3;
    float motor_4;
    float motor_5;
    float motor_6;

    void reset();
};

void CommandData::reset()
{
    linear_vel = 0;
    steering_ang = 0;

    motor_1 = 0;
    motor_2 = 0;
    motor_3 = 0;
    motor_4 = 0;
    motor_5 = 0;
    motor_6 = 0;
}

static CommandData cmd_data; // command_data;

static void vehicleCmdCallback(const geometry_msgs::Twist twist)
{
    float R_omni = 0.160 / 2;
    float R_main = 0.1765 / 2; 
   

    // float Robot_W = 0.463 / 2;
    // float Robot_W = 0.451 / 2;
    float Robot_W = 0.51 / 2;

    // float max_vel = 2; // 2 m/s (≈ 7 km/h)
    float max_vel = 2;
    float max_ang_vel = 3.5;


    // cmd_data.linear_vel = twist.linear.x / 2;
    // cmd_data.steering_ang = twist.angular.z / 2;

    cmd_data.linear_vel = twist.linear.x;
    cmd_data.steering_ang = twist.angular.z;

    // cmd_data.linear_vel = (cmd_data.linear_vel) * max_vel;
    // cmd_data.steering_ang = (cmd_data.steering_ang) * max_ang_vel;
    // cmd_data.tilt_vel = (cmd_data.tilt_vel) * max_tilt_vel;

    // if (cmd_data.linear_vel > max_vel) cmd_data.linear_vel = max_vel;
    // else if (cmd_data.linear_vel < - max_vel) cmd_data.linear_vel = - max_vel;

    if (cmd_data.steering_ang > max_ang_vel)  cmd_data.steering_ang = max_ang_vel;
    else if (cmd_data.steering_ang < -max_ang_vel) cmd_data.steering_ang = -max_ang_vel;

    // robot_kinematics -----------------
    cmd_data.motor_1 = (1 / R_omni) * (-(Robot_W)*cmd_data.steering_ang + cmd_data.linear_vel);
    cmd_data.motor_3 = (1 / R_main) * (-(Robot_W)*cmd_data.steering_ang + cmd_data.linear_vel);
    cmd_data.motor_5 = (1 / R_omni) * (-(Robot_W)*cmd_data.steering_ang + cmd_data.linear_vel);

    cmd_data.motor_2 = -(1 / R_omni) * ((Robot_W)*cmd_data.steering_ang + cmd_data.linear_vel);
    cmd_data.motor_4 = -(1 / R_main) * ((Robot_W)*cmd_data.steering_ang + cmd_data.linear_vel);
    cmd_data.motor_6 = -(1 / R_omni) * ((Robot_W)*cmd_data.steering_ang + cmd_data.linear_vel);

    cmd_data.motor_1 = cmd_data.motor_1 * 180 / M_PI * 100 * 9;
    cmd_data.motor_2 = cmd_data.motor_2 * 180 / M_PI * 100 * 9;
    cmd_data.motor_3 = cmd_data.motor_3 * 180 / M_PI * 100 * 9;
    cmd_data.motor_4 = cmd_data.motor_4 * 180 / M_PI * 100 * 9;
    cmd_data.motor_5 = cmd_data.motor_5 * 180 / M_PI * 100 * 9;
    cmd_data.motor_6 = cmd_data.motor_6 * 180 / M_PI * 100 * 9;
}

void all_motor_stop(void)
{
    char msg_stop[8] = "";
    msg_stop[0] = 0xa2; //SPEED_COMMAND Velocity 0 = stop
    for (int i = 1; i < 8; i++)  msg_stop[i] = 0x00; //SPEED_COMMAND Velocity 0 = stop

    stat = canWriteWait(hnd, 0x141, msg_stop, 8, canMSG_STD, 5);
    stat = canWriteWait(hnd, 0x142, msg_stop, 8, canMSG_STD, 5);
    stat = canWriteWait(hnd, 0x143, msg_stop, 8, canMSG_STD, 5);
    stat = canWriteWait(hnd, 0x144, msg_stop, 8, canMSG_STD, 5);
    stat = canWriteWait(hnd, 0x145, msg_stop, 8, canMSG_STD, 5);
    stat = canWriteWait(hnd, 0x146, msg_stop, 8, canMSG_STD, 5);


    printf("all motor stop \n");
}

void can_msg_set2motor(uint8_t u8_msg_motor[], uint8_t cmd_type, long cmd_data = 0)
{
    u8_msg_motor[0] = cmd_type; //0xa2;  //Speed Closed-loop Control 
    u8_msg_motor[1] = 0;

    if (cmd_type == ABS_POS_COMMAND)//절대 위치 이동(0xA4)시 최대속도 지정
    { //0x01F4 = 500  0x1388 = 5000;
        u8_msg_motor[2] = 0x88; //0xF4;//Low bit
        u8_msg_motor[3] = 0x13; //0x01;//High bit
    }
    else
    {
        u8_msg_motor[2] = 0;
        u8_msg_motor[3] = 0;
    }

    u8_msg_motor[4] = cmd_data & 0xFF;
    u8_msg_motor[5] = (cmd_data >> 8) & 0xFF;
    u8_msg_motor[6] = (cmd_data >> 16) & 0xFF;
    u8_msg_motor[7] = (cmd_data >> 24) & 0xFF;
}

static void* senderCaller(void* unused)
{
    //char msg[8] = "";
    unsigned int dlc = 8;

    printf("motor begin \n");

    // ------ enter motor control mode ------------------------

    // motor ID
    long id_motor1 = 0x141; // left front
    long id_motor3 = 0x143; // left main
    long id_motor5 = 0x145; // left rear

    long id_motor2 = 0x142; // right front
    long id_motor4 = 0x144; // right main
    long id_motor6 = 0x146; // right rear

    //Quick msg
    char msg_motor_init[8] = "";
    char msg_motor_pose_init1[8] = "";
    char msg_motor_pose_init2[8] = "";
    char msg_motor_speed_0[8] = "";
    char msg_motor_get_pos[8] = "";

    char msg_motor_1[8] = "";
    char msg_motor_2[8] = "";
    char msg_motor_3[8] = "";
    char msg_motor_4[8] = "";
    char msg_motor_5[8] = "";
    char msg_motor_6[8] = "";

    uint8_t u8_msg_motor1[8];
    uint8_t u8_msg_motor2[8];
    uint8_t u8_msg_motor3[8];
    uint8_t u8_msg_motor4[8];
    uint8_t u8_msg_motor5[8];
    uint8_t u8_msg_motor6[8];

    // Motor Start
    msg_motor_init[0] = 0x88; // Old Version, No need.
    msg_motor_speed_0[0] = 0xa2; //SPEED_COMMAND Velocity 0 = stop
    msg_motor_get_pos[0] = 0x92; //0x60 : READ_MULTITURN_POS  0x92 : READ_MULTITURN_ANGLE
    for (int i = 1; i < 8; i++)
    {
        msg_motor_init[i] = 0x00; // Old Version, No need.
        msg_motor_speed_0[i] = 0x00; //SPEED_COMMAND Velocity 0 = stop
        msg_motor_get_pos[i] = 0x00;
    }

    stat = canWriteWait(hnd, id_motor1, msg_motor_init, dlc, canMSG_STD, 5);
    stat = canWriteWait(hnd, id_motor2, msg_motor_init, dlc, canMSG_STD, 5);
    stat = canWriteWait(hnd, id_motor3, msg_motor_init, dlc, canMSG_STD, 5);
    stat = canWriteWait(hnd, id_motor4, msg_motor_init, dlc, canMSG_STD, 5);
    stat = canWriteWait(hnd, id_motor5, msg_motor_init, dlc, canMSG_STD, 5);
    stat = canWriteWait(hnd, id_motor6, msg_motor_init, dlc, canMSG_STD, 5);

    // CAN sending ----------------------------
    // while ((stat == canOK) && !willExit) {
    while (ros::ok()) {

        // can msg set to motor motors -----------------------------------
        can_msg_set2motor(u8_msg_motor1, SPEED_COMMAND, (long)cmd_data.motor_1);
        can_msg_set2motor(u8_msg_motor2, SPEED_COMMAND, (long)cmd_data.motor_2);
        can_msg_set2motor(u8_msg_motor3, SPEED_COMMAND, (long)cmd_data.motor_3);
        can_msg_set2motor(u8_msg_motor4, SPEED_COMMAND, (long)cmd_data.motor_4);
        can_msg_set2motor(u8_msg_motor5, SPEED_COMMAND, (long)cmd_data.motor_5);
        can_msg_set2motor(u8_msg_motor6, SPEED_COMMAND, (long)cmd_data.motor_6);

        
        // Send can data -----------------------------------

        stat = canWriteWait(hnd, id_motor1, u8_msg_motor1, dlc, canMSG_STD, 5);
        stat = canWriteWait(hnd, id_motor2, u8_msg_motor2, dlc, canMSG_STD, 5);
        stat = canWriteWait(hnd, id_motor3, u8_msg_motor3, dlc, canMSG_STD, 5);
        stat = canWriteWait(hnd, id_motor4, u8_msg_motor4, dlc, canMSG_STD, 5);
        stat = canWriteWait(hnd, id_motor5, u8_msg_motor5, dlc, canMSG_STD, 5);
        stat = canWriteWait(hnd, id_motor6, u8_msg_motor6, dlc, canMSG_STD, 5);

        usleep(10000);
    }

ErrorExit:
    all_motor_stop();

    return nullptr;
}

///////////////////////////////
// simple low pass filter
double vel_right_smooth = 0;
double vel_left_smooth = 0;
// float LPF_Beta = 0.018; // 0<ß<1
float LPF_Beta = 0.1; // 0<ß<1
///////////////////////////////


void robot_vel_compute(float right_motor_v, float left_motor_v)
{
    // float R_main_wheel = 0.165 / 2;
    // float Robot_W = 0.463;

    float R_main_wheel = 0.1765 / 2;
    // float Robot_W = 0.5;

    float Robot_W = 0.451;


    // float Robot_titl = 0.01;

    right_motor_v = -right_motor_v * R_main_wheel;
    left_motor_v = left_motor_v * R_main_wheel;

    //Robot_titl = tilt_motor_v * gear;

    ///////////////////////////////
    // simple low pass filter - right
    // vel_right_smooth = vel_right_smooth - (LPF_Beta * (vel_right_smooth - right_motor_v));
    // right_motor_v = vel_right_smooth;

    if (abs(right_motor_v) < 0.008) {
        right_motor_v = 0;
    }

    // simple low pass filter - left
    // vel_left_smooth = vel_left_smooth - (LPF_Beta * (vel_left_smooth - left_motor_v));
    // left_motor_v = vel_left_smooth;

    if (abs(left_motor_v) < 0.008) {
        left_motor_v = 0;
    }
    ///////////////////////////////

    vel.linear.x = (right_motor_v + left_motor_v) / 2;
    // vel.linear.x = left_motor_v;
    // vel.linear.y = right_motor_v;

    vel.angular.z = (right_motor_v - left_motor_v) / Robot_W;

    // vel.linear.x = (Robot_tilt * 0.01);
    // printf("vel_x: %f \n", vel.linear.x);
    // printf("vel_th: %6f \n", vel.angular.z);
}

void robot_pose_compute(float right_motor_p, float left_motor_p, float right_motor_p_old, float left_motor_p_old)
{
    float R_main_wheel = 0.1765 / 2;
    float Robot_W = 0.451;

    float d_right = right_motor_p - right_motor_p_old;
    float d_left = left_motor_p - left_motor_p_old;

    // d_right = d_right * 2 * M_PI / 16384 / 9; 
    // d_left = d_left * 2 * M_PI / 16384 / 9;

    // 16 bit encoder
    d_right = d_right * 2 * M_PI / 65536 / 9;
    d_left = d_left * 2 * M_PI / 65536 / 9;

    d_right = -d_right * R_main_wheel;
    d_left = d_left * R_main_wheel;

    float center = (d_right + d_left) / 2;
    float th = (d_right - d_left) / Robot_W;

    float x = cos(th) * center;
    float y = -sin(th) * center;

    pose.position.x = pose.position.x + (cos(pose.orientation.z) * x - sin(pose.orientation.z) * y);
    pose.position.y = pose.position.y + (sin(pose.orientation.z) * x + cos(pose.orientation.z) * y);
    pose.orientation.z = pose.orientation.z + th;

    if (pose.orientation.z > M_PI) {
        pose.orientation.z -= 2 * M_PI;
    }
    else if (pose.orientation.z < -M_PI) {
        pose.orientation.z += 2 * M_PI;
    }

}

int CheckedID[20] = { -1, };
void ResetChkID()
{
    for (int i = 0; i < 20; i++) CheckedID[i] = -1;
}

bool IsFirstChkID(int id)
{
    bool ret = false;
    for (int i = 0; i < 20; i++)
    {
        if (CheckedID[i] == id) return false;
        else if (CheckedID[i] < 0)
        {
            CheckedID[i] = id;
            return true;
        }
    }
    return ret;
}

static void* receiveCaller(void* unused)
{
    pose.position.x = 0;
    pose.position.y = 0;
    pose.orientation.z = 0;

    int i = 0;

    int ret = -1;
    long id;
    unsigned int dlc;
    unsigned char msg[8];
    unsigned int flag;
    unsigned long t;

    uint8_t u8_msg_motor[8];

    // Motor variable
    int id_m;


    int t_int = 0;
    int p_int = 0;
    int16_t v_int = 0;
    int c_int = 0;

    int pose_get_L_flag = 0;
    int pose_get_R_flag = 0;

    int encoder_L_turn = 0;
    int encoder_R_turn = 0;

    float p = 0;
    float v = 0;
    float c = 0;

    float right_motor_p = 0;
    float right_motor_v = 0;
    float right_motor_i = 0;

    float left_motor_p = 0;
    float left_motor_v = 0;
    float left_motor_i = 0;

    float right_motor_p_old = 0;
    float left_motor_p_old = 0;

    float left_front_motor_i = 0;
    float left_rear_motor_i = 0;


    float right_front_motor_i = 0;
    float right_rear_motor_i = 0;

    float old_IMU_Pitch = 0.0;


    ResetChkID();
    // CAN sending ----------------------------
    // while ((stat == canOK) && !willExit) {
    while (ros::ok())
    {
        ret = canReadWait(hnd, &id, &msg, &dlc, &flag, &t, -1);
        switch (ret)
        {
        case 0:

            for (int i = 0; i < 8; i++) {
                u8_msg_motor[i] = (uint8_t)msg[i];
                // printf("%d", msg[i]);
            }

            id_m = id; // driver ID num
            if (IsFirstChkID(id_m)) printf("Checked id:%04X dlc:%d \n", id, dlc);

            t_int = u8_msg_motor[1]; // motor temperature
            c_int = (u8_msg_motor[2]) | u8_msg_motor[3] << 8; // current data
            v_int = (u8_msg_motor[4]) | u8_msg_motor[5] << 8; // velocity data
            p_int = (u8_msg_motor[6]) | u8_msg_motor[7] << 8; // position data

            // printf("velocity: %d \n", (int16_t)v_int);

            // printf("position: %d \n", (int16_t)p_int);

            // left main motor
            if (id_m == 0x143) {
                left_motor_p = (float)p_int;
                left_motor_v = (float)v_int * M_PI / 180 / 9;
                left_motor_i = c_int;
                //printf("position left: %d \n", (int16_t)p_int);
                //pose_get_L_flag = 1;
            }

            // right main motor
            if (id_m == 0x144) {
                right_motor_p = (float)p_int;
                right_motor_v = (float)v_int * M_PI / 180 / 9;
                right_motor_i = c_int;
                //printf("position right: %d \n", (int16_t)p_int);
                // pose_get_R_flag = 1;
            }

            //  left front motor
            if (id_m == 0x141) {
                left_front_motor_i = (float)c_int;
                //printf("position front left: %d \n", (int16_t)p_int);
            }

            //  right front motor
            if (id_m == 0x142) {
                right_front_motor_i = (float)c_int;
                //printf("position front right: %d \n", (int16_t)p_int);
            }

            //  left rear motor
            if (id_m == 0x145) {
                left_rear_motor_i = (float)c_int;
                //printf("position rear left: %d \n", (int16_t)p_int);
            }

            //  right rear motor
            if (id_m == 0x146) {
                right_rear_motor_i = (float)c_int;
                //printf("position rear right: %d \n", (int16_t)p_int);
            }

            ///////////////////////////////////////////////
            // Test --> Need to modify -------------------
            if (abs(right_motor_p_old - right_motor_p) > 10000) {
                right_motor_p_old = right_motor_p;
            }
            if (abs(left_motor_p_old - left_motor_p) > 10000) {
                left_motor_p_old = left_motor_p;
            }
            ///////////////////////////////////////////////

            robot_vel_compute(right_motor_v, left_motor_v);
            robot_pose_compute(right_motor_p, left_motor_p, right_motor_p_old, left_motor_p_old);

            right_motor_p_old = right_motor_p;
            left_motor_p_old = left_motor_p;

            break;

        case canERR_NOMSG:

            printf("canERR_NOMSG \n");
            break;

        default:
            perror("canReadBlock error");
            break;
        }

        // usleep(10000);
    }

    return nullptr;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sending_CAN");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, vehicleCmdCallback);

    pub_vel = n.advertise<geometry_msgs::Twist>("/act_vel", 10);
    pub_pose = n.advertise<geometry_msgs::Pose>("/act_pose", 10);

    /////////////////////////////////////////
    //CAN setting start----------------------

    // can channel
    static int channel = 0;

    signal(SIGALRM, sighand);
    signal(SIGINT, sighand);
    alarm(ALARM_INTERVAL_IN_S);

    /* Allow signals to interrupt syscalls */
    siginterrupt(SIGINT, 1);

    canInitializeLibrary();

    int chanCount = 0;
    stat = canGetNumberOfChannels(&chanCount);
    printf("Channel Count = %d \n", chanCount);

    for (int i = 0; i < chanCount; i++)
    {
        u_int32_t tmp;
        printf("== Channel %d ===============================\n", i);
        stat = canGetChannelData(i, canCHANNELDATA_CARD_TYPE, &tmp, sizeof(tmp));
        printf("cardtype =              0x%08lx\n", tmp);
    }

    /* Open channel, set parameters and go on bus */
    hnd = canOpenChannel(channel, canOPEN_EXCLUSIVE);
    if (hnd < 0)
    {
        printf("canOpenChannel %d \n", channel);
        return 0;
    }

    stat = canSetBusParams(hnd, canBITRATE_1M, 0, 0, 0, 0, 0);

    canSetBusOutputControl(hnd, canDRIVER_NORMAL);

    check("canSetBusParams", stat);

    stat = canBusOn(hnd);
    check("canBusOn", stat);

    //CAN setting end------------------------
    /////////////////////////////////////////

    pthread_t th[3];
    if (pthread_create(&th[0], nullptr, senderCaller, nullptr) != 0) {
        std::perror("pthread_create 0");
        std::exit(1);
    }

    if (pthread_create(&th[1], nullptr, receiveCaller, nullptr) != 0) {
        std::perror("pthread_create 1");
        std::exit(1);
    }

    if (pthread_detach(th[0]) != 0) {
        std::perror("pthread_detach 0");
        std::exit(1);
    }

    if (pthread_detach(th[1]) != 0) {
        std::perror("pthread_detach 1");
        std::exit(1);
    }

    ros::spin();
    return 0;
}