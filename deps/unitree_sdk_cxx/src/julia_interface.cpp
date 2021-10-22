#include <thread>
#include <array>
#include <iostream>
#include <map>
#include <cmath>
#include <string>
#include <memory>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "jlcxx/jlcxx.hpp"
#include "jlcxx/functions.hpp"
#include "jlcxx/stl.hpp"
using namespace UNITREE_LEGGED_SDK;

struct RobotInterface
{
public:
    RobotInterface() : safe(LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL) {
    // RobotInterface()  {
        // InitEnvironment();
        thread_ = std::thread(&RobotInterface::ReceiveObservation, this);
        udp.InitCmdData(cmd);
        InitSend();
    }
    ~RobotInterface() {
        destruct = true;
        thread_.join(); 
    }
    void ReceiveObservation();

    void InitSend();

    void SendCommand();


    // function for Julia
    int16_t getFootForce(int idx){
        return state.footForce[idx];
    }

    // need a number of functions to read motor state
    float getMotorStateQ(int idx) {
        return state.motorState[idx].q;
    }
    float getMotorStateDQ(int idx) {
        return state.motorState[idx].dq;
    }
    float getMotorStateDDQ(int idx) {
        return state.motorState[idx].ddq;
    }
    float getMotorStateTau(int idx) {
        return state.motorState[idx].tauEst;
    }

    // functions to set control cmd
    void setMotorCmd(int motor_id, double q, double dq, double Kp, double Kd, double tau) {
        cmd.motorCmd[motor_id].q = q;
        cmd.motorCmd[motor_id].dq = dq;
        cmd.motorCmd[motor_id].Kp = Kp;
        cmd.motorCmd[motor_id].Kd = Kd;
        cmd.motorCmd[motor_id].tau = tau;
    }

    // functions to get imu readings 
    void getAcceleration(jlcxx::ArrayRef<double> accel) {
        accel[0] = state.imu.accelerometer[0];
        accel[1] = state.imu.accelerometer[1];
        accel[2] = state.imu.accelerometer[2];
    }

    void getGyroscope(jlcxx::ArrayRef<double> gyro) {
        gyro[0] = state.imu.gyroscope[0];
        gyro[1] = state.imu.gyroscope[1];
        gyro[2] = state.imu.gyroscope[2];
    }
    void getQuaternion(jlcxx::ArrayRef<double> quat) {
        quat[0] = state.imu.quaternion[0];
        quat[1] = state.imu.quaternion[1];
        quat[2] = state.imu.quaternion[2];
        quat[3] = state.imu.quaternion[3];
    }
    void getEulerAngles(jlcxx::ArrayRef<double> euler) {
        euler[0] = state.imu.rpy[0];
        euler[1] = state.imu.rpy[1];
        euler[2] = state.imu.rpy[2];
    }


//    IMU getIMU() {
//        return state.imu;
//    }

    UDP udp;
    Safety safe;
    LowState state = {0};
    LowCmd cmd = {0};
    std::thread thread_;
    bool destruct = false;
};

void RobotInterface::ReceiveObservation() {
    while(destruct == false) {
        sleep(0.002);
        // std::cout << udp.targetIP << std::endl;
        udp.Recv();
        // std::cout << "receive" << std::endl;
        udp.GetRecv(state);
        // std::cout << state.footForce[0] << std::endl;
        // std::cout << state.motorState[0].q << std::endl;
        // std::cout << state.imu.accelerometer[0] << std::endl;
    };
}

void RobotInterface::InitSend() {
    InitEnvironment();
    cmd.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = PosStopF;        // 禁止位置环
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = VelStopF;        // 禁止速度环
        cmd.motorCmd[i].Kd = 0;
        cmd.motorCmd[i].tau = 0;
    }
    safe.PositionLimit(cmd);
    udp.SetSend(cmd);
    udp.Send();
}

void RobotInterface::SendCommand() {
    cmd.levelFlag = LOWLEVEL;
    // for (int motor_id = 0; motor_id < 12; motor_id++) {
    //     cmd.motorCmd[motor_id].mode = 0x0A;
    //     cmd.motorCmd[motor_id].q = motorcmd[motor_id * 5];
    //     cmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
    //     cmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
    //     cmd.motorCmd[motor_id].Kd = motorcmd[motor_id * 5 + 3];
    //     cmd.motorCmd[motor_id].tau = motorcmd[motor_id * 5 + 4];
    // }
    safe.PositionLimit(cmd);
    udp.SetSend(cmd);
    udp.Send();
}




std::string doc()
{
   return "A1 Robot Julia Bindings";
}

// struct MutableBits
// {
//   double a;
//   double b;
// };


// struct World
// {
//   World(const std::string& message = "default hello") : msg(message){}
//   void set(const std::string& msg) { this->msg = msg; }
//   std::string greet() { return msg; }
//   std::string msg;
//   ~World() { std::cout << "Destroying World with message " << msg << std::endl; }
// };

namespace jlcxx
{
  //template<> struct IsMirroredType<IMU> : std::true_type { };
//   template<> struct IsMirroredType<MotorState> : std::true_type { };
//   template<> struct IsMirroredType<MotorCmd> : std::true_type { };
//   template<> struct IsMirroredType<LowState> : std::false_type { };
//   template<> struct IsMirroredType<LowCmd> : std::true_type { };
}

JLCXX_MODULE define_julia_module(jlcxx::Module& mod)
{
    struct Array { Array() {} double a;};
    mod.method("doc", &doc);
    // map several types
    // mod.map_type<MutableBits>("MutableBits");
    // mod.map_type<Cartesian>("Cartesian");
    // mod.map_type<IMU>("IMU");
    // mod.map_type<LED>("LED");
    // mod.map_type<MotorState>("MotorState");
    // mod.map_type<MotorCmd>("MotorCmd");
    // mod.map_type<LowState>("LowState");
    // mod.map_type<LowCmd>("LowCmd");


    mod.add_type<RobotInterface>("RobotInterface")
    .method("getFootForce", &RobotInterface::getFootForce)
    .method("getMotorStateQ", &RobotInterface::getMotorStateQ)
    .method("getMotorStateDQ", &RobotInterface::getMotorStateDQ)
    .method("getMotorStateDDQ", &RobotInterface::getMotorStateDDQ)
    .method("getMotorStateTau", &RobotInterface::getMotorStateTau)
    .method("InitSend", &RobotInterface::InitSend)
    .method("SendCommand", &RobotInterface::SendCommand)
    .method("getAcceleration", &RobotInterface::getAcceleration)
    .method("getGyroscope", &RobotInterface::getGyroscope)
    .method("getEulerAngles", &RobotInterface::getEulerAngles)
    .method("getQuaternion", &RobotInterface::getQuaternion) 
    .method("setMotorCmd", &RobotInterface::setMotorCmd);
    // .method("ReceiveObservation", &RobotInterface::ReceiveObservation);
    // .method("ReceiveObservation", &RobotInterface::ReceiveObservation);
    // .method("SendCommand", &RobotInterface::SendCommand);
    // mod.add_type<World>("World")
    // .constructor<const std::string&>()
    // .method("set", &World::set)
    // .method("greet", &World::greet);
}