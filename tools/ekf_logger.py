import os, sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import zmq 
import time 
import toml 
from docopt import docopt
from pymercury import messaging
import msgs.messages_pb2 as messages
import google.protobuf.message_factory as message_factory
import numpy as np 


def main():
    host_ip = "192.168.3.123"
    port = "5004"

    ctx = zmq.Context() 
    ekf_sub = messaging.create_sub(ctx, port, host=host_ip)
    command_sub = messaging.create_sub(ctx, "5003", host=host_ip)
    encoders_sub = messaging.create_sub(ctx, "5002", host=host_ip)
    imu_sub = messaging.create_sub(ctx, "5001", host=host_ip)

    poller = zmq.Poller() 
    poller.register(ekf_sub, zmq.POLLIN)
    poller.register(command_sub, zmq.POLLIN)
    poller.register(encoders_sub, zmq.POLLIN)
    poller.register(imu_sub, zmq.POLLIN)

    msg = messages.LeggedEKFMsg()
    command_msg = messages.MotorCmdMsg()
    encoder_msg = messages.JointSensorsMsg()
    imu_msg = messages.ImuMsg()
    t = time.time()
    
    state = np.zeros((1,32))    #[r, quat, v, p1, p2, p3, p4, acc_bias, v_ang]
    commands = np.zeros((1,13)) #[tau, time]
    encoders = np.zeros((1,37)) #[q, v, tau, time]
    imus = np.zeros((1,7))      #[acc, v_ang, time]
    f_ekf=open('tools/data/ekf.dat','wb')
    f_command=open("tools/data/command.dat", "wb")
    f_encoders=open("tools/data/encoders.dat", "wb")
    f_imu=open("tools/data/imu.dat", "wb")

    try:
        while True: 
            socks = dict(poller.poll())
            if ekf_sub in socks.keys() and socks[ekf_sub] == zmq.POLLIN:
                data = ekf_sub.recv(zmq.DONTWAIT)
                msg.ParseFromString(data)
                state[0,0:3] = [msg.pos.x, msg.pos.y, msg.pos.z]
                state[0,3:7] = [msg.quat.w, msg.quat.x, msg.quat.y, msg.quat.z]
                state[0,7:10] = [msg.v.x, msg.v.y, msg.v.z]
                state[0,10:13] = [msg.p1.x, msg.p1.y, msg.p1.z]
                state[0,13:16] = [msg.p2.x, msg.p2.y, msg.p2.z]
                state[0,16:19] = [msg.p3.x, msg.p3.y, msg.p3.z] 
                state[0,19:22] = [msg.p4.x, msg.p4.y, msg.p4.z]  
                state[0,22:25] = [msg.acc_bias.x, msg.acc_bias.y, msg.acc_bias.z]   
                state[0,25:28] = [msg.v_ang_bias.x, msg.v_ang_bias.y, msg.v_ang_bias.z]    
                state[0,28:31] = [msg.v_ang.x, msg.v_ang.y, msg.v_ang.z] 
                state[0,-1] = msg.time
                np.savetxt(f_ekf,state, fmt='%.18f')

            if command_sub in socks.keys() and socks[command_sub] == zmq.POLLIN:
                data = command_sub.recv(zmq.DONTWAIT)
                command_msg.ParseFromString(data)
                commands[0,:12] = [command_msg.FR_Hip.tau, command_msg.FR_Thigh.tau, command_msg.FR_Calf.tau,
                                   command_msg.FL_Hip.tau, command_msg.FL_Thigh.tau, command_msg.FL_Calf.tau,
                                   command_msg.RR_Hip.tau, command_msg.RR_Thigh.tau, command_msg.FL_Calf.tau, 
                                   command_msg.RL_Hip.tau, command_msg.RL_Thigh.tau, command_msg.RL_Calf.tau]
                commands[0,:12] = command_msg.time 
                np.savetxt(f_command, commands, fmt='%.18f')
            
            if encoders_sub in socks.keys() and socks[encoders_sub] == zmq.POLLIN: 
                data = encoders_sub.recv(zmq.DONTWAIT)
                encoder_msg.ParseFromString(data)
                encoders[0,:12] = [encoder_msg.positions.FR_Hip, encoder_msg.positions.FR_Thigh, encoder_msg.positions.FR_Calf,
                                   encoder_msg.positions.FL_Hip, encoder_msg.positions.FL_Thigh, encoder_msg.positions.FL_Calf,
                                   encoder_msg.positions.RR_Hip, encoder_msg.positions.RR_Thigh, encoder_msg.positions.RR_Calf,
                                   encoder_msg.positions.RL_Hip, encoder_msg.positions.RL_Thigh, encoder_msg.positions.RL_Calf]
                encoders[0,12:24] = [encoder_msg.velocities.FR_Hip, encoder_msg.velocities.FR_Thigh, encoder_msg.velocities.FR_Calf,
                                     encoder_msg.velocities.FL_Hip, encoder_msg.velocities.FL_Thigh, encoder_msg.velocities.FL_Calf,
                                     encoder_msg.velocities.RR_Hip, encoder_msg.velocities.RR_Thigh, encoder_msg.velocities.RR_Calf,
                                     encoder_msg.velocities.RL_Hip, encoder_msg.velocities.RL_Thigh, encoder_msg.velocities.RL_Calf]
                encoders[0,24:36] = [encoder_msg.tau.FR_Hip, encoder_msg.tau.FR_Thigh, encoder_msg.tau.FR_Calf,
                                     encoder_msg.tau.FL_Hip, encoder_msg.tau.FL_Thigh, encoder_msg.tau.FL_Calf,
                                     encoder_msg.tau.RR_Hip, encoder_msg.tau.RR_Thigh, encoder_msg.tau.RR_Calf,
                                     encoder_msg.tau.RL_Hip, encoder_msg.tau.RL_Thigh, encoder_msg.tau.RL_Calf]
                encoders[0,36] = encoder_msg.time 
                np.savetxt(f_encoders, encoders, fmt="%.18f")

            if imu_sub in socks.keys() and socks[imu_sub] == zmq.POLLIN: 
                data = imu_sub.recv(zmq.DONTWAIT)
                imu_msg.ParseFromString(data)

                imus[0,:6] = [imu_msg.acc.x, imu_msg.acc.y, imu_msg.acc.z, imu_msg.v_ang.x, imu_msg.v_ang.y, imu_msg.v_ang.z]
                imus[0,6] = imu_msg.time 
                np.savetxt(f_imu, imus, fmt="%.18f")
                


    except KeyboardInterrupt:
        f_encoders.close()
        f_ekf.close() 
        f_command.close() 
        f_imu.close()
        print("Interrupted")
             

if __name__ == "__main__":
    main()
