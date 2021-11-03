import os, sys 
import os, sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import zmq 
import time 
from pymercury import messaging 
import msgs.messages_pb2 as messages

def main(): 
    ctx = zmq.Context()
    pub = messaging.create_pub(ctx, "5001")

    # legged_msg = messages.JointSensorsMsg()
    # legged_msg.velocities.SetInParent()
    # legged_msg.torques.SetInParent() 
    # legged_msg.positions.SetInParent() 

    imu_msg = messages.ImuMsg()
    imu_msg.acc.SetInParent()
    imu_msg.gyro.SetInParent()
    while True:

        imu_msg.time = time.time()
        data = imu_msg.SerializeToString() 
        pub.send(data)
        time.sleep(0.1)

if __name__ == "__main__":
    main()