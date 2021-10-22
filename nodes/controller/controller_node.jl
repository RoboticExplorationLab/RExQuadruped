module Controller 
    using Revise 
    import Mercury as Hg 
    import ZMQ 
    import TOML 

    include(joinpath(@__DIR__, "../../", "msgs", "messages_pb.jl"))
     mutable struct ControllerNode <: Hg.Node 
        ## Required by Abstract Node type 
        nodeio::Hg.NodeIO 
        rate::Float64
        should_finish::Bool

        ## Protobuf Messages 
        ekf::EKFMsg 
        encoders::JointSensorsMsg
        command::MotorCmdMsg
        function ControllerNode(ekf_sub_ip::String, ekf_sub_port::String, 
                                encoder_sub_ip::String, encoder_sub_port::String,
                                command_pub_ip::String, command_pub_port::String, rate)
            nodeio = Hg.NodeIO(ZMQ.Context(1); rate=rate)
            rate= rate; 
            ctx = ZMQ.Context(1)
            should_finish = false 

            # Message init 
            ekf = EKFMsg(pos=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         quat=QuaternionMsg(w=0.0, x=0.0, y=0.0, z=0.0),
                         v=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         v_ang=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         acc_bias=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         v_ang_bias=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         time=0.0)
            
            joint_message = JointMsg(FR_Hip=0.0, FR_Thigh=0.0, FR_Calf=0.0,
                                     FL_Hip=0.0, FL_Thigh=0.0, FL_Calf=0.0,
                                     RR_Hip=0.0, RR_Thigh=0.0, RR_Calf=0.0,
                                     RL_Hip=0.0, RL_Thigh=0.0, RL_Calf=0.0)

            encoders = JointSensorsMsg(torques=deepcopy(joint_message),
                                       positions=deepcopy(joint_message),
                                       velocities=deepcopy(joint_message),
                                       time=0.0)
            
            cmd_message = CmdMsg(Kp=0.0, Kd=0.0, pos=0.0, vel=0.0, tau=0.0)
            command = MotorCmdMsg(FR_Hip=deepcopy(cmd_message),
                                  FR_Thigh=deepcopy(cmd_message),
                                  FR_Calf=deepcopy(cmd_message),
                                  FL_Hip=deepcopy(cmd_message),
                                  FL_Thigh=deepcopy(cmd_message),
                                  FL_Calf=deepcopy(cmd_message), 
                                  RR_Hip=deepcopy(cmd_message),
                                  RR_Thigh=deepcopy(cmd_message),
                                  RR_Calf=deepcopy(cmd_message),
                                  RL_Hip=deepcopy(cmd_message),
                                  RL_Thigh=deepcopy(cmd_message),
                                  RL_Calf=deepcopy(cmd_message))
            
            # Publishers (command)
            command_pub = Hg.ZmqPublisher(nodeio.ctx, command_pub_ip, command_pub_port) 
            Hg.add_publisher!(nodeio, command, command_pub)

            # Subscriber (encoders, ekf)
            encoder_sub = Hg.ZmqSubscriber(nodeio.ctx, encoder_sub_ip, encoder_sub_port; name="ENC_SUB") 
            Hg.add_subscriber!(nodeio, encoders, encoder_sub)

            ekf_sub = Hg.ZmqSubscriber(nodeio.ctx, ekf_sub_ip, ekf_sub_port; name="EKF_SUB") 
            Hg.add_subscriber!(nodeio, ekf, ekf_sub)

            return new(nodeio, rate, should_finish, ekf, encoders, command, interface)
        end 

    end 

    function Hg.compute(node::ControllerNode)
        nodeio = Hg.getIO(node)


        Hg.publish.(nodeio.pubs)
    end 

    function main(; rate=100.0)
        topics_dict = TOML.tryparsefile("$(@__DIR__)/../../topics.toml")
        command_pub_ip = topics_dict["topics"]["commands"]["server"]
        encoder_sub_ip = topics_dict["topics"]["encoders"]["server"]
        ekf_sub_ip = topics_dict["topics"]["ekf"]["server"] 

        command_pub_port = topics_dict["topics"]["commands"]["port"]
        encoder_sub_port = topics_dict["topics"]["encoders"]["port"] 
        ekf_sub_port = topics_dict["topics"]["ekf"]["port"] 

        node = ControllerNode(ekf_sub_ip, ekf_sub_port,
                              encoder_sub_ip, encoder_sub_port,
                              command_pub_ip, command_pub_port, rate)
        return node 
    end 


end 