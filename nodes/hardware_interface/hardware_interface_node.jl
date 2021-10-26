module HardwareInterface 
    using Revise 
    import Mercury as Hg 
    import ZMQ 
    import TOML

    include(joinpath(@__DIR__, "../../", "msgs", "messages_pb.jl"))
    include(joinpath(@__DIR__, "julia_robot_interface.jl"))

    mutable struct HardwareInterfaceNode <: Hg.Node 
        ## Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        ## ProtoBuf Messages 
        imu::ImuMsg
        encoders::JointSensorsMsg 
        command::MotorCmdMsg

        # Robot interface 
        interface::A1Robot.RobotInterfaceAllocated
        acceleration_imu::Vector{Float64}
        gyroscope_imu::Vector{Float64}
        function HardwareInterfaceNode(imu_pub_ip::String, imu_pub_port::String,
                                       encoder_pub_ip::String, encoder_pub_port::String,
                                       command_sub_ip::String, command_sub_port::String,
                                       rate::Float64)
            nodeio = Hg.NodeIO(ZMQ.Context(1); rate=rate)            
            rate = rate; 
            ctx = ZMQ.Context(1)
            should_finish = false 
            
            # Message definitions 
            imu = ImuMsg(acc = Vector3Msg(x=0.0, y=0.0, z=0.0),
                         gyro = Vector3Msg(x=0.0, y=0.0, z=0.0),
                         time = 0.0)
            
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
            

            # Publishers (imu, encoders)                     
            imu_pub = Hg.ZmqPublisher(nodeio.ctx, imu_pub_ip, imu_pub_port) 
            Hg.add_publisher!(nodeio, imu, imu_pub)

            encoder_pub = Hg.ZmqPublisher(nodeio.ctx, encoder_pub_ip, encoder_pub_port)
            Hg.add_publisher!(nodeio, encoders, encoder_pub)

            # Subscriber (command)
            cmd_sub = Hg.ZmqSubscriber(nodeio.ctx, command_sub_ip, command_sub_port; name="CMD_SUB") 
            Hg.add_subscriber!(nodeio, command, cmd_sub)
            
            # initialize interface 
            interface = A1Robot.RobotInterface() 
            A1Robot.InitSend(interface)
            acceleration_imu = zeros(Float64, 3)
            gyroscope_imu = zeros(Float64, 3)

            return new(nodeio, rate, should_finish, imu, encoders, command, interface, acceleration_imu, gyroscope_imu)
        end 
    end 

    function Hg.compute(node::HardwareInterfaceNode)
        nodeio = Hg.getIO(node)
        
        ## Setting publisher fields 
        A1Robot.getAcceleration(node.interface, node.acceleration_imu)
        A1Robot.getGyroscope(node.interface, node.gyroscope_imu)
        node.imu.acc.x, node.imu.acc.y, node.imu.acc.z = node.acceleration_imu
        node.imu.gyro.x, node.imu.gyro.y, node.imu.gyro.z = node.gyroscope_imu

        qs, dqs, _, τs = A1Robot.getMotorReadings(node.interface)
        for (i, field) in enumerate(propertynames(node.encoders.positions)[1:12])
            setproperty!(node.encoders.positions, field, qs[i])
            setproperty!(node.encoders.velocities, field, dqs[i])
            setproperty!(node.encoders.torques, field, τs[i])
        end 
        node.encoders.time = time() 

        ## Do things base on subscriber 
        cmd_sub = Hg.getsubscriber(node, "CMD_SUB")
        
        Hg.on_new(cmd_sub) do command 
            for (i, motor) in enumerate(propertynames(command)[1:12])
                m = getproperty(command, motor)
                A1Robot.setMotorCmd(node.interface, i-1, m.pos, m.vel, m.Kp, m.Kd, m.tau)
            end 
        end 

        Hg.publish.(nodeio.pubs)
    end 

    function main(; rate=100.0)
        topics_dict = TOML.tryparsefile("$(@__DIR__)/../../topics.toml")

        imu_pub_ip = topics_dict["topics"]["imu"]["server"]
        encoder_pub_ip = topics_dict["topics"]["encoders"]["server"]
        command_sub_ip = topics_dict["topics"]["commands"]["server"] 

        imu_pub_port = topics_dict["topics"]["imu"]["port"]
        encoder_pub_port = topics_dict["topics"]["encoders"]["port"] 
        command_sub_port = topics_dict["topics"]["commands"]["port"] 

        node =HardwareInterfaceNode(imu_pub_ip, imu_pub_port,
                                    encoder_pub_ip, encoder_pub_port,
                                    command_sub_ip, command_sub_port, rate)
        return node
    end 

end 