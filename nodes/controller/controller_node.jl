module ControllerModule 
    using Revise 
    import Mercury as Hg 
    import ZMQ 
    import TOML 
    using StaticArrays
    using Rotations
    using DelimitedFiles 

    include(joinpath(@__DIR__, "../../", "msgs", "messages_pb.jl"))
    include(joinpath(@__DIR__, "control_utils.jl"))
    include(joinpath(@__DIR__, "controller.jl"))
 
    mutable struct ControllerNode <: Hg.Node 
        ## Required by Abstract Node type 
        nodeio::Hg.NodeIO 
        rate::Float64
        should_finish::Bool

        ## Protobuf Messages 
        filtered_state::LeggedEKFMsg 
        encoders::JointSensorsMsg
        command::MotorCmdMsg

        ## Node specific member 
        controller::Controller
        balance::Bool 
        start_time::Float64
        function ControllerNode(ekf_sub_ip::String, ekf_sub_port::String, 
                                encoder_sub_ip::String, encoder_sub_port::String,
                                command_pub_ip::String, command_pub_port::String, 
                                warmstart::Bool, rate)
            nodeio = Hg.NodeIO(ZMQ.Context(1); rate=rate)
            rate= rate; 
            ctx = ZMQ.Context(1)
            should_finish = false 

            # Message init 
            filtered_state = LeggedEKFMsg(pos=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         quat=QuaternionMsg(w=0.0, x=0.0, y=0.0, z=0.0),
                         v=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         v_ang=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         acc_bias=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         v_ang_bias=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         p1=Vector3Msg(x=0.0, y=0.0, z=0.0), 
                         p2=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         p3=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         p4=Vector3Msg(x=0.0, y=0.0, z=0.0),
                         time=0.0)
            
            joint_message = JointMsg(FR_Hip=0.0, FR_Thigh=0.0, FR_Calf=0.0,
                                     FL_Hip=0.0, FL_Thigh=0.0, FL_Calf=0.0,
                                     RR_Hip=0.0, RR_Thigh=0.0, RR_Calf=0.0,
                                     RL_Hip=0.0, RL_Thigh=0.0, RL_Calf=0.0)

            encoders = JointSensorsMsg(torques=deepcopy(joint_message),
                                       positions=deepcopy(joint_message),
                                       velocities=deepcopy(joint_message),
                                       FR_foot=0.0, FL_foot=0.0, RR_foot=0.0,
                                       RL_foot=0.0,
                                       time=0.0)
            
            cmd_message = CmdMsg(Kp=0.0, Kd=0.0, pos=0.0, vel=0.0, tau=0.0, debug=0.0)
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
            
            # Load controller 
            data_eq = TOML.parsefile(joinpath(@__DIR__, "ipopt_eq_point.toml"))
            x_init = TOML.parsefile(joinpath(@__DIR__, "resting.toml"))["x_init"]
            K = readdlm(joinpath(@__DIR__, "maximal_lqr_gain.txt"), '\t', Float64, '\n')
            controller = Controller(data_eq["x_eq"], x_init, data_eq["q_stand"], 
                                    data_eq["u_eq"], K, 100.0, 1.0, false)

            # Subscriber (encoders, ekf)
            encoder_sub = Hg.ZmqSubscriber(nodeio.ctx, encoder_sub_ip, encoder_sub_port; name="ENC_SUB") 
            Hg.add_subscriber!(nodeio, encoders, encoder_sub)

            ekf_sub = Hg.ZmqSubscriber(nodeio.ctx, ekf_sub_ip, ekf_sub_port; name="EKF_SUB") 
            Hg.add_subscriber!(nodeio, filtered_state, ekf_sub)

            # Publishers (command)
            command_pub = Hg.ZmqPublisher(nodeio.ctx, command_pub_ip, command_pub_port) 
            Hg.add_publisher!(nodeio, command, command_pub)
            balance = false
            
            return new(nodeio, rate, should_finish, filtered_state, encoders, command, controller, balance, time())
        end 

    end 

    function Hg.compute(node::ControllerNode)
        nodeio = Hg.getIO(node)

        # Logic switching 
        dt = (time() - node.start_time)
        rate = dt > 8.0 ? 1.0 : dt / 8.0 # takes five second to stand 

        ## extract the sensor readings 
        x = extract_state(node.encoders, node.filtered_state)
        p_FR = @SVector [node.filtered_state.p1.x, node.filtered_state.p1.y, node.filtered_state.p1.z]
        p_RL = @SVector [node.filtered_state.p4.x, node.filtered_state.p4.y, node.filtered_state.p4.z] 
        
        ## controllers 
        if(node.balance == false)
            standing_control!(node.controller, node.command, rate)
        else
            standing_control!(node.controller, node.command, rate)
            balance_control!(node.controller, x, p_FR, p_RL, node.command)
            if(node.encoders.FR_foot < 0 || node.encoders.RL_foot < 0) 
                node.controller.isOn = false;
                node.balance = false; 
            end 
        end 

        Hg.publish.(nodeio.pubs) 
    end 

    function control_on!(node::ControllerNode)
        data_eq = TOML.parsefile(joinpath(@__DIR__, "ipopt_eq_point.toml"))
        node.controller.q_stand = data_eq["q_stand"]
        node.start_time = time() 
        node.controller.isOn = true 
        x = extract_state(node.encoders, node.filtered_state)
        node.controller.x_init = copy(x);
    end 

    function control_off!(node::ControllerNode)
        node.controller.isOn = false; 
        node.balance = false; 
    end 

    function balance_on!(node::ControllerNode)
        node.balance = true 
        node.start_time = time() 
        ## debug 
        x = extract_state(node.encoders, node.filtered_state)
        node.controller.x_init = copy(x);
        node.controller.q_stand = node.controller.x_eq[8:19]
    end 

    function balance!(node::ControllerNode)
        node.balance = true 
        node.start_time = time() 
        ## debug 
        x = extract_state(node.encoders, node.filtered_state)
        node.controller.x_init = copy(x);
        node.controller.q_stand = node.controller.x_eq[8:19]
    end 

    function main(; rate=100.0, debug=false, warmstart=true)
        topics_dict = TOML.tryparsefile("$(@__DIR__)/../../topics.toml")
        command_pub_ip = topics_dict["topics"]["commands"]["server"]
        encoder_sub_ip = topics_dict["topics"]["encoders"]["server"]
        ekf_sub_ip = topics_dict["topics"]["ekf"]["server"] 

        command_pub_port = topics_dict["topics"]["commands"]["port"]
        encoder_sub_port = topics_dict["topics"]["encoders"]["port"] 
        ekf_sub_port = topics_dict["topics"]["ekf"]["port"] 

        if(debug)
            command_pub_ip = "127.0.0.1"
            encoder_sub_ip = "127.0.0.1"
            ekf_sub_ip = "127.0.0.1" 
        end 

        node = ControllerNode(ekf_sub_ip, ekf_sub_port,
                              encoder_sub_ip, encoder_sub_port,
                              command_pub_ip, command_pub_port, warmstart, rate)
        return node 
    end 


end 