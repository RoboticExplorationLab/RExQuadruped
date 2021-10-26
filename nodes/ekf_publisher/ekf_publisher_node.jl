module EKFPublisher 
    using Revise 
    using EKF
    using LinearAlgebra
    import Mercury as Hg 
    import ZMQ 

    include(joinpath(@__DIR__, "../../", "msgs", "messages_pb.jl"))
    include(joinpath(pathof(EKF), "test", "imu_grav_comp", "imu_dynamics_discrete.jl")) 
    mutable struct FilterNode <: Hg.Node 
        nodeio::Hg.NodeIO 
        rate::Float64 
        should_finish::Bool 

        ## Messages 
        vicon::ViconMsg
        imu::ImuMsg 
        filtered_state::EKFMsg 
        
        ## time step 
        h::Float64
        ekf::ErrorStateFilter 

        function FilterNode(imu_sub_ip::String, imu_sub_port::String, 
                            vicon_sub_ip::String, vicon_sub_port::String, 
                            ekf_pub_ip::String, ekf_pub_port::String, rate::Float64)
            filterNodeIO = Hg.NodeIO(ZMQ.Context(1); rate=rate)
            rate = rate 
            should_finish = false 

            ## Initialize the messages 
            vicon = ViconMsg(pos=Vector3Msg(x=0.0, y=0.0, z=0.0), 
                             quat=QuaternionMsg(w=1.0, x=0.0, y=0.0, z=0.0),
                             time = 0.0)
            imu = ImuMsg(acc = Vector3Msg(x=0.0, y=0.0, z=0.0),
                         gyro = Vector3Msg(x=0.0, y=0.0, z=0.0),
                         time = 0.0)

            filtered_state = EKFMsg(pos=Vector3Msg(x=0.0, y=0.0, z=0.0),
                                    quat=QuaternionMsg(w=0.0, x=0.0, y=0.0, z=0.0),
                                    v=Vector3Msg(x=0.0, y=0.0, z=0.0),
                                    v_ang=Vector3Msg(x=0.0, y=0.0, z=0.0),
                                    acc_bias=Vector3Msg(x=0.0, y=0.0, z=0.0),
                                    v_ang_bias=Vector3Msg(x=0.0, y=0.0, z=0.0),
                                    time=0.0)

            ## Initialize EKF 
            h = 0.002
            state_init = zeros(length(TrunkState)); state_init[7] = 1.0 
            state = TrunkState(state_init)
            vicon_init = zeros(7); vicon_init[4] = 1.0

            P = Matrix(1.0I(length(TrunkError))) * 1e10; 
            W = Matrix(1.0I(length(TrunkError))) * 1e-3;
            W[1:3, 1:3] .= I(3) * 1e-4
            W[4:6, 4:6] .= I(3) * 1e-5
            W[7:9, 7:9] .= I(3) * 1e-4
            W[end-5:end,end-5:end] = I(6)*1e2
            R = Matrix(1.0I(length(ViconError))) * 1e-5;
            R[1:3,1:3] = I(3) * 1e-3
            R[4:6,4:6] = I(3) * 1e-3 
            ekf = ErrorStateFilter{TrunkState, TrunkError, ImuInput, Vicon, ViconError}(state, P, W, R) 

            # Publishers (ekf)
            ekf_pub = Hg.ZmqPublisher(nodeio.ctx, ekf_pub_ip, ekf_pub_port)
            Hg.add_publisher!(nodeio, filtered_state, ekf_pub)

            # Subscriber (imu, vicon)
            imu_sub = Hg.ZmqSubscriber(nodeio.ctx, imu_sub_ip, imu_sub_port; name="IMU_SUB")
            Hg.add_subscriber!(nodeio, imu, imu_sub)

            vicon_sub = Hg.ZmqSubscriber(nodeio.ctx, vicon_sub_ip, vicon_sub_port; name="VICON_SUB")
            Hg.add_subscriber!(nodeio, vicon, vicon_sub)

            return new(nodeio, rate, shoudl_finish, vicon, imu, filtered_state, ekf, h)

        end 
    end 

    function Hg.compute(node::FilterNode)
        nodeio = Hg.getIO(node)
        imu_sub = Hg.getsubscriber(node, "IMU_SUB")
        vicon_sub = Hg.getsubscriber(node, "VICON_SUB")
        ## EKF Prediction base on IMU 
        Hg.on_new(imu_sub) do imu 
            input = ImuInput(imu.acc.x, imu.acc.y, imu.acc.z, imu.gyro.x, imu.gyro.y, imu.gyro.z)
            prediction!(ekf, input, node.h)
        end 
        ## EKF Update base on IMU 
        Hg.on_new(vicon_sub) do vicon
            vicon_measurement = Vicon(vicon.position.x, vicon.position.y, vicon.position.z, vicon.quaternion.w, vicon.quaternion.x, vicon.quaternion.y, vicon.quaternion.z)
            update!(ekf, vicon_measurement)
        end 

        ## Publishing 
        r, v, q, α, β = getComponents(TrunkState(ekf.est_state))
        node.filtered_state.quat.w, node.filtered_state.quat.x, node.filtered_state.quat.y, node.filtered_state.quat.z = q 
        node.filtered_state.pos.x, node.filtered_state.pos.y, node.filtered_state.z = r 
        node.filtered_state.acc_bias.x, node.filtered_state.acc_bias.y, node.filtered_state.acc_bias.z = α 
        node.filtered_state.v.x, node.filtered_state.v.y, node.filtered_state.v.z = v 
        node.filtered_state.v_ang.x, node.filtered_state.v_ang.y, node.filtered_state.v_ang.z = imu.gyro.x, imu.gyro.y, imu.gyro.z 
        node.filtered_state.v_ang_bias.x, node.filtered_state.v_ang_bias.y, node.filtered_state.v_ang_bias.z = β 
        node.filtered_state.time = time() 
 
        Hg.publish.(nodeio.pubs)
    end 

    function main(; rate=100.0)
        topics_dict = TOML.tryparsefile("$(@__DIR__)/../../topics.toml")

        imu_sub_ip = topics_dict["topics"]["imu"]["server"]
        vicon_sub_ip = topics_dict["topics"]["vicon"]["server"] 
        ekf_pub_ip = topics_dict["topics"]["ekf"]["server"]

        imu_sub_port = topics_dict["topics"]["imu"]["port"]
        vicon_sub_port = topics_dict["topics"]["vicon"]["port"] 
        ekf_pub_port = topics_dict["topics"]["ekf"]["port"]

        node = FilterNode(imu_sub_ip, imu_sub_port,
                          vicon_sub_ip, vicon_sub_port,
                          ekf_pub_ip, ekf_pub_port, rate)
        return node
    end


end 