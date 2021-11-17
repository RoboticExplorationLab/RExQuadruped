module TrackerEKFPublisher 
    using Revise 
    using LinearAlgebra
    import Mercury as Hg 
    import ZMQ 
    import TOML
    import EKF 
    import EKF.CommonSystems as ComSys
    using Rotations
    using StaticArrays

    include(joinpath(@__DIR__, "../../", "msgs", "messages_pb.jl"))
    mutable struct FilterNode <: Hg.Node 
        nodeio::Hg.NodeIO 

        ## Messages 
        vicon::ViconMsg
        imu::ImuMsg 
        filtered_state::EKFMsg 
        
        ## time step 
        h::Float64
        ekf::EKF.ErrorStateFilter 
        timer::Float64

        function FilterNode(imu_sub_ip::String, imu_sub_port::String, 
                            vicon_sub_ip::String, vicon_sub_port::String, 
                            ekf_pub_ip::String, ekf_pub_port::String, rate::Float64)
            nodeio = Hg.NodeIO(ZMQ.Context(1); rate=rate)

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
            state = ComSys.TrackerState{Float64}(zeros(3)..., [1.,0,0,0]..., zeros(6)...)
            # stateErr = ComSys.TrackerError{Float64}(zeros(12)...)
            # input = ComSys.GyroInput{Float64}(zeros(3))
            # meas = ComSys.ViconMeasure{Float64}(zeros(3)..., [1.,0,0,0]...)
            # measErr = ComSys.ViconError{Float64}(zeros(6))
            vicon_init = zeros(7); vicon_init[4] = 1.0
            
            P = Matrix(1.0I(length(ComSys.TrackerError))) * 1e10; 
            W = Matrix(1.0I(length(ComSys.TrackerError))) * 1e-3;
            W[1:3, 1:3] .= I(3) * 1e-2
            W[4:6, 4:6] .= I(3) * 1e-3
            W[7:9, 7:9] .= I(3) * 1e-2
            W[10:12, 10:12] = I(3) * 1

            R = Matrix(1.0I(length(ComSys.ViconError))) * 1;
            R[1:3,1:3] = I(3) * 1e-3
            R[4:6,4:6] = I(3) * 1e-3 
            ekf = EKF.ErrorStateFilter{ComSys.TrackerState, ComSys.TrackerError, ComSys.GyroInput}(state, P, W) 

            # Publishers (ekf)
            ekf_pub = Hg.ZmqPublisher(nodeio.ctx, ekf_pub_ip, ekf_pub_port)
            Hg.add_publisher!(nodeio, filtered_state, ekf_pub)

            # Subscriber (imu, vicon)
            imu_sub = Hg.ZmqSubscriber(nodeio.ctx, imu_sub_ip, imu_sub_port; name="IMU_SUB")
            Hg.add_subscriber!(nodeio, imu, imu_sub)

            vicon_sub = Hg.ZmqSubscriber(nodeio.ctx, vicon_sub_ip, vicon_sub_port; name="VICON_SUB")
            Hg.add_subscriber!(nodeio, vicon, vicon_sub)
            timer = time()
            return new(nodeio, vicon, imu, filtered_state, h, ekf, timer)

        end 
    end 

    function Hg.compute(node::FilterNode)
        nodeio = Hg.getIO(node)
        imu_sub = Hg.getsubscriber(node, "IMU_SUB")
        vicon_sub = Hg.getsubscriber(node, "VICON_SUB")

        ## EKF Prediction base on IMU 
        Hg.on_new(imu_sub) do imu 
            dt = time() - node.timer 
            node.timer = time()
            input = EKF.CommonSystems.GyroInput(imu.gyro.x, imu.gyro.y, imu.gyro.z)
            EKF.prediction!(node.ekf, input, dt)
        end 

        ## EKF Update base on IMU 
        Hg.on_new(vicon_sub) do vicon
            meas = ComSys.ViconMeasure{Float64}(vicon.pos.x, vicon.pos.y, vicon.pos.z, vicon.quat.w, vicon.quat.x, vicon.quat.y, vicon.quat.z)
            oriObs = EKF.Observation(
                meas,
                SMatrix{length(ComSys.ViconError),length(ComSys.ViconError),Float64}(I(length(ComSys.ViconError))*1e-3),
            )
            EKF.update!(node.ekf, oriObs)
        end 

        ## Publishing 
        r, q, v, β = EKF.CommonSystems.getComponents(EKF.CommonSystems.TrackerState(node.ekf.est_state))
        node.filtered_state.quat.w, node.filtered_state.quat.x, node.filtered_state.quat.y, node.filtered_state.quat.z = Rotations.params(q) 
        node.filtered_state.pos.x, node.filtered_state.pos.y, node.filtered_state.pos.z = r 
        node.filtered_state.acc_bias.x, node.filtered_state.acc_bias.y, node.filtered_state.acc_bias.z = [0.0, 0.0, 0.0]
        node.filtered_state.v.x, node.filtered_state.v.y, node.filtered_state.v.z = v 
        node.filtered_state.v_ang.x, node.filtered_state.v_ang.y, node.filtered_state.v_ang.z = node.imu.gyro.x, node.imu.gyro.y, node.imu.gyro.z 
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