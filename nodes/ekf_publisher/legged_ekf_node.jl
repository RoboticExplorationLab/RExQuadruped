module LeggedEKF
    using Revise 
    using EKF 
    using ForwardDiff
    using LinearAlgebra
    import Mercury as Hg
    using ZMQ 
    import TOML
    using StaticArrays
    include(joinpath(@__DIR__, "../../", "msgs", "messages_pb.jl"))
    include(joinpath(@__DIR__, "../", "controller", "model.jl"))
    include(joinpath(@__DIR__, "../", "controller", "control_utils.jl"))
    mutable struct FilterNode <: Hg.Node 
        nodeio::Hg.NodeIO 

        ## Messages 
        imu::ImuMsg 
        filtered_state::LeggedEKFMsg 
        encoders::JointSensorsMsg 

        ## Node specific members 
        h::Float64
        ekf::EKF.ErrorStateFilter 
        timer::Float64
        R::Matrix
        contact1::EKF.CommonSystems.ContactObservation1 
        contact2::EKF.CommonSystems.ContactObservation2 
        contact3::EKF.CommonSystems.ContactObservation3 
        contact4::EKF.CommonSystems.ContactObservation4

        function FilterNode(imu_sub_ip::String, imu_sub_port::String, 
                            encoder_sub_ip::String, encoder_sub_port::String, 
                            ekf_pub_ip::String, ekf_pub_port::String, rate::Float64)
            nodeio = Hg.NodeIO(ZMQ.Context(1); rate=rate)

            ## Initialize messages 
            imu = ImuMsg(acc = Vector3Msg(x=0.0, y=0.0, z=0.0),
                         gyro = Vector3Msg(x=0.0, y=0.0, z=0.0),
                         time = 0.0)

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
                                       FR_foot = 0.0, FL_foot = 0.0, 
                                       RR_foot = 0.0, RL_foot = 0.0,
                                       time=0.0)
            
            ## Initialize EKF 
            h = 0.01
            s_init = zeros(length(EKF.CommonSystems.LeggedState)); s_init[4] = 1.0
            state = EKF.CommonSystems.LeggedState(s_init);
            P = Matrix(1.0I(length(EKF.CommonSystems.LeggedError))) * 1e-1; 
            P[10:21, 10:21] .= I(12) * 1e2
            P[22:27, 22:27] = I(6) * 1e2
            W = Matrix(1.0I(length(EKF.CommonSystems.LeggedError)));
            W[1:3,1:3] = I(3) * 1e-2          # position uncertainty 
            W[4:6, 4:6] = I(3) * 1e-2        # orientation uncertainty 
            W[7:9, 7:9] = I(3) * 1e-2         # velocity uncertainty 
            W[10:21, 10:21] = I(12) * 1e-2    # foot position uncertainty while in contact
            W[22:24, 22:24] = I(3) * 1e-1       # acc bias uncertainty 
            W[25:27, 25:27] = I(3) * 1e-2     # rotation bias uncertainty  
            ekf = EKF.ErrorStateFilter{EKF.CommonSystems.LeggedState, 
                                       EKF.CommonSystems.LeggedError, 
                                       EKF.CommonSystems.ImuInput}(state, P, W)

            R1 = SMatrix{3,3,Float64}(Diagonal(ones(3)) * 1e-2 )
            R = SMatrix{12,12, Float64}(Diagonal(ones(12)) * 1e-2)
            contact1 = EKF.CommonSystems.ContactObservation1(
                                    EKF.CommonSystems.ContactMeasure(zeros(3)), R1)
            contact2 = EKF.CommonSystems.ContactObservation2(
                                    EKF.CommonSystems.ContactMeasure(zeros(3)), R1)
            contact3 = EKF.CommonSystems.ContactObservation3(
                                    EKF.CommonSystems.ContactMeasure(zeros(3)), R1)
            contact4 = EKF.CommonSystems.ContactObservation4(
                                    EKF.CommonSystems.ContactMeasure(zeros(3)), R1)

            EKF.update!(ekf, contact1)
            EKF.update!(ekf, contact2)
            EKF.update!(ekf, contact3)
            EKF.update!(ekf, contact4)

            # Subscriber (encoders, imu)
            encoder_sub = Hg.ZmqSubscriber(nodeio.ctx, encoder_sub_ip, encoder_sub_port; name="ENC_SUB")
            Hg.add_subscriber!(nodeio, encoders, encoder_sub)

            imu_sub = Hg.ZmqSubscriber(nodeio.ctx, imu_sub_ip, imu_sub_port; name="IMU_SUB")
            Hg.add_subscriber!(nodeio, imu, imu_sub)

            # Publisher (ekf)
            ekf_pub = Hg.ZmqPublisher(nodeio.ctx, ekf_pub_ip, ekf_pub_port)
            Hg.add_publisher!(nodeio, filtered_state, ekf_pub)
            
            timer = time() 
            return new(nodeio, imu, filtered_state, encoders, h, ekf, timer, R, contact1, contact2, contact3, contact4)
        end 
    end 

    function Hg.compute(node::FilterNode)
        nodeio = Hg.getIO(node)
        imu_sub = Hg.getsubscriber(node, "IMU_SUB")
        enc_sub = Hg.getsubscriber(node, "ENC_SUB")

        ## EKF Prediction based on IMU 
        Hg.on_new(imu_sub) do imu 
            dt = time() - node.h 
            input = EKF.CommonSystems.ImuInput(imu.acc.x, imu.acc.y, imu.acc.z, imu.gyro.x, imu.gyro.y, imu.gyro.z)
            EKF.prediction!(node.ekf, input, dt)
            node.h = time()
        end 

        ## EKF Update based on encoders 
        Hg.on_new(enc_sub) do encoders 
            vs, qs, τs, fs = extract_sensor_readings(encoders)
            qs = map_motor_arrays(qs, MotorIDs_c, MotorIDs_rgb)
            p = fk(qs)
            J = dfk(qs)
            node.contact1.measurement = EKF.CommonSystems.ContactMeasure(p[1:3])
            node.contact2.measurement = EKF.CommonSystems.ContactMeasure(p[4:6])
            node.contact3.measurement = EKF.CommonSystems.ContactMeasure(p[7:9])
            node.contact4.measurement = EKF.CommonSystems.ContactMeasure(p[10:12])
            
            J1 = @view J[1:3,:]
            node.contact1.measure_cov = SMatrix{3,3,Float64}(J1 * node.R * J1') 
            J2 = @view J[4:6,:]
            node.contact2.measure_cov = SMatrix{3,3,Float64}(J2 * node.R * J2') 
            J3 = @view J[7:9,:]
            node.contact3.measure_cov = SMatrix{3,3,Float64}(J3 * node.R * J3') 
            J4 = @view J[10:12,:]
            node.contact4.measure_cov = SMatrix{3,3,Float64}(J4 * node.R * J4') 

            if(fs[1] > 0)
                EKF.update!(node.ekf, node.contact1)
            else
                node.ekf.est_cov[10:12,10:12] .= node.ekf.est_cov[10:12,10:12] + I(3)*1e2
            end 
            if(fs[2] > 0)
                EKF.update!(node.ekf, node.contact2)
            else 
                node.ekf.est_cov[13:15,13:15] .= node.ekf.est_cov[13:15,13:15] + I(3)*1e2
            end 
            if(fs[3] > 0)
                EKF.update!(node.ekf, node.contact3)
            else 
                node.ekf.est_cov[16:18,16:18] .= node.ekf.est_cov[16:18,16:18] + I(3)*1e2
            end
            if(fs[4] > 0)
                EKF.update!(node.ekf, node.contact4)
            else 
                node.ekf.est_cov[19:21,19:21] .= node.ekf.est_cov[19:21,19:21] + I(3)*1e2
            end 
        end 

        ## Publishing 
        r, q, v, p1, p2, p3 ,p4, α, β = EKF.CommonSystems.getComponents(EKF.CommonSystems.LeggedState(node.ekf.est_state))
        node.filtered_state.quat.w, node.filtered_state.quat.x, node.filtered_state.quat.y, node.filtered_state.quat.z = q.w, q.x, q.y, q.z
        node.filtered_state.pos.x, node.filtered_state.pos.y, node.filtered_state.pos.z = r 
        node.filtered_state.acc_bias.x, node.filtered_state.acc_bias.y, node.filtered_state.acc_bias.z = α 
        node.filtered_state.v.x, node.filtered_state.v.y, node.filtered_state.v.z = v 
        node.filtered_state.v_ang.x, node.filtered_state.v_ang.y, node.filtered_state.v_ang.z = node.imu.gyro.x, node.imu.gyro.y, node.imu.gyro.z 
        node.filtered_state.v_ang_bias.x, node.filtered_state.v_ang_bias.y, node.filtered_state.v_ang_bias.z = β 
        node.filtered_state.p1.x, node.filtered_state.p1.y, node.filtered_state.p1.z = p1
        node.filtered_state.p2.x, node.filtered_state.p2.y, node.filtered_state.p2.z = p2 
        node.filtered_state.p3.x, node.filtered_state.p3.y, node.filtered_state.p3.z = p3 
        node.filtered_state.p4.x, node.filtered_state.p4.y, node.filtered_state.p4.z = p4
        node.filtered_state.time = time() 
        Hg.publish.(nodeio.pubs)
    end 

    function restart_ekf!(node::FilterNode)
        vs, qs, τs, fs = extract_sensor_readings(node.encoders)
        node.ekf.est_state[1:3] .= [0.0, 0.0, 0.3]
        node.ekf.est_state[4:7] = [1.0; 0.0; 0.0; 0.0]
        node.ekf.est_state[8:end] .= 0
        node.ekf.est_cov[:,:] = Diagonal(ones(length(EKF.CommonSystems.LeggedError))) * 1e-1
        node.ekf.est_cov[10:21, 10:21] .= I(12) * 1e2
        node.ekf.est_cov[22:27, 22:27] .= I(6) * 1e2
    end 
    function main(; rate=100.0, debug=false)
        topics_dict = TOML.tryparsefile("$(@__DIR__)/../../topics.toml")

        if debug
            imu_sub_ip = "127.0.0.1"
            encoder_sub_ip = "127.0.0.1" 
            ekf_pub_ip = "127.0.0.1"
        else 
            imu_sub_ip = topics_dict["topics"]["imu"]["server"]
            encoder_sub_ip = topics_dict["topics"]["encoders"]["server"] 
            ekf_pub_ip = topics_dict["topics"]["ekf"]["server"]
        end 

        imu_sub_port = topics_dict["topics"]["imu"]["port"]
        encoder_sub_port = topics_dict["topics"]["encoders"]["port"] 
        ekf_pub_port = topics_dict["topics"]["ekf"]["port"]

        node = FilterNode(imu_sub_ip, imu_sub_port,
                          encoder_sub_ip, encoder_sub_port,
                          ekf_pub_ip, ekf_pub_port, rate)
        return node
    end
end 