""" 
    This file contains functions that handles the difference indexing scheme 
    between the unitree output and our controller. Also includes function that 
    helps fill out protobuf command messages 
"""

struct MotorIDs 
    FR_Hip::Int; FR_Thigh::Int; FR_Calf::Int
    FL_Hip::Int; FL_Thigh::Int; FL_Calf::Int 
    RR_Hip::Int; RR_Thigh::Int; RR_Calf::Int
    RL_Hip::Int; RL_Thigh::Int; RL_Calf::Int
end  

const MotorIDs_c = MotorIDs(1,2,3,4,5,6,7,8,9,10,11,12)
const MotorIDs_rgb = MotorIDs(1,5,9,2,6,10,3,7,11,4,8,12)

function map_motor_arrays(cmd1::AbstractVector{Float64}, ids1::MotorIDs, ids2::MotorIDs)
    cmd2 = @MVector zeros(length(cmd1))
    for motor in fieldnames(MotorIDs) 
        id1 = getfield(ids1,motor)
        id2 = getfield(ids2,motor)
        cmd2[id2] = cmd1[id1]
    end 
    cmd2 = SVector(cmd2)
    return cmd2
end 

### Functions that help set protobuf commands ###
function set_position_cmds!(cmds_msg::MotorCmdMsg, pos::AbstractVector{Float64}, 
                            Kp::Real, Kd::Real)
    for (i,motor) in enumerate(fieldnames(MotorIDs))
        m = getproperty(cmds_msg, motor)
        m.Kp = Kp 
        m.Kd = Kd 
        m.pos = pos[i]
        m.tau = 0.0
        m.vel = 0.0
    end 
end 

function set_torque_cmds!(cmds_msg::MotorCmdMsg, torques::AbstractVector{Float64})
    posStopF = 2.146e9
    velStopF = 16000.0e0
    for (i, motor) in enumerate(fieldnames(MotorIDs))
        # if motor in [:FL_Calf, :RR_Calf, :FL_Thigh, :RR_THigh, :FL_Hip, :RR_Hip]
        # if motor in [:FR_Calf, :RL_Calf, :FR_Thigh, :RL_THigh, :FR_Hip, :RL_Hip]
            m = getproperty(cmds_msg, motor)
            m.Kp = 0.0 
            m.Kd = 0.0
            m.pos = posStopF 
            m.vel = velStopF 
            m.tau = torques[i]
        # end
    end 
end 

function set_torque_cmds_debug!(cmds_msg::MotorCmdMsg, torques::AbstractVector{Float64})
    posStopF = 2.146e9
    velStopF = 16000.0e0
    for (i, motor) in enumerate(fieldnames(MotorIDs))
        m = getproperty(cmds_msg, motor)
        m.debug = torques[i]
    end 
end 

### Function that extract protobuf messages to controller statespace ###
function extract_sensor_readings(encoders::JointSensorsMsg)
    vs = SVector{12}([getproperty(encoders.velocities, motor) for motor in fieldnames(MotorIDs)]) 
    qs = SVector{12}([getproperty(encoders.positions, motor) for motor in fieldnames(MotorIDs)]) 
    τs = SVector{12}([getproperty(encoders.torques, motor) for motor in fieldnames(MotorIDs)])
    fs = SVector{4}([encoders.FR_foot, encoders.FL_foot, encoders.RR_foot, encoders.RL_foot])
    return vs, qs, τs, fs
end 

function extract_state(encoders::JointSensorsMsg, filtered_state::LeggedEKFMsg)
    x = @MVector zeros(37)
    vs, qs, τs, _ = extract_sensor_readings(encoders)
    x[1:4] = [filtered_state.quat.w filtered_state.quat.x filtered_state.quat.y filtered_state.quat.z] 
    x[5:7] = [filtered_state.pos.x filtered_state.pos.y filtered_state.pos.z]
    x[20:22] = [filtered_state.v_ang.x filtered_state.v_ang.y filtered_state.v_ang.z] 
    x[23:25] = [filtered_state.v.x filtered_state.v.y filtered_state.v.z]
    x[8:19] = map_motor_arrays(qs, MotorIDs_c, MotorIDs_rgb)
    x[26:end] = map_motor_arrays(vs, MotorIDs_c, MotorIDs_rgb) 
    x = SVector(x)
    return x
end 

function extract_state(encoders::JointSensorsMsg, filtered_state::EKFMsg)
    x = @MVector zeros(37)
    vs, qs, τs, _ = extract_sensor_readings(encoders)
    x[1:4] = [filtered_state.quat.w filtered_state.quat.x filtered_state.quat.y filtered_state.quat.z] 
    x[5:7] = [filtered_state.pos.x filtered_state.pos.y filtered_state.pos.z]
    x[20:22] = [filtered_state.v_ang.x filtered_state.v_ang.y filtered_state.v_ang.z] 
    x[23:25] = [filtered_state.v.x filtered_state.v.y filtered_state.v.z]
    x[8:19] = map_motor_arrays(qs, MotorIDs_c, MotorIDs_rgb)
    x[26:end] = map_motor_arrays(vs, MotorIDs_c, MotorIDs_rgb) 
    x = SVector(x)
    return x
end 



