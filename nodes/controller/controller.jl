
mutable struct Controller
    x_eq::Vector{Float64} # equilibrium pose 
    x_init::Vector{Float64}
    q_stand::Vector{Float64} # standing pose prior to balance 
    u_eq::Vector{Float64} # equilibrium control torques 
    K::Matrix{Float64} # balance control gains
    Kp::Float64 # joint P control gain 
    Kd::Float64 # joint D control gain 
    isOn::Bool 
end 

function standing_control!(controller::Controller, 
                           command::MotorCmdMsg, rate::Float64)
    
    q_target = controller.q_stand 
    q_start = controller.x_init[8:19]
    q_interpolated = q_start * (1 - rate) + q_target * rate 

    set_position_cmds!(command, q_interpolated, controller.Kp * controller.isOn, controller.Kd* controller.isOn)
end 

function balance_control!(controller::Controller, x::Vector{Float64}, 
                                                  p_FR::AbstractVector, 
                                                  p_RL::AbstractVector, 
                                                  command::MotorCmdMsg)
    x_err = @MVector zeros(36)
    
    ### Angle error 
    quat_measured = Rotations.UnitQuaternion(x[1:4])
    quat_des = Rotations.UnitQuaternion(controller.x_eq[1:4])
    θ_err = Rotations.rotation_error(quat_measured, quat_des, Rotations.CayleyMap())
    x_err[1:3] = θ_err 

    ### Position error 
    v_support = p_FR[1:2] - p_RL[1:2] # support line 
    P_project = v_support * v_support' / (v_support' * v_support) # projection matrix 
    p_project = p_RL[1:2] + P_project(x[5:6] - p_RL[1:2]) # projected point on the line 
    x_err[4:5] = x[5:6] - p_project 
    
    ### rest of the error   
    x_err[7:18] = x[8:19] - controller.x_eq[8:19] # joint error 
    x_err[19:21] = x[20:22]  # ω
    x_err[22:24] = x[23:25]  # v 
    x_err[25:36] = x[26:end] # joint v 
    
    ### calculate control 
    u = -K*x_err + controller.u_eq
        
    set_torque_cmds!(command, u)
end

function joint_linear_interpolation(q_start, q_target, rate)
    q_now = q_start * (1-rate) + q_target * rate 
    return q_now 
end 

