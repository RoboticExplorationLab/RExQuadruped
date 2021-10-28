
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
    q_interpolated = map_motor_arrays(q_interpolated, MotorIDs_rgb, MotorIDs_c)
    set_position_cmds!(command, q_interpolated, controller.Kp * controller.isOn, controller.Kd* controller.isOn)
end 

function balance_control!(controller::Controller, x::Vector{Float64}, command::MotorCmdMsg)
    
end

function joint_linear_interpolation(q_start, q_target, rate)
    q_now = q_start * (1-rate) + q_target * rate 
    return q_now 
end 

