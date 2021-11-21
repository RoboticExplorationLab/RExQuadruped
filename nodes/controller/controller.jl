
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

function balance_control!(controller::Controller, x::AbstractVector, 
                                                  p_FR::AbstractVector, 
                                                  p_RL::AbstractVector, 
                                                  command::MotorCmdMsg,
                                                  control_error::ControlErrorMsg)
    x_err = @MVector zeros(36)
    
    ### Angle error 
    quat_measured = Rotations.UnitQuaternion(x[1:4])
    quat_des = Rotations.UnitQuaternion(controller.x_eq[1:4])
    θ_err = Rotations.rotation_error(quat_measured, quat_des, Rotations.CayleyMap())
    x_err[1:3] = θ_err 

    ### Position error 
    eq_point = controller.x_eq[5:6]
    v_support = p_FR[1:2] - p_RL[1:2] # support line 
    P_project = v_support * v_support' / (v_support' * v_support) # projection matrix 
    p_project = p_RL[1:2] + P_project*(x[5:6] - p_RL[1:2]) # projected point on the line 
    # x_err[4:5] = quat_des[1:2,1:2]' * (x[5:6] - p_project)
    x_err[4:5] = quat_des[1:2,1:2]' * (x[5:6] - eq_point)
    x_err[6] = x[7] - controller.x_eq[7]

    ### rest of the error   
    x_err[7:18] = x[8:19] - controller.x_eq[8:19] # joint error 
    x_err[19:21] = x[20:22]  # ω
    v_world = quat_measured * x[23:25] # world frame velocity 
    x_err[22:23] = (quat_des' * v_world)[1:2]  #- P_project * x[23:24]  # v in body frame 
    x_err[24] = v_world[3]
    x_err[25:36] = x[26:end] # joint v 
    
    ### calculate control 
    u_fb = -controller.K*x_err 
    # u_fb = min.(max.(u_fb, -8.0), 8.0)
    u = u_fb + controller.u_eq 
    u = min.(max.(u, -20.0), 20.0)

    ### safety 
    if(any(abs.(θ_err) .> 0.2) || any(abs.(x_err[4:6]) .> 0.1)) 
        println("breaking due to attitude")
        controller.isOn = false 
    end 

    ## save data to file 
    # open("control_error.txt", "a") do io 
    #     println(io, x_err)
    # end 

    if(controller.isOn)
        u = map_motor_arrays(u, MotorIDs_rgb, MotorIDs_c)
        set_torque_cmds!(command, u* controller.isOn)
        # set_torque_cmds_debug!(command, u)
    end

    ## set control error 
    control_error.pos.x, control_error.pos.y, control_error.pos.z = x_err[4:6]
    control_error.attitude.x, control_error.attitude.y, control_error.attitude.z = x_err[1:3]
    control_error.v_ang.x, control_error.v_ang.y, control_error.v_ang.z = x_err[19:21]
    control_error.vel.x, control_error.vel.y, control_error.vel.z = x_err[22:24]
    control_error.time = time()
    
    # for (i, motor) in enumerate(fieldnames(MotorIDs))
    #     joint_pos = getproperty(control_error.joint_pos, motor)
    #     joint_vel = getproperty(control_error.joint_vel, motor)
    # end  

 end

function joint_linear_interpolation(q_start, q_target, rate)
    q_now = q_start * (1-rate) + q_target * rate 
    return q_now 
end 

