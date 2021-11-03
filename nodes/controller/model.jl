""" 
    This file contains function for modeling the quadruped 
"""

function fk(q::AbstractVector)
    # these two variables indicates the quadrant of the leg
    x_hip = 0.183 # hip dispalcement from the trunk frame 
    y_hip = 0.047 
    Δy_thigh = 0.08505 # thigh displacement from the hip frame 
    l_limb = 0.2 # joint length for the leg and thigh. calf and thigh has the same x, y config 

    q_FR = q[[1,5,9]]
    q_FL = q[[2,6,10]]
    q_RR = q[[3,7,11]]
    q_RL = q[[4,8,12]]


    fk(x_mir, y_mir, θ) = [-l_limb * sin(θ[2] + θ[3]) - l_limb * sin(θ[2]) + x_hip * x_mir; 
                           y_hip * y_mir + Δy_thigh* y_mir * cos(θ[1]) + l_limb*sin(θ[1])*cos(θ[2]+θ[3]) + l_limb*sin(θ[1])*cos(θ[2]); 
                           Δy_thigh * y_mir *  sin(θ[1]) - l_limb * cos(θ[1]) * cos(θ[2] + θ[3]) - l_limb * cos(θ[1])*cos(θ[2]) ] 

    p = [fk(1, -1, q_FR)...;
         fk(1, 1, q_FL)...; 
         fk(-1,-1, q_RR)...; 
         fk(-1,1, q_RL)...]                   
    return p
end

function dfk(q::AbstractVector)
    return ForwardDiff.jacobian(t->fk(t), q)
end 
