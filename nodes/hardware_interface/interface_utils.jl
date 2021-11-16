const HIP_LIMIT = [-0.802851, 0.802851]
const THIGH_LIMIT = [-1.0472, 4.18879]
const CALF_LIMIT = [-2.69653, -0.916298]
const joint_buffer = 0.05 # buffer zone in which we start applying the 
                          # position hold 

""" 
    Limit the torques at the joint limit so it doesn't break it. 
    If a joint approach its limit and its torque is going in the 
    direction of its limit, we apply a position hold at the limit. 
    If a joint is at its limit and its torque is going away from the 
    limit, then we apply the torque 
"""
function torque_limiter!(index, m::CmdMsg, joint_pos::AbstractVector,
                                           joint_vel::AbstractVector, 
                                           Kp=100.0, Kd = 5.0, dt=0.005)
    if(m.tau != 0)
        q = joint_pos[index];
        v = joint_vel[index];
        upper = 0.0; 
        lower = 0.0; 
        if(index in [1,4,7,10])
            lower = HIP_LIMIT[1]
            upper = HIP_LIMIT[2];
        elseif (index in [2,5,8,11])
            lower = THIGH_LIMIT[1];
            upper = THIGH_LIMIT[2];
        else # not doing it for the calf 
            lower = CALF_LIMIT[1];
            upper = CALF_LIMIT[2];
            return 
        end

        # lower limit 
        if(q + v*dt + 10*m.tau*dt^2 - lower < joint_buffer && sign(m.tau) == -1)
            m.Kp = Kp 
            m.Kd = Kd 
            m.pos = lower 
            m.tau = 0.0
            m.vel = 0.0
        end 

        # upper limit 
        if(q + v*dt + 10*m.tau*dt^2 - upper > -joint_buffer && sign(m.tau) == 1)
            m.Kp = Kp 
            m.Kd = Kd 
            m.pos = upper 
            m.tau = 0.0
            m.vel = 0.0
        end 

        # m.pos = upper
        # m.Kp = 0.0
    end 
end 