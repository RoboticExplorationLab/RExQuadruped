include(joinpath(@__DIR__, "../", "../",  "nodes", "hardware_interface", "interface_utils.jl"))

posStopF = 2.146e9
velStopF = 16000.0e0
m = CmdMsg(Kp=0.0, Kd=0.0, pos=posStopF, vel=velStopF, tau=0.0)
index = 4
try
    while true 
        qs, dqs, _, τs = A1Robot.getMotorReadings(interface)
        m = CmdMsg(Kp=0.0, Kd=0.0, pos=posStopF, vel=velStopF, tau=2.0)
        println(qs[4:6])
        torque_limiter!(index, m, qs, dqs)
        A1Robot.setMotorCmd(interface, index-1, m.pos, m.vel, m.Kp, m.Kd, m.tau)
        A1Robot.SendCommand(interface)
        sleep(0.005)
    end 
catch Keybo
    println("interrupted!")
end
