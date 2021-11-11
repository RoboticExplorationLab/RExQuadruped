include(joinpath(@__DIR__, "../", "../", "nodes", "hardware_interface", "julia_robot_interface.jl"))
include(joinpath(@__DIR__, "../", "../", "msgs", "messages_pb.jl"))

interface = A1Robot.RobotInterface() 
A1Robot.InitSend(interface)