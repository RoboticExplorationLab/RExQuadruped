import Mercury as Hg 

include("$(@__DIR__)/../nodes/controller/controller_node_vicon.jl")


controller_node = ControllerModule.main(; rate=200.0, debug=false, warmstart=true)

controller_task = Threads.@spawn Hg.launch(controller_node) 
