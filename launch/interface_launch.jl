import Mercury as Hg 

include("$(@__DIR__)/../nodes/hardware_interface/hardware_interface_node.jl")


interface_node = HardwareInterface.main(; rate=200.0)

interface_node_task = Threads.@spawn Hg.launch(interface_node)
