import Mercury as Hg 

# include("$(@__DIR__)/../nodes/ekf_publisher/ekf_publisher.jl")
# include("$(@__DIR__)/../nodes/dummy_node/dummy_node.jl")
# include("$(@__DIR__)/../nodes/hardware_interface/hardware_interface_node.jl")
# include("$(@__DIR__)/../nodes/controller/controller_node.jl")
include("$(@__DIR__)/../nodes/ekf_publisher/vicon_ekf_node.jl")


# dummy_node = DummyPublisher.main(; rate=100.0)
# interface_node = HardwareInterface.main(; rate=200.0)
# controller_node = Controller.main(; rate=200.0)
ekf_node = EKFPublisher.main(; rate=200.0)

# dummy_node_task = Threads.@spawn Hg.launch(dummy_node)
# interface_node_task = Threads.@spawn Hg.launch(interface_node)
# controller_task = Threads.@spawn Hg.launch(controller_node) 
ekf_task = Threads.@spawn Hg.launch(ekf_node) 
