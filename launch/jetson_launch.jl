import Mercury as Hg 

# include("$(@__DIR__)/../nodes/ekf_publisher/ekf_publisher.jl")
include("$(@__DIR__)/../nodes/dummy_node/dummy_node.jl")

dummy_node = DummyPublisher.main(; rate=100.0)

dummy_node_task = Threads.@spawn Hg.launch(dummy_node)
