import Mercury as Hg 

include("$(@__DIR__)/../nodes/ekf_publisher/legged_ekf_node.jl")

ekf_node = LeggedEKF.main(; rate=200.0);
ekf_task = Threads.@spawn Hg.launch(ekf_node) 