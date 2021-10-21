module EKFPublisher 
    using Revise 
    import Mercury as Hg 
    import ZMQ 

    mutable struct FilterNode <: Hg.Node 
        nodeio::Hg.NodeIO 
        rate::Float64 
        should_finish::Bool 
        
        include(joinpath(@__DIR__, "../../", "msgs", "imu_msg_pb.jl"))
        function FilterNode(imu_sub_ip::String, imu_sub_port::String, rate::Float64)
            filterNodeIO = Hg.NodeIO(ZMQ.Context(1); rate=rate)
            rate = rate 
            ctx = Context(1)

            test = ImuMsg(x=0.0, y=0.0, z=0.0, time=0.0)
        end 
    end 

    function Hg.compute(node::FilterNode)
        println("???")
    end 


end 