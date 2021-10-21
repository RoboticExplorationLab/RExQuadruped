module DummyPublisher
    using Revise 
    import Mercury as Hg 
    import ZMQ 
    include(joinpath(@__DIR__, "../../", "msgs", "imu_msg_pb.jl"))

    mutable struct DummyNode <: Hg.Node 

        ## Required by Abstract Node type 
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool
        
        ## Specfic to this node 
        imu_msg::ImuMsg

        function DummyNode(dummy_pub_ip::String, dummy_pub_port::String, rate::Float64)
            dummyNodeIO = Hg.NodeIO(ZMQ.Context(1); rate=rate)
            rate = rate; 
            ctx = ZMQ.Context(1)

            should_finish = false 

            imu_msg = ImuMsg(x=0.0, y=0.0, z=0.0, time=0.0)
            dummy_pub = Hg.ZmqPublisher(ctx, dummy_pub_ip, dummy_pub_port)
            Hg.add_publisher!(dummyNodeIO, imu_msg, dummy_pub)
            
            return new(dummyNodeIO, rate, should_finish, imu_msg)
        end 

        function Hg.compute(node::DummyNode)
            nodeio = Hg.getIO(node)

            node.imu_msg.x += 1 
            
            Hg.publish.(nodeio.pubs)
        end 
    end 

    

    function main(; rate=100.0, debug=false)
        test_ip = "127.0.0.1"
        test_port = "5556"
        dummy_pub_node = DummyNode(test_ip, test_port, 100.0)
        return dummy_pub_node
    end 

end 