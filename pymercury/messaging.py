import zmq 

def create_pub(ctx, port, host="127.0.0.1"):
    socket = ctx.socket(zmq.PUB)
    socket.bind("tcp://%s:%s" % (host, port))
    return socket 

def create_sub(ctx, port, host="127.0.0.1"):
    socket = ctx.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE,1)
    socket.connect("tcp://%s:%s" % (host, port))
    print("tcp://%s:%s" % (host, port))
    socket.subscribe("")
    return socket
