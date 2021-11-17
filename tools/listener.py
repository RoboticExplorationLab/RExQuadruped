""" 
Usage:
    listener.py --topic=<topic> <host_ip> [--hz]
    listener.py <port> <host_ip> [--hz]

Options:
    --hz            print out hertz 
    --port=<port>   port number 
    --topic=<topic> topic name 
"""

import os, sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import zmq 
import time 
import toml 
from docopt import docopt
from pymercury import messaging
import msgs.messages_pb2 as messages
import google.protobuf.message_factory as message_factory

def main():
    args = docopt(__doc__)
    port = args["<port>"]
    host_ip = args["<host_ip>"]
    topic_name = args["--topic"]
    print_hz = args["--hz"]

    topic_info = toml.load("topics.toml")
    factory = message_factory.MessageFactory()

    # Load protobuf messages by name 
    msg_type = None 
    if topic_name != None:
        msg_type = topic_info["topics"][topic_name]["message_type"] 
        port = topic_info["topics"][topic_name]["port"]
        host_ip = host_ip
        # host_ip = topic_info["topics"][topic_name]["server"]
        msg_descriptor = messages.DESCRIPTOR.message_types_by_name[msg_type]
        msg = factory.GetPrototype(msg_descriptor)()


    ctx = zmq.Context() 
    sub = messaging.create_sub(ctx, port, host=host_ip)
    poller = zmq.Poller() 
    poller.register(sub, zmq.POLLIN)

    t = time.time()
    try:
        while True: 
            socks = dict(poller.poll())
            if sub in socks.keys() and socks[sub] == zmq.POLLIN:
                data = sub.recv(zmq.DONTWAIT)
                if(print_hz):
                    dt = time.time() - t
                    print( round(1/dt, 2))
                    t = time.time()
                
                if(print_hz == False and msg_type != None):
                    msg.ParseFromString(data)
                    print(msg)

    except KeyboardInterrupt:
        print("Interrupted")
             

if __name__ == "__main__":
    main()
