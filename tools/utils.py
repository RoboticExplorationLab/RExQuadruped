import os, sys 
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import toml 
from pymercury import messaging
import msgs.messages_pb2 as messages
import google.protobuf.message_factory as message_factory

def gen_message_by_name(name):
    factory = message_factory.MessageFactory()
    msg_descriptor = messages.DESCRIPTOR.message_types_by_name[name]
    msg = factory.GetPrototype(msg_descriptor)()
    return msg 
