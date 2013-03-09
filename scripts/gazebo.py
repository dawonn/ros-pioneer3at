#!/usr/bin/env python

import socket
import asyncore
import time
import sys
import types

from datetime import datetime

from proto.packet_pb2       import Packet
from proto.publish_pb2      import Publish
from proto.subscribe_pb2    import Subscribe
from proto.pose_pb2         import Pose
from proto.gz_string_pb2    import GzString
from proto.gz_string_v_pb2  import GzString_V
from proto.publishers_pb2   import Publishers
from proto.world_stats_pb2  import WorldStatistics


MASTER_TCP_IP   = '127.0.0.1'
MASTER_TCP_PORT = 11345

NODE_TCP_IP     = '127.0.0.1'
NODE_TCP_PORT   = 11452

TCP_BUFFER_SIZE = 8192


class gzSubscriber(asyncore.dispatcher):

    def __init__(self, publisher, callback):
        asyncore.dispatcher.__init__(self)
        
        self.publisher = publisher
        self.callback  = callback
        
        self.buffer = ""
        
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect( (self.publisher.host, self.publisher.port) )
        
        pk            = Packet()
        pk.stamp.sec  = int(time.time())
        pk.stamp.nsec = datetime.now().microsecond
        pk.type       = "sub"

        sub           = Subscribe()
        sub.topic     = self.publisher.topic
        sub.msg_type  = self.publisher.msg_type
        sub.host      = self.publisher.host
        sub.port      = self.publisher.port
        
        pk.serialized_data = sub.SerializeToString()
        self.buffer += hex(pk.ByteSize()).rjust(8) + pk.SerializeToString()
        
        asyncore.loop(timeout=0.5, count=1)
        
    def handle_connect(self):
        print "[gzSubscriber] Subscriber Connected."
        pass

    def handle_close(self):
        print "[gzSubscriber] Connection Closed."
        self.close()
        
    def writable(self):
        return (len(self.buffer) > 0)

    def handle_write(self):
        sent = self.send(self.buffer)
        self.buffer = self.buffer[sent:]
        
    def handle_read(self):
        #print "[gzSubscriber] Subscriber Read:"
        data = ""
        try:
            data = self.recv(TCP_BUFFER_SIZE)
            if data:
                msg_num = 0
                while msg_num < 10:
                    # Hard Limit for number of messages 
                    # contained within a single TCP packet
                    # to eliminate inf-loop possiblity
                    msg_num += 1
                    
                    # Parse Protobuf Packet
                    pk_len = int(data[:8],16)
                    self.callback(data[8:(pk_len+8)])
              
                    # Process the next packet if available
                    if len(data) > (pk_len+8):
                        data = data[(pk_len+8):]
                    else:
                        break
        except:
            #print "[gzSubscriber] handle_read error:", sys.exc_info()
            pass   
                    
                    
class gzPublisherHandler(asyncore.dispatcher):

    def __init__(self, sock):
        asyncore.dispatcher.__init__(self, sock)

        self.buffer = ""
        self.subinfo = Subscribe()
        
    def handle_read(self):
        data = self.recv(TCP_BUFFER_SIZE)
        if data:
            #try:
            msg_num = 0
            while msg_num < 100:
                # Hard Limit for number of messages 
                # contained within a single TCP packet
                # to eliminate inf-loop possiblity
                msg_num += 1
            
                # Parse Protobuf Packet
                pk_len = int(data[:8],16)            
                pk = Packet()
                pk.ParseFromString(data[8:(pk_len+8)])

                if pk.type == "sub":
                    self.subinfo.ParseFromString(pk.serialized_data)
                    #print "Subscriber Connected:\n", self.subinfo
                else:
                    print "[gzPublisherHandler] Unhandled Packet: ", pk.type, "\n", pk.serialized_data
          
                # Process the next packet if available
                if len(data) > (pk_len+8):
                    data = data[(pk_len+8):]
                else:
                    break

    def writable(self):
        #print "Connection Writable."
        return (len(self.buffer) > 0)
        
    def handle_write(self):
        sent        = self.send(self.buffer)
        self.buffer = self.buffer[sent:]

    def handle_close(self): 
        print "[gzPublisherHandler] Subscriber Connection Closed."
        self.close()
           
        
class gzPublisher(asyncore.dispatcher):

    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        
        self.subscribers = []
        
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((host, port))
        self.listen(5)
        
        asyncore.loop(timeout=0.5, count=1)
        
    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print '[gzPublisher] Incoming connection from %s' % repr(addr)
            if self.subscribers == []:
              self.subscribers = [gzPublisherHandler(sock)] 
            else:
              self.subscribers.append(gzPublisherHandler(sock))    
        
        
class gzMaster(asyncore.dispatcher):
     
    def __init__(self, host=MASTER_TCP_IP, port=MASTER_TCP_PORT):
        asyncore.dispatcher.__init__(self)
        
        self.buffer        = ""
        self.namespace     = ""
        self.publishers    = Publishers()
        self.subscriptions = []
        
        self.Pub  = gzPublisher(NODE_TCP_IP, NODE_TCP_PORT)
        
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.connect( (host, port) )
                
        self.SpinOnce()

    def handle_connect(self):
        print "[gzMaster] Master Connected."
        pass

    def handle_close(self):
        raise IOError("[gzMaster] Connection Closed.")
        self.close()

    def handle_read(self):
        #print "[gzMaster] Connection Read."
        try:
          data = self.recv(TCP_BUFFER_SIZE)
        except:
          data = ""
          print "[gzMaster] handle_read recv error:", sys.exc_info()
          
        if data:
            msg_num = 0
            while msg_num < 100:
              # Hard Limit for number of messages 
              # contained within a single TCP packet
              # to eliminate inf-loop possiblity
              msg_num += 1
              
              # Extract message length from header
              pk_len = int(data[:8],16) 
              
              # Parse Protobuf Packet
              pk = Packet()
              pk.ParseFromString(data[8:(pk_len+8)])
              
              # Handle Payloads
              if pk.type == "version_init":
                x = GzString()
                x.ParseFromString(pk.serialized_data)
                print "[gzMaster] Version String: ", x.data
                             
              elif pk.type == "topic_namepaces_init":
                x = GzString_V()
                x.ParseFromString(pk.serialized_data)
                self.namespace = x.data[1] + "/" + x.data[0]
                #print "Topic Namespace:\n", self.namespace
               
              elif pk.type == "publishers_init":
                self.publishers.ParseFromString(pk.serialized_data)
                #print "Publishers_init:\n", self.publishers
                
              elif pk.type == "publisher_add":
                x = Publish()
                x.ParseFromString(pk.serialized_data)
                self.publishers.publisher.extend([x])
                print "[gzMaster] Publisher Added:", x.topic, "[" , x.msg_type, "]"            
                
              elif pk.type == "publisher_del":
                  x = Publish()
                  x.ParseFromString(pk.serialized_data)            
                  print "[gzMaster] Publisher Deleted:", x.topic, "[" , x.msg_type, "]" 
                  
                  for p in self.publishers.publisher:
                      if  (p.topic    == x.topic
                      and  p.host     == x.host 
                      and  p.port     == x.port 
                      and  p.msg_type == x.msg_type):
                          print "Publisher Deleted: \n", p
                          p.Clear
                          break
                  print self.publishers
                  self.publishers.ParseFromString(self.publishers.SerializeToString)
                  print "============================="                  
                  print self.publishers
                  
              elif pk.type == "unsubscribe":
                x = Subscribe()
                x.ParseFromString(pk.serialized_data)
                print "[gzMaster] Unsubscribe:\n", x  
                
                for s in self.Pub.subscribers:
                    if  (s.subinfo.topic    == x.topic
                    and  s.subinfo.host     == x.host 
                    and  s.subinfo.port     == x.port 
                    and  s.subinfo.msg_type == x.msg_type):
                        print "Sub Deleted: \n", s.subinfo
                        del s
                        
              else:
                print "[gzMaster] Unhandled Packet: ", pk.type, "\n", pk.serialized_data
              
              # Process the multiple packets
              if len(data) > (pk_len+8):
                data = data[(pk_len+8):]
              else:
                break
             
    def writable(self):
        #print "Connection Writable."
        return (len(self.buffer) > 0)

    def handle_write(self):
        print "[gzMaster] Sending Data"
        sent = self.send(self.buffer)
        self.buffer = self.buffer[sent:]
    
    def WaitForMaster(self):
        # Wait for Master connection (Hack)
        if self.namespace == "":
            max_loops = 100
            while (self.namespace == "" and max_loops > 0):
                max_loops -= 1
                self.SpinOnce()
                
            if max_loops == 0:
              raise IOError("Timeout waiting for Master")
                        
    def AdvertisePublisher(self, topic, msg_type):
        
        # Insert namespace 
        self.WaitForMaster()       
        topic = topic.replace("~", self.namespace)

        for p in self.publishers.publisher:
            if (p.topic    == topic  
            and p.msg_type == msg_type
            and p.host     == NODE_TCP_IP 
            and p.port     == NODE_TCP_PORT ):
                #print "[AdvertisePublisher] Publisher Exists"
                return
        print "[AdvertisePublisher] Publisher Submitted"
      
        # Register as a Gazebo Publisher
        pk            = Packet()
        pk.stamp.sec  = int(time.time())
        pk.stamp.nsec = datetime.now().microsecond
        pk.type       = "advertise"

        pub           = Publish()
        pub.topic     = topic
        pub.msg_type  = msg_type
        pub.host      = NODE_TCP_IP
        pub.port      = NODE_TCP_PORT

        pk.serialized_data = pub.SerializeToString()
        self.buffer += hex(pk.ByteSize()).rjust(8) + pk.SerializeToString()
        
        self.SpinOnce()   
        
    def Publish(self, topic, msg):
        
        # Insert namespace
        self.WaitForMaster()
        topic = topic.replace("~", self.namespace)
  
        # Advertise publisher if needed
        self.AdvertisePublisher(topic, msg.DESCRIPTOR.full_name)
  
        # Publish Packet to all subscribers
        for s in self.Pub.subscribers:
            if (s.subinfo.topic    == topic
            and s.subinfo.msg_type == msg.DESCRIPTOR.full_name  ):                         
                print "[Publish] Found sub: ", s.subinfo.host, s.subinfo.port
                pk_pub                 = Packet()
                pk_pub.type            = msg.DESCRIPTOR.full_name
                pk_pub.stamp.sec       = int(time.time())
                pk_pub.stamp.nsec      = datetime.now().microsecond
                pk_pub.serialized_data = msg.SerializeToString()

                s.buffer += hex(msg.ByteSize()).rjust(8) + msg.SerializeToString()  
                
                self.SpinOnce()
    
    def Subscribe(self, topic, callback):
    
        self.WaitForMaster()
        
        # Insert namespace        
        topic = topic.replace("~", self.namespace)
        
        for p in self.publishers.publisher:
            if p.topic == topic:
                s = gzSubscriber(p, callback)
                self.subscriptions.append(s)
                break
    
    def SpinOnce(self, timeout=0.5):
        asyncore.loop(timeout, count=1)
        
    def Spin(self, period=0.5):
        asyncore.loop()
        

