#!/usr/bin/env python3
from magi.srv import UARTList, UARTCrt, UARTUnreg
from magi.msg import UARTInstance
from std_msgs.msg import UInt8
import serial
from threading import Lock

import time
import rclpy
import threading
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor



class UARTPortHandler(Node):

    def __init__(self,port:str,baudrate:int,inst_num:int):
        super().__init__(f"uart_{inst_num}")
        self.interface=serial.Serial(port,baudrate,timeout=0)
        self.inst_num=inst_num
        self.publisher=self.create_publisher(UInt8, f"uart_{inst_num}_recieve", 4096*2)
        self.listener=self.create_subscription(UInt8, f"uart_{inst_num}_transmit", self._transmit,4096*2)
        self.running=True
        self.get_logger().info(f"uart_{inst_num} inited")
        self.read_thread = threading.Thread(target=self._read)
        self.read_thread.start()


    def _transmit(self,msg):
        self.interface.write(msg.data.to_bytes(1))
        pass

    def _read(self):
        while self.running:
            try:
                data = self.interface.read(1000)
                if data:
                    for d in data:
                        self.publisher.publish(UInt8(data=d))
            except serial.SerialException as e:
                    self.get_logger().error(f"Serial error: {e}")
                    self.running = False
            time.sleep(1/100)

    def __del__(self):
        self.running=False


class UARTsrv(Node):

    availids=[i for i in range(200)][::-1]
    interfaces=dict()

    def __init__(self,Executor):
        super().__init__('UARThandler')
        self.Executor=Executor
        self.crt_srv = self.create_service(UARTCrt, 'UARTCreateInstance', self.createInstance)
        self.unreg_srv= self.create_service(UARTUnreg, 'UARTUnregInstance', self.unregisterIstance)
        self.list_srv= self.create_service(UARTList, 'UARTListInstances', self.listInterfces)
        self.lk=Lock()
        self.get_logger().info("UART srv start")
    
    def createInstance(self, request, response):
        with self.lk:
            if len(self.availids)==0:
               response.result=False
               return response
            k=self.availids.pop()
            try:
                UARTI=UARTPortHandler(request.port,request.baudrate,k)
                self.Executor.add_node(UARTI)
                self.interfaces[k]=(request.name,request.port,\
                                                    request.baudrate,\
                                                    UARTI, Lock())
                
                response.inst.inst_num=k
                response.inst.name,response.inst.port,\
                    response.inst.baudrate,_,_=self.interfaces[k]
                response.result=True
            except Exception as e:
                self.get_logger().error(str(e))
                self.availids.append(k)
                response.result=False
                return response
        return response
    
    def unregisterIstance(self, request, response):
        with self.lk:
            if request.inst_num in self.interfaces.keys():
                with self.interfaces[response.inst_num][4]:
                    try:
                        self.Executor.remove_node(self.interfaces[response.inst_num][3])
                        self.interfaces[response.inst_num][3].destroy_node()
                    except  Exception as e:
                        self.get_logger().error(str(e))
                    self.interfaces.pop(request.inst_num)
                    self.availids.append(request.inst_num)
                    self.availids.sort(reverse=True)
                    response.result=True
            else:
                response.result=False
        return response
    
    def listInterfces(self, request, response):
        if self.lk:
            for k in self.interfaces:
                intf=UARTInstance()
                intf.inst_num=k
                intf.name,intf.port,intf.baudrate,_,_=self.interfaces[k]
                response.instances.append(intf)
        response.num=len(response.instances)
        return response


def main():
    try:
            rclpy.init()
            Executor=MultiThreadedExecutor()
            srv = UARTsrv(Executor)
            Executor.add_node(srv)
            #rclpy.spin(srv)
            Executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()