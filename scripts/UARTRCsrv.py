#!/usr/bin/env python3
from magi.srv import UARTList, UARTCrt, UARTUnreg, URCTRD
from magi.msg import UARTInstance
from std_msgs.msg import UInt16
import serial
from threading import Lock

import time
import rclpy
import threading
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor



class UARTPortHandler(Node):

    def __init__(self,port:str,baudrate:int,inst_num:int, Executor):
        super().__init__(f"uartrc_{inst_num}")
        self.Executor=Executor
        self.interface=serial.Serial(port,baudrate,rtscts=True, timeout=0.1)
        self.inst_num=inst_num
        self.service=self.create_service(URCTRD, f"uartrc_{inst_num}_srv",)
        self.running=True
        self.get_logger().info(f"uartrc_{inst_num} inited")
        self.lk=True
        
    def transmit(self, data):
        self.interface.write(bytes(data))
        self.interface.flush()

    def transaction(self,request,response):
        with self.lk:
            self.Executor.create_task(self.transmit, request.data_t.data)
            received_bytes=self.interface.read(request.rds)
            request.data_r.data=[UInt16(received_bytes[i+1] << 8) | UInt16(received_bytes[i]) for i in range(8192)]


class UARTsrv(Node):

    availids=[i for i in range(200)][::-1]
    interfaces=dict()

    def __init__(self,Executor):
        super().__init__('UARThandler')
        self.Executor=Executor
        self.crt_srv = self.create_service(UARTCrt, 'UARTRCCreateInstance', self.createInstance)
        self.unreg_srv= self.create_service(UARTUnreg, 'UARTRCUnregInstance', self.unregisterIstance)
        self.list_srv= self.create_service(UARTList, 'UARTRCListInstances', self.listInterfces)
        self.lk=Lock()
        self.get_logger().info("UART srv start")
    
    def createInstance(self, request, response):
        with self.lk:
            if len(self.availids)==0:
               response.result=False
               return response
            k=self.availids.pop()
            try:
                UARTI=UARTPortHandler(request.port,request.baudrate,k,self.Executor)
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