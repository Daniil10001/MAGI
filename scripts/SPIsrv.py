#!/usr/bin/env python3
from magi.srv import SPIList, SPICrt, SPIUnreg,SPIdrqst
from magi.msg import SPIInstance
import spidev
from threading import Lock

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SPIsrv(Node):

    availids=[i for i in range(200)][::-1]
    interfaces=dict()

    def __init__(self):
        super().__init__('SPIhandler')
        self.crt_srv = self.create_service(SPICrt, 'SPICreateInstance', self.createInstance)
        self.unreg_srv= self.create_service(SPIUnreg, 'SPIUnregInstance', self.unregisterIstance)
        self.list_srv= self.create_service(SPIList, 'SPIListInstances', self.listInterfces)
        self.trs_srv= self.create_service(SPIdrqst, 'SPITransaction', self.transaction)
        self.lk=Lock()
        self.get_logger().info("SPI srv start")

    def createInstance(self, request, response):
        with self.lk:
            if len(self.availids)==0:
               response.result=False
               return response
            k=self.availids.pop()
            try:
                spi=spidev.SpiDev()
                spi.open(request.bus_num,request.device_num)
                spi.max_speed_hz = request.speed  # 1MHz
                spi.mode = request.mode
                self.interfaces[k]=(request.name,request.bus_num,\
                                                   request.device_num, spi, Lock())
                response.inst.inst_num=k
                response.inst.name,response.inst.bus_num,\
                    response.inst.device_num,_,_=self.interfaces[k]
                response.result=True
            except Exception as e:
                self.get_logger().error(str(e))
                self.availids.append(response.inst_num)
                response.result=False
                return response
        return response
    
    def unregisterIstance(self, request, response):
        with self.lk:
            if request.inst_num in self.interfaces.keys():
                with self.interfaces[request.inst_num][4]:
                    try:
                        self.interfaces[request.inst_num][3].close()
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
                intf=SPIInstance()
                intf.inst_num=k
                intf.name,intf.bus_num,intf.device_num,_,_=self.interfaces[k]
                response.instances.append(intf)
        response.num=len(response.instances)
        return response

    def transaction(self, request, response):
        if request.inst_num in self.interfaces.keys():
            with self.interfaces[request.inst_num][4]:
                self.get_logger().info(f"SPI transfer {request.inst_num} start")
                response.data_r.data=self.interfaces[request.inst_num][3].xfer(request.data_t.data)
                response.data_r.lenght=len(response.data_r.data)
                self.get_logger().info(f"SPI transfer {request.inst_num} end")
                response.result=True
        else:
            response.result=False
        return response

    def __del__(self):
        for k in self.interfaces:
            try:
                self.interfaces[k][3].close()
            except Exception as e:
                self.get_logger().error(str(e))


def main():
    try:
            rclpy.init()
            srv = SPIsrv()

            rclpy.spin(srv)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()