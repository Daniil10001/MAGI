#!/usr/bin/env python3
from magi.srv import SPIList, SPICrt, SPIUnreg
from magi.msg import SPIInstance
import spidev

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

    def createInstance(self, request, response):
        if len(self.availids)==0:
            response.result=False
            return response
        response.inst_num=self.availids.pop()
        try:
            spi=None#spidev.SpiDev()
            #spi.open(request.bus_num,request.device_num)
            #spi.max_speed_hz = request.speed  # 1MHz
            #spi.mode = request.mode
            self.interfaces[response.inst_num]=(request.bus_num,request.device_num, spi)
            response.result=True
        except Exception as e:
            print(e)
            self.availids.append(response.inst_num)
            response.result=False
            return response
        return response
    
    def unregisterIstance(self, request, response):
        if request.inst_num in self.interfaces.keys():
            try:
                self.interfaces[response.inst_num][2].close()
            except:
                pass #do some logs
            self.interfaces.pop(request.inst_num)
            self.availids.append(request.inst_num)
            self.availids.sort(reverse=True)
            response.result=True
        else:
            response.result=False
        return response
    
    def listInterfces(self, request, response):
        for k in self.interfaces:
            intf=SPIInstance()
            intf.inst_num=k
            intf.interface_num,intf.device_num,_=self.interfaces[k]
            response.instances.append(intf)
        response.num=len(response.instances)
        return response

    def __del__(self):
        for k in self.interfaces:
            try:
                self.interfaces[k][2].close()
            except:
                pass #do some logs


def main():
    try:
        with rclpy.init():
            srv = SPIsrv()

            rclpy.spin(srv)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()