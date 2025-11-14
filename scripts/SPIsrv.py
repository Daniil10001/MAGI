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

    def createInstance(self, request, response):
        with self.lk:
            if len(self.availids)==0:
               response.result=False
               return response
            response.inst_num=self.availids.pop()
            try:
                spi=None#spidev.SpiDev()
                #spi.open(request.bus_num,request.device_num)
                #spi.max_speed_hz = request.speed  # 1MHz
                #spi.mode = request.mode
                self.interfaces[response.inst_num]=(request.name,request.bus_num,\
                                                   request.device_num, spi, Lock())
                response.result=True
            except Exception as e:
                print(e)
                self.availids.append(response.inst_num)
                response.result=False
                return response
        return response
    
    def unregisterIstance(self, request, response):
        with self.lk:
            if request.inst_num in self.interfaces.keys():
                with self.interfaces[response.inst_num][4]:
                    try:
                        self.interfaces[response.inst_num][3].close()
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
        if self.lk:
            for k in self.interfaces:
                intf=SPIInstance()
                intf.inst_num=k
                intf.name,intf.interface_num,intf.device_num,_,_=self.interfaces[k]
                response.instances.append(intf)
        response.num=len(response.instances)
        return response

    def transaction(self, request, response):
        if request.inst_num in self.interfaces.keys():
            with self.interfaces[response.inst_num][4]:
                response.data_r=self.interfaces[response.inst_num][3].xfer(request.data_t)
        else:
            response.result=False
        return response

    def __del__(self):
        for k in self.interfaces:
            try:
                self.interfaces[k][3].close()
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