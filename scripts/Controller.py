#!/usr/bin/env python3
from threading import Lock

import rclpy, time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from magi.srv import UARTCrt, SPICrt, SPIUnreg, UARTUnreg, SPIdrqst, SendData, IsBusy
from std_msgs.msg import UInt8, Bool
from magi.msg import UARTInstance, Data, SPIInstance
from constant import STM32Buffer
from threading import Lock

class Controller(Node):

    __SPI:SPIInstance=None
    __UART:UARTInstance=None

    def __init__(self,bus:int,device:int,speed:int,port:str,baudrate:int, exec):
        super().__init__('controller')
        self.exec=exec
        self.send_srv = self.create_service(SendData, 'controller/SendData', self.DataRecieved)
        self.isbusy_srv = self.create_service(IsBusy, 'controller/IsBusy', self.BusyCheck)
        
        self.buffer=[]
        self.lk=Lock()
        self.data_lk=Lock()


        self.data_pub = self.create_publisher(Data, 'RecivedData', 600)
        
        self.uart_rq = self.create_client(UARTCrt, 'UARTCreateInstance')
        self.spi_rq = self.create_client(SPICrt, 'SPICreateInstance')
        self.spi_tr = self.create_client(SPIdrqst, 'SPITransaction')

        while not self.uart_rq.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('UART service not available, waiting again...')
    
        request = UARTCrt.Request()
        request.name="ControllerUART"
        request.port=port
        request.baudrate=baudrate
        future = self.uart_rq.call_async(request)
        self.get_logger().info('UART service call initiated.')
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if future.done():
            self.__UART = future.result().inst
            self.get_logger().info("UART service responsed")
            if not future.result().result:
                 raise Exception("UART failed to execute")
        else:
            self.get_logger().error(f'UART service call timed out after 5 seconds.')
            raise Exception("UART servies not executed in estimated time")
        
        self.ctrl_state=0b00
        self.uart_t=self.create_publisher(UInt8,f"uart_{self.__UART.inst_num}_transmit",10)
        self.uart_r=self.create_subscription(UInt8,f"uart_{self.__UART.inst_num}_recieve",self.update_state,10)

        while not self.spi_rq.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SPI service not available, waiting again...')
        
        request = SPICrt.Request()
        request.name="ControllerSPI"
        request.bus_num=bus
        request.device_num=device
        request.speed=speed
        request.mode=1
        self.future = self.spi_rq.call_async(request)
        self.get_logger().info('SPI service call initiated.')
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5)
        if self.future.done():
            self.__SPI = self.future.result().inst
            self.get_logger().info("SPI service responsed")
            if not self.future.result().result:
                 raise Exception("SPI failed to execute")
        else:
            self.get_logger().error(f'UART service call timed out after 5 seconds.')
            raise Exception("UART servies not executed in estimated time")
        
        while not self.spi_tr.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SPI transaction service not available, waiting again...')
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Controller successfully started.")

    def timer_callback(self):
        if not self.lk.locked():
            msg=self.ctrl_state+0xB0
            self.uart_t.publish(UInt8(data=msg))
            self.get_logger().info("transmited "+str(hex(msg)))
             
    def spi_tr_cb(self,future):
        # self.get_logger().info("spi data recieved")
        self.get_logger().info("SPI service responsed")
        if not self.future.result().result:
            self.get_logger().error(f'Transfer serviece failed')
            return
        if self.ctrl_state&0x01:
            self.data_pub.publish(self.future.result().data_r)
        self.ctrl_state=0
        self.lk.release()

    def update_state(self, msg):
            if self.lk.locked():
                 return
            self.get_logger().info("recieved "+str(hex(msg.data)))
            if msg.data&0x04:
                self.lk.acquire()
                self.get_logger().info("start transfer")
                rq_prep=SPIdrqst.Request()
                rq_prep.data_t.data=(self.buffer.pop() if\
                    self.ctrl_state&0x02 else [0]*STM32Buffer)
                rq_prep.inst_num=self.__SPI.inst_num
                self.future = self.spi_tr.call_async(rq_prep)
                self.future.add_done_callback(self.spi_tr_cb)
                self.get_logger().info('Transfer serviece called')
            else:
                self.ctrl_state=msg.data & (0x03 if len(self.buffer)>0 else 0x01)
    
    def BusyCheck(self, request, response):
        response.result=Bool(len(self.buffer)>0)
        return response

    def DataRecieved(self, request, response):
        with self.data_lk:
            self.get_logger().info("recieved data")
            if len(self.buffer)>0:
                 response.result=False
                 return response
            response.result=True
            self.buffer=request.data
            return response
    


def main():
    try:
            rclpy.init()
            exec=MultiThreadedExecutor(num_threads=3)
            node = Controller(1,0,8000000,"/dev/ttyAMA1",115200,exec)
            
            exec.add_node(node)
            exec.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()