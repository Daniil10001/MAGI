#!/usr/bin/env python3
from threading import Lock

import rclpy, time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from magi.srv import UARTCrt, SPICrt, SPIUnreg, UARTUnreg, SPIdrqst, SendData, IsBusy
from std_msgs.msg import UInt8, Bool
from magi.msg import UARTInstance, Data, SPIInstance

class Controller(Node):

    __SPI:SPIInstance=None
    __UART:UARTInstance=None

    def __init__(self,bus:int,device:int,port:str,baudrate:int):
        super().__init__('controller')
        self.send_srv = self.create_service(SendData, 'controller/SendData', self.DataRecieved)
        self.isbusy_srv = self.create_service(IsBusy, 'controller/IsBusy', self.BusyCheck)
        
        self.buffer=[]
        self.buf_lk=Lock()
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
            __UART = future.result().inst
            self.get_logger().info("UART service responsed")
            if not future.result().result:
                 raise Exception("UART failed to execute")
        else:
            self.get_logger().error(f'UART service call timed out after 5 seconds.')
            raise Exception("UART servies not executed in estimated time")
        
        self.ctrl_state=0b00
        self.uart_t=self.create_publisher(UInt8,f"uart_{__UART.inst_num}_transmit",10)
        self.uart_r=self.create_subscription(UInt8,f"uart_{__UART.inst_num}_recieve",self.update_state,10)

        while not self.spi_rq.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SPI service not available, waiting again...')
        
        request = SPICrt.Request()
        request.name="ControllerSPI"
        request.bus_num=bus
        request.device_num=device
        future = self.spi_rq.call_async(request)
        self.get_logger().info('SPI service call initiated.')
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if future.done():
            __SPI = future.result().inst
            self.get_logger().info("SPI service responsed")
            if not future.result().result:
                 raise Exception("SPI failed to execute")
        else:
            self.get_logger().error(f'UART service call timed out after 5 seconds.')
            raise Exception("UART servies not executed in estimated time")
        
        while not self.spi_tr.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SPI transaction service not available, waiting again...')

    def update_state(self, msg):
        self.get_logger().info("recieved "+str(msg.data))
        if msg.data&0x04:
            self.get_logger().info("start transfer")
            pass #do  transfer
        self.ctrl_state=0x03&msg.data
    
    def BusyCheck(self, request, response):
         response.response=Bool(self.data_lk.locked())
         return response

    def DataRecieved(self, request, response):
        with self.data_lk:
            self.get_logger().info("recieved data")
            self.buffer=request.data
            while len(self.buffer):
                time.sleep(0.01)
            

         
    
    
    


def main():
    try:
        with rclpy.init():
            node = Controller(1,0,"/dev/ttyAMA1",9600)

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()