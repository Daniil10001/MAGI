#!/usr/bin/env python3
from threading import Lock

import rclpy, time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from magi.srv import UARTCrt,  UARTUnreg, SendData, BoolSRV, URCTRD
from std_msgs.msg import UInt8, Bool
from magi.msg import UARTInstance, Data, SPIInstance
from constant import STM32Buffer
from threading import Lock

class Controller(Node):

    __UARTRC:UARTInstance=None
    __UART:UARTInstance=None

    def __init__(self,data_port:str,data_baudrate:int,control_port:str,control_baudrate:int, exec):
        super().__init__('controller')
        self.exec=exec
        self.send_srv = self.create_service(SendData, 'controller/SendData', self.DataRecieved)
        self.isbusy_srv = self.create_service(BoolSRV, 'controller/IsBusy', self.BusyCheck)
        
        self.buffer=[]
        self.lk=Lock()
        self.data_lk=Lock()


        self.data_pub = self.create_publisher(Data, 'RecivedData', 600)
        
        self.uart_rq = self.create_client(UARTCrt, 'UARTCreateInstance')
        self.uartrc_rq = self.create_client(UARTCrt, 'UARTRCCreateInstance')
        

        while not self.uart_rq.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('UART service not available, waiting again...')
    
        request = UARTCrt.Request()
        request.name="ControllerUART"
        request.port=control_port
        request.baudrate=control_baudrate
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

        while not self.uartrc_rq.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('SPI service not available, waiting again...')
        
        request = UARTCrt.Request()
        request.name="ControllerUARTRC"
        request.port=data_port
        request.baudrate=data_baudrate
        future = self.uartrc_rq.call_async(request)
        self.get_logger().info('UART service call initiated.')
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if future.done():
            self.__UARTRC = future.result().inst
            self.get_logger().info("UART service responsed")
            if not future.result().result:
                 raise Exception("UART failed to execute")
        else:
            self.get_logger().error(f'UART service call timed out after 5 seconds.')
            raise Exception("UART servies not executed in estimated time")
        
        self.uartrc_tr=self.create_client(URCTRD,f"uartrc_{self.__UART.inst_num}_tr")
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Controller successfully started.")

    def timer_callback(self):
        if not self.lk.locked():
            msg=self.ctrl_state+0xB0
            self.uart_t.publish(UInt8(data=msg))
            self.get_logger().info("transmited "+str(hex(msg)))
             
    def data_tr_cb(self,future):
        # self.get_logger().info("spi data recieved")
        self.get_logger().info("Data service responsed")
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
                rq_prep=URCTRD.Request()
                rq_prep.data_t.data=(self.buffer.pop() if\
                    self.ctrl_state&0x02 else [])
                rq_prep.rds=STM32Buffer
                self.future = self.uartrc_tr.call_async(rq_prep)
                self.future.add_done_callback(self.data_tr_cb)
                self.get_logger().info('Transfer serviece called')
            else:
                self.ctrl_state=msg.data & (0x03 if len(self.buffer)>0 else 0x01)
    
    def BusyCheck(self, request, response):
        response.result=bool(len(self.buffer)>0)
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
            node = Controller("/dev/ttyAMA2",1152000,"/dev/ttyAMA1",115200,exec)
            
            exec.add_node(node)
            exec.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()