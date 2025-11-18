#!/usr/bin/env python3
from threading import Lock

import rclpy, time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pydub import AudioSegment
import numpy as np

from magi.srv import SendData, IsBusy
from std_msgs.msg import UInt8, Bool, String
from magi.msg import Data
from asyncio import Future
from constant import STM32Buffer

class SoundManager(Node):

    def __init__(self):
        super().__init__("SoundManager")
        self.send_data = self.create_client(SendData, 'controller/SendData')
        self.check_r = self.create_client(IsBusy, 'controller/IsBusy')

        self.__filequeue=[]
        self.__datareq=SendData.Request()

        while not self.send_data.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for controller starting up...')
        
        while not self.check_r.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for controller starting up...')

        self.add_to_queue=self.create_subscription(String, "addNewFileToPlay",self.addToQueue,30)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.data_future = self.send_data.call_async(self.__datareq)


        
    def addToQueue(self,msg):
        self.__filequeue.append(msg.data)
    
    def timer_callback(self):
        if self.data_future.done():
            if self.data_future.result().result:
                if len(self.__filequeue)==0:
                     return
                try:
                    audio = AudioSegment.from_wav(self.__filequeue.pop(0))
                    audio = audio.set_channels(1)
                    audio = audio.set_frame_rate(8000)
                    audio = audio.set_sample_width(2)
                    samples = audio.get_array_of_samples()
                    samples = np.array(samples, dtype=np.int16)
                    samples = np.pad(samples,(0,STM32Buffer-samples.shape[0]%STM32Buffer), mode =  'constant', constant_values = 0).reshape(-1, STM32Buffer)
                    ldata=[Data(data=samples[1],size=STM32Buffer) for i in range(samples.shape[0])]
                    self.__datareq.data=ldata
                    self.__datareq.size=samples.shape[0]
                except Exception as e:
                     self.get_logger().error(str(e))
            future = self.check_r.call_async(IsBusy.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1)
            if future.done():
                if not future.result().result:
                     self.data_future = self.send_data.call_async(self.__datareq)
            else:
                self.get_logger().error(f'Controller service call timed out.')
                raise Exception("Controller service call timed out.")
                
        
def main():
    try:
        with rclpy.init():
            node = SoundManager()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()