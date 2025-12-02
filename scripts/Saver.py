#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import wave
import struct
from magi.msg import Data
import gnuplotlib as gp

class AudioSaverNode(Node):
    def __init__(self):
        super().__init__('audio_saver_node')
        self.subscription = self.create_subscription(
            Data,
            '/RecivedData',
            self.audio_callback,
            600
        )
        self.samples = np.array([], dtype=np.int16)  # Buffer to accumulate all samples
        self.sample_rate = 8000  # Matching your 8000 Hz sampling rate
        self.output_file = 'saved_audio'  # Output WAV file path
        self.num = 0
        self.get_logger().info('Audio saver node initialized and subscribing to audio topic.')

    def audio_callback(self, msg):
        # Extract signed int16 samples and append to buffer
        chunk = np.array(msg.data, dtype=np.int16)
        gp.plot( chunk,
          unset='grid', terminal='dumb 160 20' )
        # Save to WAV file using wave module
        with wave.open(self.output_file+str(self.num)+".wav", 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(self.sample_rate)
            # Pack int16 samples to bytes
            wav_data = struct.pack(f'<{len(chunk)}h', *chunk)
            self.get_logger().info(f"{len(wav_data)}")
            wav_file.writeframes(wav_data)
        self.num+=1
        
        self.get_logger().info("Saved sample to"+str(self.num)+".wav")

def main(args=None):
    rclpy.init(args=args)
    node = AudioSaverNode()
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()