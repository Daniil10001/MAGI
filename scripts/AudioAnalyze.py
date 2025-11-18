#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.signal import resample
from faster_whisper import WhisperModel
from magi.msg import Data  
from constant import STM32Buffer

#AI generated need to be prooved and reworked for AI promt getting

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.subscription = self.create_subscription(
            Data,
            '/RecivedData',
            self.audio_callback,
            600
        )
        # Initialize Faster Whisper model (install via: pip install faster-whisper)
        # Choose model size: 'tiny', 'base', 'small', 'medium', 'large-v3' (larger models are more accurate but slower)
        self.model = WhisperModel('base', device='cpu', compute_type='int8')  # Use 'cuda' if GPU available
        self.get_logger().info('Offline Speech-to-Text node with Faster Whisper initialized and subscribing to audio topic.')

    def audio_callback(self, msg):
        # Extract signed int16 samples
        samples = msg.data  # Assuming msg.data is a list of int16
        if len(samples) == 0:
            self.get_logger().warn('Received empty audio data.')
            return

        # Convert to NumPy array
        audio_array = np.array(samples, dtype=np.int16)

        # Resample from 8000 Hz to 16000 Hz (required for Whisper)
        original_rate = 8000
        target_rate = 16000
        num_samples = int(len(audio_array) * target_rate / original_rate)
        audio_float = audio_array.astype(np.float32) / 32768.0
        resampled_float = resample(audio_float, num_samples)
        resampled_array = (resampled_float * 32768).clip(-32768, 32767).astype(np.int16)

        try:
            # Perform offline speech-to-text using Faster Whisper
            segments, info = self.model.transcribe(resampled_array, beam_size=5)
            # Detect language if not specified
            detected_language = info.language if info else 'unknown'
            self.get_logger().info(f'Detected language: {detected_language} (probability: {info.language_probability if info else "N/A"})')

            # Collect transcribed text
            transcribed_text = ' '.join(segment.text.strip() for segment in segments)
            self.get_logger().info(f'Transcribed text: {transcribed_text}')
            # Optionally, publish the text to another topic or process further
        except Exception as e:
            self.get_logger().error(f'Unexpected error during recognition: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()