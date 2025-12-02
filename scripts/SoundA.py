#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import collections
import numpy as np
import traceback
import pyaudio
import torch
import webrtcvad
import whisper
import sys
import scipy.signal  # Required for resampling
from magi.msg import Data
from magi.srv import BoolSRV
from std_msgs.msg import UInt8, Bool, String
import threading

FORMAT = pyaudio.paInt32  # S32_LE
CHANNELS = 2  # Stereo
RATE = 48000  # Device rate
CHUNK_DURATION_MS = 30
PADDING_DURATION_MS = 1500
CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)
NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS = int(400 / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS_END = int(200 / CHUNK_DURATION_MS)
MAX_DURATION_SECONDS = 60  # Prevent OOM by limiting max segment
MAX_CHUNKS = int(MAX_DURATION_SECONDS * 1000 / CHUNK_DURATION_MS)

class AudioAnalize(Node):
    def __init__(self, model,stream,vad):
        super().__init__('audio_analizer_node')
        self.pub = self.create_publisher(String, 'UserRequests', 100)
        self.isworkg_srv = self.create_service(BoolSRV, 'audio_analizer/IsWorking', self.IsWorking)
        self.ton_srv = self.create_service(BoolSRV, 'audio_analizer/TurnOn', self.TurnOn)
        self.model=model
        self.stream=stream
        self.vad=vad
        self.ring_buffer = collections.deque(maxlen=NUM_PADDING_CHUNKS)
        self.voiced_frames = []
        self.triggered = False
        self.thread=threading.Thread(target=self.Analizator)
        self.thread.start()
        self.working=True
    
    def IsWorking(self,request, response):
        response.result=Bool(data=self.working)
        return response

    def TurnOn(self,request, response):
        if self.working:
            response.result=Bool(data=False)
            return response
        self.thread=threading.Thread(target=self.Analizator)
        self.thread.start()
        self.working=True
        response.result=Bool(data=True)
        return response

    def Analizator(self):
        try:
            self.stream.start_stream()
            self.get_logger().info("Sound analization node created")
            while True:
                chunk = self.stream.read(CHUNK_SIZE)
                audio = np.frombuffer(chunk, np.int32).reshape(-1, CHANNELS)
                mono_audio = audio.mean(axis=1).astype(np.int32)
                # Convert to 16-bit for VAD using bitwise shift (preserves sign)
                audio_16 = (mono_audio >> 16).astype(np.int16)
                chunk_16 = audio_16.tobytes()
                is_speech = self.vad.is_speech(chunk_16, RATE)
                
                self.ring_buffer.append((chunk, is_speech))
                
                num_voiced = len([frame for frame, speech in self.ring_buffer if speech])
                #print(num_voiced,ring_buffer.maxlen)
                if not self.triggered:
                    if num_voiced > 0.6 * self.ring_buffer.maxlen:
                        print("Speech detected...")
                        self.triggered = True
                        self.voiced_frames.extend([frame for frame, _ in self.ring_buffer])
                        self.ring_buffer.clear()
                else:
                    self.voiced_frames.append(chunk)
                    if len(self.ring_buffer)!=self.ring_buffer.maxlen:
                        continue
                    if len(self.voiced_frames) >= MAX_CHUNKS:
                        print("Max duration reached, processing...")
                        buffer = b''.join(self.voiced_frames)
                        self.stream.stop_stream()
                        self.transcribe_audio(buffer)
                        self.voiced_frames = []
                        self.triggered = False
                        self.ring_buffer.clear()
                        break
                    elif num_voiced < 0.1 * self.ring_buffer.maxlen:
                        print("Speech ended.")
                        buffer = b''.join(self.voiced_frames)
                        self.stream.stop_stream()
                        self.transcribe_audio(buffer)
                        self.voiced_frames = []
                        self.triggered = False
                        self.ring_buffer.clear()
                        break
        except Exception as e:
            self.get_logger().info(str(traceback.format_exc())+"\n"+str(e))
            sys.exit(1)
        self.working=False

    def transcribe_audio(self,buffer):
        self.get_logger().info("trst")
        audio = np.frombuffer(buffer, np.int32).reshape(-1, CHANNELS).astype(np.float32) / 2147483648.0
        audio_mono = audio.mean(axis=1)  # Downmix to mono
        # Resample to 16 kHz for Whisper
        new_length = int(len(audio_mono) * 16000 / RATE)
        audio_resampled = scipy.signal.resample(audio_mono, new_length)
        result = self.model.transcribe(audio_resampled, fp16=torch.cuda.is_available(), language="en")
        text = result["text"].strip()
        if text:
            self.get_logger().info(f"Transcribed: {text}")
            self.pub.publish(String(text))

def main(args=None):
    # Target device name
    DEVICE_NAME = "mic_sv"

    # Initialize components
    vad = webrtcvad.Vad(2)  # More aggressive for better noise rejection
    pa = pyaudio.PyAudio()

    # List input devices
    print("Available input devices:")
    input_device_index = None
    for i in range(pa.get_device_count()):
        dev = pa.get_device_info_by_index(i)
        if dev['maxInputChannels'] > 0:
            print(f"Index {i}: {dev['name']}")
            if DEVICE_NAME.lower() in dev['name'].lower():
                input_device_index = i
                print(f"Selected device: {dev['name']} (Index {i})")
                break

    if input_device_index is None:
        print(f"Device containing '{DEVICE_NAME}' not found.")
        sys.exit(1)

    # Open stream
    try:
        stream = pa.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True,
                        input_device_index=input_device_index, start=False,
                        frames_per_buffer=CHUNK_SIZE)
        stream.stop_stream()
    except IOError as e:
        print(f"Error opening stream: {e}")
        sys.exit(1)

    model = whisper.load_model("small")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = model.to(device)
    rclpy.init(args=args)
    node = AudioAnalize(model,stream,vad)
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()