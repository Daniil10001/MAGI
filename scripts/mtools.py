import rclpy, time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from magi.msg import Data
from magi.srv import BoolSRV
from std_msgs.msg import UInt8, Bool, String
import subprocess
#Сюда нужно добавлять действия, что должны произойти позже, после размышлений


# Actions
#------------------------------------------------

class Tools(Node):
    def __init__(self):
        super().__init__('tool_node')
        self.AfterActions=[]

    def get_songl_ist(self):
        """Return list of songs availible to play"""
        results = [
        "Bohemian Rhapsody",
        "Imagine",
        "Hotel California",
        "Stairway to Heaven",
        "Blinding Lights",
        "Shape of You",
        "Rolling in the Deep",
        "Take on Me",
        "Uptown Funk",
        "Old Town Road"
        ]
        return " ; ".join(results)

    def play_song_a(self, song):
        self.get_logger().info(song)

    def play_song(self, song: str):
        """Play song. If all is ok return nothing."""
        self.AfterActions.append(lambda: self.play_song_a(song))
        return ""

    #def make_directory(self, name: str):
    #    result = subprocess.run(['mkdir', name], capture_output=True, text=True, check=True)
    #    return "Created successfully with code "+str(result.returncode)

    def list_files(self):
        result = subprocess.run(['ls','-lah'], capture_output=True, text=True, check=True)
        return "Result is "+result.stdout

    def toolList(self):
        return [self.get_songl_ist,self.play_song,
                self.make_directory,self.list_files]

#------------------------------------------------