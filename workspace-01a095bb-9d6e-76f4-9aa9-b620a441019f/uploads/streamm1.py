import time
import subprocess
import socket
import io
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput

class DroneCameraStreamer:
    def __init__(self, resolution=(1280, 720)):
        self.resolution = resolution
        self.picam2 = Picamera2()
        self.ffmpeg_proc = None

    def initialize_camera(self):
        config = self.picam2.create_video_configuration(
            main={"size": self.resolution, "format": "YUV420"},
            controls={"AfMode": 2, "AfSpeed": 1}
        )
        self.picam2.configure(config)
        self.picam2.start()
        print(f"? Camera 3 initialized at {self.resolution}")

    def start_streaming(self):
        ffmpeg_cmd = [
            'ffmpeg', '-loglevel', 'error',
            '-f', 'h264', '-i', '-', 
            '-c:v', 'copy', 
            '-f', 'rtsp', '-rtsp_transport', 'tcp',
            'rtsp://localhost:8554/mystream'
        ]
        
        # Start FFmpeg with a pipe for stdin
        self.ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

        # FIX: Wrap the raw pipe in a BufferedWriter to satisfy Picamera2
        # This prevents the 'Must pass io.BufferedIOBase' error
        buffered_stdin = io.BufferedWriter(self.ffmpeg_proc.stdin)
        output = FileOutput(buffered_stdin)

        encoder = H264Encoder(bitrate=2500000)
        
        self.picam2.start_recording(encoder, output)
        
        print(f"?? Stream LIVE: http://{self.get_ip()}:8889/mystream")

    def get_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 1))
            return s.getsockname()[0]
        except: return '127.0.0.1'
        finally: s.close()

    def stop(self):
        if self.picam2: self.picam2.stop_recording()
        if self.ffmpeg_proc:
            self.ffmpeg_proc.stdin.close()
            self.ffmpeg_proc.terminate()

if __name__ == "__main__":
    streamer = DroneCameraStreamer()
    try:
        streamer.initialize_camera()
        streamer.start_streaming()
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.stop()
