import serial
import cv2
import numpy as np
class serialListener:
    """
    Class for reading image data from open mv
    """
    def __init__(self, port: str, width: int, height: int):
        """
        Constructor
        Args:
            port: Serial port to open
            width: expected image width
            height: expected image height
        """
        self.port = port
        self.serial = serial.Serial(
            port=self.port,
            baudrate=115200,
            timeout=1
        )
        self.width = width
        self.height = height
        self.header = b'\xAA\x55\xAA\x55'#header for receiving images
    def read_serial_image(self):
        ser = self.serial
        imageSize = self.width*self.height

        try:
            if ser.is_open:
                #checks for header so bytes can properly be received, ideally only needs to work once
                buffer = bytearray()
                while True:
                    byte = ser.read(1)
                    if not byte:
                        continue
                    buffer += byte
                    if buffer[-len(self.header):] == self.header:
                        break

                imageData = ser.read(imageSize)  # reads Width*Height beights from serial port

                if len(imageData) != imageSize:
                    print(f"Incomplete image data: {len(imageData)}/{imageSize} bytes")
                    return None
               
                
                # Convert to numpy array and then to OpenCV image
                imgArray = np.frombuffer(imageData, dtype=np.uint8)
                image = imgArray.reshape((self.height, self.width)).astype(np.uint8)
                return image
            
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            return None
        except Exception as e:
            print(f"Error reading image: {e}")
            return None
    def close(self):
        """Close serial connection"""
        if self.serial.is_open:
            self.serial.close()
            print("Serial port closed")
            
    def main():
        return None
