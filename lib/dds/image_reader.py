#
# image_reader.py
#

import socket
import cv2
import numpy as np
import math

class ImageReader:

    def __init__(self, _host, _port, timeout=5.0):
        self.sd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sd.settimeout(timeout)
        self.host = _host
        self.port = _port

    def connect(self):
        self.sd.connect( (self.host, self.port) )

    def _recv_exact(self, size):
        data = bytes()
        while len(data) < size:
            chunk = self.sd.recv(size - len(data))
            if not chunk:
                raise ConnectionError("Camera stream closed while reading image data")
            data += chunk
        return data

    def read_image(self, _width, _height):
        packet_size = int.from_bytes(self._recv_exact(4), byteorder="little")
        image_data = self._recv_exact(packet_size)

        pixel_count = _width * _height
        if packet_size == pixel_count * 4:
            image = np.frombuffer(image_data, np.uint8).reshape(_height, _width, 4)
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        elif packet_size == pixel_count * 3:
            image = np.frombuffer(image_data, np.uint8).reshape(_height, _width, 3)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            image = self._decode_dynamic_size(image_data, packet_size)
            if image.shape[1] != _width or image.shape[0] != _height:
                image = cv2.resize(image, (_width, _height), interpolation=cv2.INTER_NEAREST)
        return image

    def _decode_dynamic_size(self, image_data, packet_size):
        for channels in (4, 3):
            if packet_size % channels != 0:
                continue
            dynamic_pixel_count = packet_size // channels
            side = int(math.isqrt(dynamic_pixel_count))
            if side * side != dynamic_pixel_count:
                continue

            if channels == 4:
                image = np.frombuffer(image_data, np.uint8).reshape(side, side, 4)
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

            image = np.frombuffer(image_data, np.uint8).reshape(side, side, 3)
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        raise ValueError(f"Unexpected camera packet size: {packet_size} bytes")

    def request_image(self, dds, width, height):
        dds.publish("read_image", 0, dds.DDS_TYPE_INT)
        dds.wait("tick")
        dds.publish("read_image", 1, dds.DDS_TYPE_INT)
        dds.wait("tick")
        return self.read_image(width, height)



if __name__ == "__main__":
    from dds import *
    dds = DDS()
    dds.start()
    imr = ImageReader('localhost', 4445)
    imr.connect()
    while True:
        dds.publish('read_image', 1, DDS.DDS_TYPE_INT)
        img = imr.read_image(512, 512)
        cv2.imshow('image', img)
        k = cv2.waitKey(20)
        if k == ord('q'):
            break

    dds.stop()
