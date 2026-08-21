"""Small OpenCV-to-ROS image conversion without cv_bridge."""

import numpy as np
from sensor_msgs.msg import Image


def bgr_to_image_message(frame: np.ndarray, stamp, frame_id: str) -> Image:
    """Copy a uint8 BGR frame into a tightly packed sensor_msgs/Image."""
    if not isinstance(frame, np.ndarray):
        raise TypeError('frame must be a numpy array')
    if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('frame must have shape HxWx3 and dtype uint8')

    contiguous = np.ascontiguousarray(frame)
    height, width, _ = contiguous.shape
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = height
    message.width = width
    message.encoding = 'bgr8'
    message.is_bigendian = 0
    message.step = width * 3
    message.data = contiguous.tobytes()
    return message
