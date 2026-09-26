"""Light-level hints for the desktop camera preview; never a mapping gate."""
import numpy as np


def brightness_signal(data, width, height, stride):
    """Summarize a sampled mono8 ROS image without changing its pixels."""
    if width <= 0 or height <= 0 or stride < width or len(data) < height * stride:
        raise ValueError('Invalid mono8 image layout')
    pixels = np.frombuffer(data, dtype=np.uint8, count=height * stride).reshape(height, stride)
    sample = pixels[::8, :width:8]
    median = float(np.median(sample))
    dark_percent = float(np.mean(sample < 50) * 100)
    return dict(medianGray=median, pixelsBelow50Percent=dark_percent,
                lowLight=median < 55 and dark_percent >= 65)
