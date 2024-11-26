from geometry_msgs.msg import Quaternion

import numpy as np
from numpy.typing import NDArray

def rotmat2q(T: NDArray) -> Quaternion:
    # Function that transforms a 3x3 rotation matrix to a ros quaternion representation
    

    if T.shape != (3, 3):
        raise ValueError

    # TODO: implement this
    raise NotImplementedError

    return q