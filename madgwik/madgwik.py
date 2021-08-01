import warnings
import numpy as np
from numpy.linalg import norm
from quaternion import Quaternion


class MadgwickAHRS:
    samplePeriod = 1/256
    quaternion = Quaternion(1, 0, 0, 0)
    beta = 0.5

    def __init__(self, sampleperiod=None, quaternion=None, beta=None):
        """
        Initialize the class with the given parameters.
        :param sampleperiod: The sample period
        :param quaternion: Initial quaternion
        :param beta: Algorithm gain beta
        :return:
        """
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta

    def update_imu(self, gyroscope, accelerometer):
        """
        Perform one update step with data from a IMU sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        """
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Gradient descent algorithm corrective step
        v = np.array([
            2*(q[1]*q[3] - q[0]*q[2]),
            2*(q[0]*q[1] + q[2]*q[3]),
            2*(q[0]**2 - q[1]**2 - q[2]**2 + q[3]**3)
        ])
        error = np.cross(v, accelerometer)

        ref = gyroscope - self.beta * error

        # Compute rate of change of quaternion
        qdot = (q.__mul__(Quaternion(0, 0.5 * ref[0], 0.5 * ref[1], 0.5 * ref[2])))

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion