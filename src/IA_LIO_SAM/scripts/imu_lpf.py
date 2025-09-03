#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Imu


class Kalman:
    def __init__(self, dt: float, q_scalar: float, r_scalar: float):
        self.dt = float(dt)
        # State: [value, rate]'
        self.A = np.array([[1.0, self.dt], [0.0, 1.0]])
        self.C = np.array([[1.0, 0.0]])
        # Process/measurement noise
        self.Q = np.identity(2) * float(q_scalar)
        self.R = np.identity(1) * float(r_scalar)
        # Covariance and initial state
        self.P = np.eye(2)
        self.x = np.array([[0.0], [0.0]])

    def predict(self) -> None:
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, measurement: float) -> None:
        S = self.C @ self.P @ self.C.T + self.R
        K = (self.P @ self.C.T) / S
        y = measurement - (self.C @ self.x)
        self.x = self.x + K * y
        I = np.eye(2)
        self.P = (I - K @ self.C) @ self.P

    def value(self) -> float:
        return float(self.x[0, 0])


class ImuKalman:
    def __init__(self):
        in_topic = rospy.get_param("~in", "/ouster/imu")
        out_topic = rospy.get_param("~out", "/ouster/imu_lpf")
        fs = float(rospy.get_param("~rate", 100.0))  # Hz
        dt = 1.0 / max(fs, 1e-6)
        # Noise params (tunable)
        q = float(rospy.get_param("~q", 0.002))
        r = float(rospy.get_param("~r", 0.1))
        rospy.loginfo(
            "IMU Kalman: fs=%.1fHz dt=%.4fs q=%.4g r=%.4g in=%s out=%s",
            fs,
            dt,
            q,
            r,
            in_topic,
            out_topic,
        )

        # Per-axis Kalman filters for gyro and accel
        self.gx = Kalman(dt, q, r)
        self.gy = Kalman(dt, q, r)
        self.gz = Kalman(dt, q, r)
        self.ax = Kalman(dt, q, r)
        self.ay = Kalman(dt, q, r)
        self.az = Kalman(dt, q, r)

        self.pub = rospy.Publisher(out_topic, Imu, queue_size=50)
        self.sub = rospy.Subscriber(in_topic, Imu, self.cb, queue_size=200)

    def cb(self, m: Imu):
        # Predict all
        self.gx.predict(); self.gy.predict(); self.gz.predict()
        self.ax.predict(); self.ay.predict(); self.az.predict()

        # Update with current measurements
        self.gx.update(m.angular_velocity.x)
        self.gy.update(m.angular_velocity.y)
        self.gz.update(m.angular_velocity.z)
        self.ax.update(m.linear_acceleration.x)
        self.ay.update(m.linear_acceleration.y)
        self.az.update(m.linear_acceleration.z)

        out = Imu()
        out.header = m.header

        # Gyro filtered
        out.angular_velocity.x = self.gx.value()
        out.angular_velocity.y = self.gy.value()
        out.angular_velocity.z = self.gz.value()
        out.angular_velocity_covariance = m.angular_velocity_covariance

        # Accel filtered
        out.linear_acceleration.x = self.ax.value()
        out.linear_acceleration.y = self.ay.value()
        out.linear_acceleration.z = self.az.value()
        out.linear_acceleration_covariance = m.linear_acceleration_covariance

        # Orientation passthrough
        out.orientation = m.orientation
        out.orientation_covariance = m.orientation_covariance

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("imu_kalman")
    ImuKalman()
    rospy.spin()
