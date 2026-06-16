"""WebotsRobot РІР‚вЂќ HAL for Webots."""

import math
import sys
from pathlib import Path

_CONTROLLER_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _CONTROLLER_DIR.parent
if str(_PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(_PROJECT_DIR))

from controller import DistanceSensor, Motor, Robot
from robot_mapper.models.command import RobotCommand
from robot_mapper.models.pose import Pose
from robot_mapper.models.sensor_data import SensorData

WHEEL_RADIUS = 0.06
AXLE_LENGTH = 0.40

SONAR_ANGLES = {
    "ps0": -1.27,
    "ps1": -0.77,
    "ps2": -0.35,
    "ps3":  0.35,
    "ps4":  0.77,
    "ps5":  1.27,
}

class WebotsRobot:
    """Wraps a Webots Robot node."""

    def __init__(self) -> None:
        self._robot = Robot()
        self._timestep = int(self._robot.getBasicTimeStep())

        self._left_motor = self._get_motor("left_wheel_motor")
        self._right_motor = self._get_motor("right_wheel_motor")

        self._left_encoder = self._robot.getDevice("left_wheel_sensor")
        self._right_encoder = self._robot.getDevice("right_wheel_sensor")
        if self._left_encoder:
            self._left_encoder.enable(self._timestep)
        if self._right_encoder:
            self._right_encoder.enable(self._timestep)

        self._sonars: list[DistanceSensor] = []
        self._sonar_angles: list[float] = []
        self._sonar_names: list[str] = []

        for name in sorted(SONAR_ANGLES.keys()):
            sensor = self._robot.getDevice(name)
            if sensor is None:
                print(f"  [WebotsRobot] WARNING: sonar '{name}' not found - skipping")
                continue
            sensor.enable(self._timestep)
            self._sonars.append(sensor)
            self._sonar_names.append(name)
            self._sonar_angles.append(SONAR_ANGLES[name])

        print(f"  [WebotsRobot] Found {len(self._sonars)} sonars: {self._sonar_names}")

        self._gps = self._robot.getDevice("gps")
        if self._gps:
            self._gps.enable(self._timestep)
        else:
            print("  [WebotsRobot] WARNING: GPS not found")

        self._gyro = self._robot.getDevice("gyro")
        if self._gyro:
            self._gyro.enable(self._timestep)
        else:
            print("  [WebotsRobot] WARNING: Gyro not found")
        self._theta = 0.0

    @property
    def timestep(self) -> int:
        return self._timestep

    @property
    def dt(self) -> float:
        """Timestep in seconds."""
        return self._timestep / 1000.0

    def step(self) -> int:
        return self._robot.step(self._timestep)

    def read_pose(self) -> Pose:
        """Read pose from GPS + Gyro."""
        x, y = 0.0, 0.0

        if self._gps:
            gps_vals = self._gps.getValues()
            x, y = gps_vals[0], gps_vals[1]

        if self._gyro:
            gyro_vals = self._gyro.getValues()
            self._theta += gyro_vals[1] * self.dt

        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        return Pose(x=x, y=y, theta=self._theta)

    def read_sonars(self) -> SensorData:
        distances: list[float] = []
        for sonar in self._sonars:
            raw = sonar.getValue()
            distances.append(raw if raw > 0 else float("inf"))
        return SensorData(
            distances=distances,
            angles=list(self._sonar_angles),
        )

    def apply_command(self, cmd: RobotCommand) -> None:

        w_l = (2.0 * (-cmd.linear_velocity)
               - cmd.angular_velocity * AXLE_LENGTH) / (2.0 * WHEEL_RADIUS)
        w_r = (2.0 * (-cmd.linear_velocity)
               + cmd.angular_velocity * AXLE_LENGTH) / (2.0 * WHEEL_RADIUS)

        self._left_motor.setVelocity(w_l)
        self._right_motor.setVelocity(w_r)

    def stop(self) -> None:
        self.apply_command(RobotCommand(linear_velocity=0.0, angular_velocity=0.0))

    def _get_motor(self, name: str) -> Motor:
        motor = self._robot.getDevice(name)
        if motor is None:
            raise RuntimeError(f"Motor '{name}' not found")
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        return motor
