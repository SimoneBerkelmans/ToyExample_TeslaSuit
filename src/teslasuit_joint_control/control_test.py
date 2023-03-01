from dataclasses import dataclass
from teslasuit_sdk.ts_mapper import TsBone2dIndex
import numpy as np
from src.teslasuit_joint_control.suit_handler import Teslasuit
import easygui

@dataclass
class Sensor():
    sensor_index: TsBone2dIndex
    qd: [0]


@dataclass
class SensorModel():
    def __init__(self) -> None:
        self.sensor1 = Sensor(1)
        self.sensor2 = Sensor(2)


class SensorUpdator():

    def __init__(self, suit: Teslasuit, model: SensorModel):
        self.suit = suit
        self.suit.start_mocap_streaming()
        self.model = model

    def update_rawdata(self):
        for sensor in [self.model.__dict__[key]
                      for key in iter(self.model.__dict__)]:
            qd = self.suit.get_gyro(sensor.sensor_index)
            sensor.qd = qd[0]




