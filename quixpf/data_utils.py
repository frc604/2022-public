from dataclasses import dataclass


@dataclass
class Landmark:
    id: int
    x: float
    y: float
    z: float


@dataclass
class Vision:
    id: int
    bearing: float
    elevation: float


@dataclass
class Odometry:
    id: int
    x: float
    y: float
    theta: float
