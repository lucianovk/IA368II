from enum import Enum

class Mode(str, Enum):
    CLEANING = "CLEANING"
    AUTODOCKING = "AUTODOCKING"
    CHARGING = "CHARGING"
    CHARGED = "CHARGED"
    IDLE = "IDLE"  # safe default
