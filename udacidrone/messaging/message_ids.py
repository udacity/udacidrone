from enum import Enum, auto


class MsgID(Enum):
    """
    TODO: Description of each message
    """
    ANY = auto()
    STATE = auto()
    GLOBAL_POSITION = auto()
    LOCAL_POSITION = auto()
    GLOBAL_HOME = auto()
    LOCAL_VELOCITY = auto()
    CONNECTION_CLOSED = auto()
    RAW_GYROSCOPE = auto()
    RAW_ACCELEROMETER = auto()
    BAROMETER = auto()
    DISTANCE_SENSOR = auto()
    ATTITUDE = auto()
