from enum import Enum

class MessageTypeEnum(Enum):
    GLOBAL_POSITION = {"int_value": 33, "str_value": "GLOBAL_POSITION_INT"}
    MISSION_CURRENT = {"int_value": 42, "str_value": "MISSION_CURRENT"}
