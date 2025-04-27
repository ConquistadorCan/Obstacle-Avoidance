from enum import Enum

class LogStatusEnum(Enum):
    WAITING = "🟡 [WAITING]"
    SUCCESS = "🟢 [SUCCESS]"
    ERROR = "🔴 [ERROR]"
    WARNING = "🟠 [WARNING]"
    INFO = "ℹ️ [INFO]"
