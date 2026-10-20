"""Typed gripper faults used for fail-safe reporting and recovery."""
from enum import Enum


class FaultCategory(str, Enum):
    NONE = 'NONE'
    TEMPERATURE = 'TEMPERATURE'
    COMMUNICATION = 'COMMUNICATION'
    MOTOR = 'MOTOR'
    MECHANICAL = 'MECHANICAL'
    CONFIGURATION = 'CONFIGURATION'
    UNKNOWN = 'UNKNOWN'


class GripperFault(Exception):
    def __init__(self, category, message):
        super().__init__(message)
        self.fault_category = FaultCategory(category)


class MotorFault(GripperFault, IOError):
    def __init__(self, message):
        super().__init__(FaultCategory.MOTOR, message)


class MechanicalFault(GripperFault, IOError):
    def __init__(self, message):
        super().__init__(FaultCategory.MECHANICAL, message)


class EncoderFault(GripperFault, ValueError):
    def __init__(self, message):
        super().__init__(FaultCategory.MECHANICAL, message)


class ConfigurationFault(GripperFault, ValueError):
    def __init__(self, message):
        super().__init__(FaultCategory.CONFIGURATION, message)


class DeviceConfigurationFault(GripperFault, IOError):
    def __init__(self, message):
        super().__init__(FaultCategory.CONFIGURATION, message)


def category_of(error):
    category = getattr(error, 'fault_category', None)
    if category is not None:
        return FaultCategory(category)
    if isinstance(error, OSError):
        return FaultCategory.COMMUNICATION
    return FaultCategory.UNKNOWN