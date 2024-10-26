import math
import wpimath
import wpimath.geometry

import utilities.constants as constants

def convert_ticks_to_degrees(tick_count) -> float:
    # Negative to get to correct rotation direction
    return -math.degrees(tick_count * (2 * math.pi) / constants.TURN_ENCODER_TICKS)

def convert_ticks_to_radians(tick_count) -> float:
    # Negative to get to correct rotation direction 
    return -(tick_count * (2 * math.pi) / constants.TURN_ENCODER_TICKS)

# def offset_encoder(encoder_value, offset) -> float:
#     return encoder_value - offset
    
def filter_input(controller_input: float,apply_deadband:bool = True) -> float:
    controller_input_corrected = math.copysign(math.pow(controller_input,2),controller_input)

    if apply_deadband:
        return wpimath.applyDeadband(controller_input_corrected,constants.DEADBAND)
    else:
        return controller_input_corrected

def RPM_to_RadiansPerSecond(RPM) -> float: 
    return (RPM / 60) * (2 * math.pi)

def convert_rpm_to_mps(RPM:float) -> float:
    return (RPM/60)*(2*math.pi*constants.WHEEL_RADIUS)

def clamp(value, min_val, max_val):
    if(value > min_val and value < max_val):
        return value
    elif(value < min_val):
        return min_val
    else:
        return max_val