# ---------------------------------------------- IMPORTS ----------------------------------------------
from hub import light_matrix, button, motion_sensor, port
import runloop
import time
import motor, motor_pair
import sys


# ------------------------------------------------ LIBS ------------------------------------------------
class CONSTANTS:
    MOTOR_SIZE_MED = "med"
    MOTOR_SPEED_MAPPINGS = {MOTOR_SIZE_MED: 660}
    WHEEL_SIZE_SM = "sm"
    WHEEL_SIZE_LG = "lg"
    WHEEL_DIAMETER_MAPPINGS = {WHEEL_SIZE_SM: 17.5}
    TURN_LEFT = "left"
    TURN_RIGHT = "right"
    VALID_TURN_DIRECTIONS = [TURN_LEFT, TURN_RIGHT]


class Logger:

    LOG_LEVEL_NOTSET = 0
    LOG_LEVEL_DEBUG = 10
    LOG_LEVEL_INFO = 20
    LOG_LEVEL_WARNING = 30
    LOG_LEVEL_ERROR = 40
    LOG_LEVEL_CRITICAL = 50

    LOG_LEVEL_MAP = {
        LOG_LEVEL_NOTSET: "NOTSET",
        LOG_LEVEL_DEBUG: "DEBUG",
        LOG_LEVEL_INFO: "INFO",
        LOG_LEVEL_WARNING: "WARN",
        LOG_LEVEL_ERROR: "ERROR",
        LOG_LEVEL_CRITICAL: "CRIT",
    }

    logging_level = LOG_LEVEL_DEBUG
    logging_level_name = LOG_LEVEL_MAP.get(logging_level, "0")

    hack_diff_val = None

    @classmethod
    def log(cls, msg, level=LOG_LEVEL_INFO, **kwargs):
        if level >= cls.logging_level:
            epoch = str(time.ticks_ms())
            msg = msg.format(**kwargs)
            logging_level_name = cls.LOG_LEVEL_MAP.get(level, "0")
            formatted = "[{ts:7}]: [{level:6}]: {msg}".format(
                ts=epoch, level=logging_level_name, msg=msg
            )
            print(formatted)

    @classmethod
    def set_level(cls, new_level):
        cls.logging_level = new_level
        cls.logging_level_name = cls.LOG_LEVEL_MAP.get(
            cls.logging_level, "LEVEL NOT FOUND"
        )

    @classmethod
    def debug(cls, msg, **kwargs):
        cls.log(msg, level=cls.LOG_LEVEL_DEBUG, **kwargs)

    @classmethod
    def info(cls, msg, **kwargs):
        cls.log(msg, level=cls.LOG_LEVEL_INFO, **kwargs)

    @classmethod
    def warn(cls, msg, **kwargs):
        cls.log(msg, level=cls.LOG_LEVEL_WARNING, **kwargs)

    @classmethod
    def error(cls, msg, **kwargs):
        cls.log(msg, level=cls.LOG_LEVEL_ERROR, **kwargs)

    @classmethod
    def critical(cls, msg, **kwargs):
        cls.log(msg, level=cls.LOG_LEVEL_CRITICAL, **kwargs)

    @classmethod
    def print_new_value(cls, msg, new_value, level=LOG_LEVEL_DEBUG, **kwargs):
        # only print if value has changed, cuts down on output from tight loops
        if new_value != cls.hack_diff_val:
            new_msg = msg + str(new_value)
            cls.log(new_msg, level=level, **kwargs)
            cls.hack_diff_val = new_value


class PID:

    def __init__(self, tuning, setpoint: float, initial_input=0, limits=(14, 100)):
        # these are the tuning variables for the PID
        self.tuning = tuning

        self.limits = limits

        # variables for holding the state
        self.setpoint = setpoint
        self.last_error = self.setpoint - initial_input

        # set initial state for PID
        self.reset()

    def reset(self, initial_input=0):
        self.error_sum = 0
        self.last_error = self.setpoint - initial_input
        self.last_calculation = time.ticks_us()

    def compute(self, input: float) -> float:
        # Calculate the time since the last call
        now = time.ticks_us()
        delta_time = now - self.last_calculation

        # Computations (part of the integrals and derivatives)
        error = self.setpoint - input
        self.error_sum += error * delta_time
        delta_error = (error - self.last_error) / delta_time

        # Calculate output
        kp, ki, kd = self.tuning
        output = kp * error + ki * self.error_sum + kd * delta_error
        output = self._clamp(output, self.limits)

        # Update values for next time
        self.last_error = error
        self.last_calculation = time.ticks_us()

        return output

    def _clamp(self, value, limits):
        lower, upper = limits
        if value is None:
            return None
        elif (upper is not None) and (value > upper):
            return upper
        elif (lower is not None) and (value < lower):
            return lower
        return value


# ------------------------------------------------ UTILS ------------------------------------------------
def lookup_max_drive_velocity(motor_size):
    return CONSTANTS.MOTOR_SPEED_MAPPINGS.get(motor_size, 0)


def calculate_velocity(percentage=0.75, motor_size=CONSTANTS.MOTOR_SIZE_MED):
    return int(percentage * lookup_max_drive_velocity(motor_size))


def cm_to_degress(cm, wheel_size):
    diameter = CONSTANTS.WHEEL_DIAMETER_MAPPINGS.get(wheel_size, 0)
    return int(360 / diameter * cm)


# -------------------------------------------- ROBOT CLASSES --------------------------------------------
class Robot:

    def __init__(
        self,
        name="Robot",
        left_drive_motor=None,
        right_drive_motor=None,
        drive_motor_size=CONSTANTS.MOTOR_SIZE_MED,
        wheel_size=CONSTANTS.WHEEL_SIZE_SM,
        attachment_motor_ports={},
        sensor_ports={},
        **kwargs
    ):
        self.name = name
        self.left_drive_motor = left_drive_motor
        self.right_drive_motor = right_drive_motor
        self.drive_motor_pair = motor_pair.PAIR_1
        self.drive_motor_size = drive_motor_size
        self.wheel_size = wheel_size
        self.attachment_motor_ports = attachment_motor_ports
        self.sensor_ports = sensor_ports

        # Robot will store any other key-value-pairs passed in
        for k, v in kwargs.items():
            setattr(self, k, v)

        # cache these to avoid calculating lots of times
        self.max_velocity = lookup_max_drive_velocity(self.drive_motor_size)
        self.default_velocity = calculate_velocity(0.75, self.drive_motor_size)

        self.pair_drive_motors()

    def pair_drive_motors(self):
        if self.left_drive_motor is not None and self.right_drive_motor is not None:
            msg = "Pair {pair} set to ({left}, {right})."
            Logger.info(
                msg,
                pair=self.drive_motor_pair,
                left=self.left_drive_motor,
                right=self.right_drive_motor,
            )
            motor_pair.pair(
                self.drive_motor_pair, self.left_drive_motor, self.right_drive_motor
            )
        else:
            msg = "Skipping pairing, left: {left}, right: {right}."
            Logger.info(msg, left=self.left_drive_motor, right=self.right_drive_motor)

    def _validate_direction(self, direction: str):
        if direction is None or direction.lower() not in ["left", "right"]:
            raise ValueError("Invalid turn direction parameter " + str(direction))

        return direction.lower()

    def _validate_target_angle(self, target_angle):
        if target_angle < 0 or target_angle > 180:
            msg = "Invalid target angle {}".format(target_angle)
            raise ValueError(msg)

    async def tank_turn_alt(
        self, target_angle, direction="right", starting_power=40, pid_controller=None
    ):
        # only 0 - 180 is currently supported
        # direction can only be "left" or "right"
        # power 0 - 100, percentage of max to use -- this translates to initial speed
        direction = self._validate_direction(direction)
        self._validate_target_angle(target_angle)

        await self.reset_yaw()
        target_angle = target_angle * 10  # tilt_angles are measured in decigrees
        pid_controller = pid_controller or self.pid_turning(
            target_angle, starting_power
        )
        Logger.debug(
            "Starting yaw == {}, target == {}".format(self.yawstr(), target_angle)
        )

        lvm = rvm = 1
        # adjust for direction of turn
        if direction == "right":
            rvm = -1  # right velocity multiplier
        elif direction == "left":
            lvm = -1  # left velocity multiplier

        # create a callback that will terminate the loop when the angle is reached
        def reached_target_yaw():
            yaw = Robot.yaw()
            error = target_angle - abs(yaw)

            controller_output = pid_controller.compute(yaw)
            # Logger.print_new_value("controller_output == ", controller_output)
            # Logger.print_new_value("error == ", error)

            # we need to convert the power value to a velocity, which is what the move_tank
            # method needs
            velocity = calculate_velocity(percentage=controller_output / 100)
            motor_pair.move_tank(self.drive_motor_pair, velocity * lvm, velocity * rvm)
            if yaw >= target_angle:
                return True
            else:
                return False

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_yaw)
        motor_pair.stop(self.drive_motor_pair, stop=motor.HOLD)
        Logger.debug("Ending yaw == " + self.yawstr())

    def pid_turning(self, setpoint, upper_limit=40):
        # convenience method to create a pid for the tank turn maneuver
        # robots that inherit from this class may want to override this method
        # to provide different tuning data
        tuning = (0.5, 0, 0)
        limits = (14, upper_limit)
        pid = PID(tuning, setpoint, initial_input=0, limits=limits)

        return pid

    @staticmethod
    async def reset_yaw(angle=0):
        await runloop.until(motion_sensor.stable)
        motion_sensor.reset_yaw(0)
        # wait a small amount for the code to finish running before using gyro
        time.sleep_ms(10)

    @staticmethod
    def yaw():
        return motion_sensor.tilt_angles()[0]

    @staticmethod
    def yawstr():
        return str(Robot.yaw())


class MyRobot(Robot):
    VALUES = {
        "name": "mini",
        "left_drive_motor": port.A,
        "right_drive_motor": port.B,
        "attachment_motor_ports": {"front": port.C},
        "sensor_ports": {"light": port.E},
    }

    def __init__(self):
        super().__init__(**self.VALUES)


robot = MyRobot()


# ------------------------------------------------ STEPS ------------------------------------------------
async def main():
    # write your code here
    await light_matrix.write("Hi!")


async def step1():
    print("Step 1: Static ")
    print(robot.__dict__)
    kp = 0.5
    ki = 0
    kd = 0
    pid = PID((kp, ki, kd), 90)

    for angle in range(0, 95):
        output = pid.compute(angle)
        print("input: {}, output: {}".format(angle, output))


async def step2():
    print("Step 2: Turning")
    # await robot.tank_turn_alt(20, direction="right", starting_power=25)
    await robot.tank_turn_alt(90, direction="left", starting_power=40)
    # await robot.tank_turn_alt(181, direction="right", starting_power=40)
    # print("between " + MyRobot.yawstr())
    # await robot.tank_turn(90, direction="right", percentage=100)


async def step3():
    print("step3")
    # await robot.drive_straight_cm(10)


async def step4():
    angles = [0, 45, 90, 135, 180, 225, 170, 315, 360]
    for angle in angles:
        true_angle = await robot.tank_turn_suzanne_way(angle)
        print((angle, true_angle))


async def end_program():
    # there just doesn't seem to be a *graceful* way to exit, this will trigger
    # a stack trace, similar to raising a SystemExit error
    Logger.info("Program complete, exiting.")
    sys.exit(0)


# ---------------------------------------------- SEQUENCES ----------------------------------------------
everything = [main(), step1(), end_program()]
steps = [step2()]
test = [step2()]

# ------------------------------------------------- RUN! -------------------------------------------------
runloop.run(*steps)
