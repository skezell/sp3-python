# ---------------------------------------------- IMPORTS ----------------------------------------------
from hub import light_matrix, button, motion_sensor, port
import runloop
import time
import motor, motor_pair, color_sensor
import sys


# ------------------------------------------------ LIBS ------------------------------------------------
class CONSTANTS:
    MOTOR_SIZE_MED = "med"
    MOTOR_SPEED_MAPPINGS = {MOTOR_SIZE_MED: 660}
    WHEEL_SIZE_SM = "sm"
    WHEEL_SIZE_LG = "lg"
    WHEEL_CIRCUM_MAPPINGS = {WHEEL_SIZE_SM: 17.5, WHEEL_SIZE_LG: 27.6}


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

    hack_values = {}

    @classmethod
    def log(cls, msg, *args, level=LOG_LEVEL_INFO, **kwargs):
        if level >= cls.logging_level:
            epoch = str(time.ticks_ms())
            msg = msg.format(*args, **kwargs)
            logging_level_name = cls.LOG_LEVEL_MAP.get(level, "0")
            formatted = "[{ts:7}]: [{level:6}]: {msg}".format(
                ts=epoch, level=logging_level_name, msg=msg
            )
            # maybe someday we can write to a log file, like a big boy program
            print(formatted)

    @classmethod
    def set_level(cls, new_level):
        # TODO: add check to make sure new level is valid
        cls.logging_level = new_level
        cls.logging_level_name = cls.LOG_LEVEL_MAP.get(
            cls.logging_level, "LEVEL NOT FOUND"
        )

    @classmethod
    def debug(cls, msg, *args, **kwargs):
        cls.log(msg, *args, level=cls.LOG_LEVEL_DEBUG, **kwargs)

    @classmethod
    def info(cls, msg, *args, **kwargs):
        cls.log(msg, *args, level=cls.LOG_LEVEL_INFO, **kwargs)

    @classmethod
    def warn(cls, msg, *args, **kwargs):
        cls.log(msg, *args, level=cls.LOG_LEVEL_WARNING, **kwargs)

    @classmethod
    def error(cls, msg, *args, **kwargs):
        cls.log(msg, *args, level=cls.LOG_LEVEL_ERROR, **kwargs)

    @classmethod
    def critical(cls, msg, *args, **kwargs):
        cls.log(msg, *args, level=cls.LOG_LEVEL_CRITICAL, **kwargs)

    # TODO: really this should be a separate metrics store class & methods
    @classmethod
    def print_new_value(cls, label, new_value, *args, level=LOG_LEVEL_DEBUG, **kwargs):
        # only print if value has changed, cuts down on output from tight loops
        hack_diff_value = cls.hack_values.get(label)
        if new_value != hack_diff_value:
            new_msg = label + " == " + str(new_value)
            cls.log(new_msg, *args, level=level, **kwargs)
            cls.hack_diff_val = new_value


class PID:

    def __init__(
        self, name, initial_input=0, setpoint=0, tuning=(0, 0, 0), limits=(0, 0)
    ):
        self.name = name
        self.tuning = tuning
        self.limits = limits

        # variables for holding the target state
        self.setpoint = setpoint
        self.last_error = self.setpoint - initial_input

        # set initial state for PID
        self.reset(initial_input=initial_input)

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


def calculate_velocity(power=75, motor_size=CONSTANTS.MOTOR_SIZE_MED):
    return int(power / 100 * lookup_max_drive_velocity(motor_size))


def cm_to_degress(cm, wheel_size):
    circumference = CONSTANTS.WHEEL_CIRCUM_MAPPINGS.get(wheel_size, 0)
    return 360 / circumference * cm


def inches_to_degrees(inches, wheel_size):
    return cm_to_degress(inches * 2.54, wheel_size)


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

        self.pids = {}

        # Robot will store any other key-value-pairs passed in
        for k, v in kwargs.items():
            setattr(self, k, v)

        # cache these to avoid calculating lots of times
        self.max_velocity = lookup_max_drive_velocity(self.drive_motor_size)
        self.default_velocity = calculate_velocity(
            power=75, motor_size=self.drive_motor_size
        )

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

    # TODO: add pivot turn & add turn method that also takes the turn type: tank or pivot
    # TODO: support greater than 180 degrees & let you specify them in 0-360 + direction
    # TODO: should we support providing a terminator also -- e.g. turn until 90 degrees or hit black???
    async def tank_turn(self, target_angle, direction="right", starting_power=40):
        # only 0 - 180 is currently supported
        # direction can only be "left" or "right"
        # power 0 - 100, percentage of max to use -- this translates to initial speed
        direction = self._validate_direction(direction)
        self._validate_target_angle(target_angle)

        await self.reset_yaw()
        target_angle = target_angle * 10  # tilt_angles are measured in decigrees
        pid_controller = self.pid_turning(target_angle, upper_limit=starting_power)
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
            controller_output = pid_controller.compute(yaw)

            # we need to convert the power value to a velocity, which is what the move_tank
            # method needs
            velocity = calculate_velocity(
                power=int(controller_output), motor_size=self.drive_motor_size
            )
            motor_pair.move_tank(self.drive_motor_pair, velocity * lvm, velocity * rvm)
            Logger.print_new_value("actual yaw", yaw)
            if abs(yaw) >= abs(target_angle):
                return True
            else:
                return False

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_yaw)
        motor_pair.stop(self.drive_motor_pair, stop=motor.HOLD)
        Logger.debug("Ending yaw == " + self.yawstr())

    # TODO: tune the 2nd & 3rd parameters ?
    def pid_turning(self, setpoint, upper_limit=40):
        # convenience method to create a pid for the tank turn maneuver
        # robots that inherit from this class may want to override this method
        # to provide different tuning data
        tuning = (0.4, 0, 0)
        limits = (14, upper_limit)
        pid = PID(
            "turning_velocity",
            initial_input=0,
            setpoint=setpoint,
            tuning=tuning,
            limits=limits,
        )

        return pid

    # TODO: tune 2nd & 3rd parameters
    def pid_drive_straight(self, setpoint=0, lower_limit=-100, upper_limit=100):
        # convenience method to create a pid for the drive straight method
        # robots that inherit from this class may want to override this method
        # to provide different tuning data
        tuning = (0.1, 0, 0)
        limits = (lower_limit, upper_limit)
        pid = PID(
            "steering_straight",
            initial_input=0,
            setpoint=setpoint,
            tuning=tuning,
            limits=limits,
        )

        return pid

    def create_pid(self, name, initial_input, setpoint, tuning, limits):
        pid = PID(
            name,
            initial_input=initial_input,
            setpoint=setpoint,
            tuning=tuning,
            limits=limits,
        )
        self.pids[name] = pid  # register it TODO: not sure this is needed ??
        return pid

    async def drive_straight_distance(
        self, cm=None, inches=None, direction="forward", power=40, acceleration=500
    ):
        if cm:
            target = cm_to_degress(cm, self.wheel_size)
        elif inches:
            target = inches_to_degrees(inches, self.wheel_size)
        else:
            raise ValueError("Either cm or inches are required.")

        motor.reset_relative_position(self.left_drive_motor, 0)

        def terminator():
            distance_traveled = abs(motor.relative_position(port.A))

            if distance_traveled > target:
                return True
            else:
                return False

        await robot.drive_straight(
            direction=direction,
            terminator=terminator,
            power=power,
            acceleration=acceleration,
        )

    async def drive_straight(
        self,
        direction="forward",
        terminator=None,
        power=40,
        acceleration=500,
    ):
        Logger.debug("Starting yaw == {}".format(self.yawstr()))
        pid = self.pid_drive_straight()
        velocity = calculate_velocity(power=power, motor_size=self.drive_motor_size)
        await self.reset_yaw()

        # TODO: change this to a validation function
        velocity = abs(velocity) if direction == "forward" else abs(velocity) * -1

        # for some reason the `steering` numbers are the opposite sign as the yaw numbers :head-shake:
        sign = -1 if direction == "forward" else 1

        def reached_destination():
            yaw = self.yaw()
            steering = pid.compute(yaw) * sign
            motor_pair.move(
                self.drive_motor_pair,
                int(steering),
                velocity=velocity,
                acceleration=acceleration,
            )

            return terminator()

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_destination)
        motor_pair.stop(self.drive_motor_pair)

        Logger.debug("Ending yaw == {}".format(self.yawstr()))

    @staticmethod
    async def reset_yaw(angle=0):
        await runloop.until(motion_sensor.stable)
        motion_sensor.reset_yaw(0)
        # wait a small amount for the code to finish running before using gyro
        time.sleep_ms(20)

    @staticmethod
    def yaw():
        return motion_sensor.tilt_angles()[0]

    @staticmethod
    def yawstr():
        return str(Robot.yaw())


class MyRobot(Robot):
    # TODO: add in the newer values rather than relying on the defaults from the parent class
    VALUES = {
        "name": "mini",
        "left_drive_motor": port.A,
        "right_drive_motor": port.B,
        "attachment_motor_ports": {"front": port.C},
        "sensor_ports": {"color": port.E},
    }

    def __init__(self):
        super().__init__(**self.VALUES)


robot = MyRobot()


# ------------------------------------------------ STEPS ------------------------------------------------
async def main():
    # write your code here
    await light_matrix.write("Hi!")


async def step1():
    print("Step 1: Static")
    print(robot.__dict__)
    port = robot.sensor_ports.get("color")
    print(color_sensor.color(port))
    print(color_sensor.rgbi(port))
    print(color_sensor.reflection(port))


async def step2():
    print("Step 2: Turning")
    await robot.tank_turn(20, direction="right", starting_power=25)
    await robot.tank_turn(90, direction="left", starting_power=40)
    # await robot.tank_turn(181, direction="right", starting_power=40)


async def step3():
    print("Step 3: Drive Straight")
    distance = 30
    await robot.drive_straight_distance(distance, direction="forward")
    await robot.drive_straight_distance(distance, direction="backward", power=100)


async def step4():
    print("Step 4: Combo Straight & Turns")
    distance = 20
    await robot.drive_straight_distance(distance)
    time.sleep_ms(100)
    await robot.tank_turn(90)
    time.sleep_ms(100)
    await robot.tank_turn(90, direction="left", starting_power=80)
    time.sleep_ms(100)
    await robot.drive_straight_distance(distance, direction="backward")


async def end_program():
    # there just doesn't seem to be a *graceful* way to exit, this will trigger
    # a stack trace, similar to raising a SystemExit error
    Logger.info("Program complete, exiting.")
    sys.exit(0)


# ---------------------------------------------- SEQUENCES ----------------------------------------------
everything = [main(), step1(), end_program()]
steps = [step1()]
test = [step2()]

# ------------------------------------------------- RUN! -------------------------------------------------
runloop.run(*steps)
