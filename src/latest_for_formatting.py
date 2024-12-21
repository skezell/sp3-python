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
    logging_level_name = LOG_LEVEL_MAP.get(logging_level, 0)

    @classmethod
    def log(cls, msg, level=LOG_LEVEL_INFO):
        if level >= cls.logging_level:

            epoch = str(time.ticks_ms())
            formatted = "[{ts:7}]: [{level:6}]: {msg}".format(
                ts=epoch, level=cls.logging_level_name, msg=msg
            )
            print(formatted)

    @classmethod
    def set_level(cls, new_level):
        cls.logging_level = new_level
        cls.logging_level_name = cls.LOG_LEVEL_MAP.get(cls.logging_level, 0)

    @classmethod
    def debug(cls, msg):
        cls.log(msg, level=cls.LOG_LEVEL_DEBUG)

    @classmethod
    def info(cls, msg):
        cls.log(msg, level=cls.LOG_LEVEL_INFO)

    @classmethod
    def warn(cls, msg):
        cls.log(msg, level=cls.LOG_LEVEL_WARNING)

    @classmethod
    def error(cls, msg):
        cls.log(msg, level=cls.LOG_LEVEL_ERROR)

    @classmethod
    def critical(cls, msg):
        cls.log(msg, level=cls.LOG_LEVEL_CRITICAL)


class Util:
    @staticmethod
    def lookup_max_drive_velocity(motor_size):
        return CONSTANTS.MOTOR_SPEED_MAPPINGS.get(motor_size, 0)

    @staticmethod
    def calculate_velocity(percentage=0.75, motor_size=CONSTANTS.MOTOR_SIZE_MED):
        return int(percentage * Util.lookup_max_drive_velocity(motor_size))

    @staticmethod
    def cm_to_degress(cm, wheel_size):
        diameter = CONSTANTS.WHEEL_DIAMETER_MAPPINGS.get(wheel_size, 0)
        return int(360 / diameter * cm)


# -------------------------------------------- ROBOT CLASSES --------------------------------------------
class Robot:
    # currently we aren't using these directly in order to take advantage of auto-complete
    DEFAULTS = {
        "name": "Robot",
        "left_drive_motor": None,
        "right_drive_motor": None,
        "drive_motor_pair": motor_pair.PAIR_1,
        "drive_motor_size": CONSTANTS.MOTOR_SIZE_MED,
        "attachment_motor_ports": {},
        "sensor_ports": {},
    }

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

        self.max_velocity = Util.lookup_max_drive_velocity(self.drive_motor_size)
        self.default_velocity = Util.calculate_velocity(0.75, self.drive_motor_size)
        self.pair_drive_motors()

    def pair_drive_motors(self):
        Logger.debug(
            "Pairing motors PAIR:"
            + str(self.drive_motor_pair)
            + ", LEFT:"
            + str(self.left_drive_motor)
            + ", RIGHT:"
            + str(self.right_drive_motor)
        )
        if self.left_drive_motor is not None and self.right_drive_motor is not None:
            motor_pair.pair(
                self.drive_motor_pair, self.left_drive_motor, self.right_drive_motor
            )

    def _validate_direction(self, direction: str):
        if direction is None or direction.lower() not in ["left", "right"]:
            raise ValueError("Invalid direction parameter " + str(direction))

        return direction.lower()

    async def tank_turn(
        self, target_angle, direction="right", velocity=None, percentage=None
    ):
        direction = self._validate_direction(direction)
        left_velocity = right_velocity = velocity or Util.calculate_velocity(
            percentage or 0, motor_size=self.drive_motor_size
        )
        await runloop.until(motion_sensor.stable)  # wait for gyro to stabilize
        await self.reset_yaw()
        Logger.debug("Starting yaw == " + str(self.yaw()))

        # adjust for direction of turn
        if direction == "right":
            right_velocity = right_velocity * -1
        elif direction == "left":
            left_velocity = left_velocity * -1

        target_angle = target_angle * 10  # tilt_angles are measured in decigrees

        motor_pair.move_tank(self.drive_motor_pair, left_velocity, right_velocity)

        # create a callback that will terminate the loop when the angle is reached
        def reached_target_yaw():
            yaw = Robot.yaw()
            # motors are already moving, but calling this again lets us alter the velocity
            offset = 0
            adjustment = int((target_angle - yaw + offset) * 0.5 / 1000)
            print("adjustment = " + str(adjustment))
            motor_pair.move_tank(self.drive_motor_pair, left_velocity, right_velocity)
            if abs(yaw) >= target_angle:
                return True
            else:
                return False

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_yaw)
        Logger.debug("Stopping motor pair " + str(self.drive_motor_pair))
        motor_pair.stop(self.drive_motor_pair)
        Logger.debug("Ending yaw == " + self.yawstr())

    async def tank_turn_alt(self, target_angle, direction="right", power=40):
        # notes: power is expressed as number between 1-100, percentage between .01-1
        # both represent the percentage of the max speed to use
        # be careful switching back and forth between them
        direction = self._validate_direction(direction)

        p_constant = 0.5
        min_power = 14
        max_power = power

        # left_velocity = right_velocity = Util.calculate_velocity(percentage, motor_size=self.drive_motor_size)
        await self.reset_yaw()
        Logger.debug("Starting yaw == " + str(self.yaw()))

        lvm = rvm = 1
        # adjust for direction of turn
        if direction == "right":
            rvm = -1  # right velocity multiplier
        elif direction == "left":
            lvm = -1  # left velocity multiplier

        # velocity = Util.calculate_velocity(percentage, self.drive_motor_size)

        target_angle = target_angle * 10  # tilt_angles are measured in decigrees
        # motor_pair.move_tank(self.drive_motor_pair,velocity*lvm, velocity*rvm)

        # create a callback that will terminate the loop when the angle is reached
        def reached_target_yaw():
            yaw = Robot.yaw()
            error = target_angle - abs(yaw)
            controller_output = error * p_constant
            if controller_output > max_power:
                controller_output = max_power
            elif controller_output < min_power:
                controller_output = min_power

            # # motors are already moving, but calling this again lets us alter the velocity
            # offset = 0
            # adjustment = int((target_angle - yaw + offset) * .5/1000)
            print("target_angle = " + str(target_angle))
            print("yaw = " + str(yaw))
            print("error = " + str(error))
            velocity = Util.calculate_velocity(controller_output / 100)
            print("velocity = " + str(velocity))
            motor_pair.move_tank(self.drive_motor_pair, velocity * lvm, velocity * rvm)
            if error <= 0:
                return True
            else:
                return False

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_yaw)
        Logger.debug("Stopping motor pair " + str(self.drive_motor_pair))
        motor_pair.stop(self.drive_motor_pair, stop=motor.HOLD)
        Logger.debug("Ending yaw == " + self.yawstr())

    async def drive_straight_cm(self, distance_cm, direction="forward", velocity=None):
        velocity = velocity or self.default_velocity
        await self.reset_yaw()
        motor.reset_relative_position(self.left_drive_motor, 0)

        target_degrees = (
            Util.cm_to_degress(distance_cm, self.wheel_size) * 10
        )  # elsewhere, sensors are reporting decidegrees
        velocity = abs(velocity) if direction == "forward" else abs(velocity) * -1

        def reached_target_distance():
            distance_traveled = motor.relative_position(self.left_drive_motor)
            if abs(distance_traveled) >= target_degrees:  # measured in decidegrees
                return True
            else:
                return False

        # start motor pair
        motor_pair.move(self.drive_motor_pair, 0, velocity=velocity)

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_distance)
        motor_pair.stop(self.drive_motor_pair)

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
    print("step1")
    print(robot.__dict__)


async def step2():
    print("step2")
    await robot.tank_turn_alt(90, direction="right", power=25)
    # print("between " + MyRobot.yawstr())
    # await robot.tank_turn(90, direction="right", percentage=100)


async def step3():
    print("step3")
    await robot.drive_straight_cm(10)


async def all_done():
    Logger.info("Program complete, exiting.")
    sys.exit(0)


# ---------------------------------------------- SEQUENCES ----------------------------------------------
everything = [main(), step1(), all_done()]
steps = [step1(), step2()]

# ------------------------------------------------- RUN! -------------------------------------------------
runloop.run(*steps)
