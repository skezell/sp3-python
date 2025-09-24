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
    logging_level_name = LOG_LEVEL_MAP.get(logging_level, 0)

    hack_diff_val = None

    @classmethod
    def log(cls, msg, level=LOG_LEVEL_INFO, **kwargs):
        epoch = str(time.ticks_ms())
        msg = msg.format(**kwargs)
        formatted = "[{ts:7}]: [{level:6}]: {msg}".format(
            ts=epoch, level=cls.logging_level_name, msg=msg
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


def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


class PID(object):
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=0.01,
        output_limits=(None, None),
        auto_mode=True,
        proportional_on_measurement=False,
        differential_on_measurement=True,
        error_map=None,
        time_fn=None,
        starting_output=0.0,
    ):
        """
        Initialize a new PID controller.

        :param Kp: The value for the proportional gain Kp
        :param Ki: The value for the integral gain Ki
        :param Kd: The value for the derivative gain Kd
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time: The time in seconds which the controller should wait before generating
            a new output value. The PID works best when it is constantly called (eg. during a
            loop), but with a sample time set so that the time difference between each update is
            (close to) constant. If set to None, the PID will compute a new output value every time
            it is called.
        :param output_limits: The initial output limits to use, given as an iterable with 2
            elements, for example: (lower, upper). The output will never go below the lower limit
            or above the upper limit. Either of the limits can also be set to None to have no limit
            in that direction. Setting output limits also avoids integral windup, since the
            integral term will never be allowed to grow outside of the limits.
        :param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on
            the input directly rather than on the error (which is the traditional way). Using
            proportional-on-measurement avoids overshoot for some types of systems.
        :param differential_on_measurement: Whether the differential term should be calculated on
            the input directly rather than on the error (which is the traditional way).
        :param error_map: Function to transform the error value in another constrained value.
        :param time_fn: The function to use for getting the current time, or None to use the
            default. This should be a function taking no arguments and returning a number
            representing the current time. The default is to use time.monotonic() if available,
            otherwise time.time().
        :param starting_output: The starting point for the PID's output. If you start controlling
            a system that is already at the setpoint, you can set this to your best guess at what
            output the PID should give when first calling it to avoid the PID outputting zero and
            moving the system away from the setpoint.
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.differential_on_measurement = differential_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_error = None
        self._last_input = None

        if time_fn is not None:
            # Use the user supplied time function
            self.time_fn = time_fn
        else:
            import time

            try:
                # Get monotonic time to ensure that time deltas are always positive
                self.time_fn = time.monotonic
            except AttributeError:
                # time.monotonic() not available (using python < 3.3), fallback to time.time()
                self.time_fn = time.time

        self.output_limits = output_limits
        self.reset()

        # Set initial state of the controller
        self._integral = _clamp(starting_output, output_limits)

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.

        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).

        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = self.time_fn()
        if dt is None:
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError("dt has negative value {}, must be positive".format(dt))

        if (
            self.sample_time is not None
            and dt < self.sample_time
            and self._last_output is not None
        ):
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        error = self.setpoint - input_
        d_input = input_ - (
            self._last_input if (self._last_input is not None) else input_
        )
        d_error = error - (
            self._last_error if (self._last_error is not None) else error
        )

        # Check if must map the error
        if self.error_map is not None:
            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(
            self._integral, self.output_limits
        )  # Avoid integral windup

        if self.differential_on_measurement:
            self._derivative = -self.Kd * d_input / dt
        else:
            self._derivative = self.Kd * d_error / dt

        # Compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_error = error
        self._last_time = now

        return output

    def __repr__(self):
        return (
            "{self.__class__.__name__}("
            "Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, "
            "setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, "
            "output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, "
            "proportional_on_measurement={self.proportional_on_measurement!r}, "
            "differential_on_measurement={self.differential_on_measurement!r}, "
            "error_map={self.error_map!r}"
            ")"
        ).format(self=self)

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not."""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller."""
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """
        Enable or disable the PID controller, optionally setting the last output value.

        This is useful if some system has been manually controlled and if the PID should take over.
        In that case, disable the PID by setting auto mode to False and later when the PID should
        be turned back on, pass the last output variable (the control variable) and it will be set
        as the starting I-term when the PID is set to auto mode.

        :param enabled: Whether auto mode should be enabled, True or False
        :param last_output: The last output, or the control variable, that the PID should start
            from when going from manual mode to auto mode. Has no effect if the PID is already in
            auto mode.
        """
        if enabled and not self._auto_mode:
            # Switching from manual mode to auto, reset
            self.reset()

            self._integral = last_output if (last_output is not None) else 0
            self._integral = _clamp(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).

        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if (None not in limits) and (max_output < min_output):
            raise ValueError("lower limit must be less than upper limit")

        self._min_output = min_output
        self._max_output = max_output

        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)

    def reset(self):
        """
        Reset the PID controller internals.

        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._integral = _clamp(self._integral, self.output_limits)

        self._last_time = self.time_fn()
        self._last_output = None
        self._last_input = None
        self._last_error = None


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
    # DEFAULTS = {
    #     "name": "Robot",
    #     "left_drive_motor": None,
    #     "right_drive_motor": None,
    #     "drive_motor_pair": motor_pair.PAIR_1,
    #     "drive_motor_size": CONSTANTS.MOTOR_SIZE_MED,
    #     "attachment_motor_ports": {},
    #     "sensor_ports": {},
    # }

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
        self.max_velocity = Util.lookup_max_drive_velocity(self.drive_motor_size)
        self.default_velocity = Util.calculate_velocity(0.75, self.drive_motor_size)

        self.pair_drive_motors()

    def pair_drive_motors(self):
        if self.left_drive_motor is not None and self.right_drive_motor is not None:
            msg = "Pairing motors. PAIR: {pair}, set to ({left}, {right})."
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
            Logger.debug(msg, left=self.left_drive_motor, right=self.right_drive_motor)

    def _validate_direction(self, direction: str):
        if direction is None or direction.lower() not in ["left", "right"]:
            raise ValueError("Invalid turn direction parameter " + str(direction))

        return direction.lower()

    async def tank_turn_v3(
        self, degrees_to_rotate, direction=CONSTANTS.TURN_RIGHT, starting_power=40
    ):
        pass

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

    async def tank_turn_suzanne_way(
        self, target_angle_as_circle, direction="right", power=40
    ):
        # by target_angle_as_circle, i mean 0 - 360 degrees + direction determines the turn
        # because the gyro yaw way of measuring is confusing and i might want to turn more than 180 degrees
        direction = self._validate_direction(direction)
        direction_as_int = -1 if direction == "right" else 1
        true_target = 0
        if target_angle_as_circle == 0:
            return true_target  # already there ;)
        elif target_angle_as_circle < 0:
            raise ValueError(
                "this turn does not support negative angles, specify 0-360 + direction"
            )
        elif target_angle_as_circle < 180:
            true_target = target_angle_as_circle * direction_as_int
        elif target_angle_as_circle == 180:
            true_target = target_angle_as_circle
        elif target_angle_as_circle > 180:
            print("above 180")
            return (360 - target_angle_as_circle) * direction_as_int
        return true_target

        # call alt tank turn here with adjusted angle

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
            Logger.print_new_value("yaw ", yaw)
            velocity = Util.calculate_velocity(controller_output / 100)
            motor_pair.move_tank(self.drive_motor_pair, velocity * lvm, velocity * rvm)
            if error <= 0:
                return True
            else:
                return False

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_yaw)
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
    await robot.tank_turn_alt(270, direction="right", power=25)
    # print("between " + MyRobot.yawstr())
    # await robot.tank_turn(90, direction="right", percentage=100)


async def step3():
    print("step3")
    await robot.drive_straight_cm(10)


async def step4():
    angles = [0, 45, 90, 135, 180, 225, 170, 315, 360]
    for angle in angles:
        true_angle = await robot.tank_turn_suzanne_way(angle)
        print((angle, true_angle))


async def end_program():
    try:
        Logger.info("Program complete, exiting.")
        sys.exit(0)
    except SystemExit:
        # suppress the stack trace for a regular exit
        pass


# ---------------------------------------------- SEQUENCES ----------------------------------------------
everything = [main(), step1(), end_program()]
steps = [step1(), step2()]
test = [step4()]

# ------------------------------------------------- RUN! -------------------------------------------------
runloop.run(*test)
