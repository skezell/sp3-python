# ------------------------------- IMPORTS --------------------------------------------
from hub import light_matrix, button, motion_sensor, port
import runloop
import time
import motor, motor_pair
import hub


class Robot:
    MAX_SPEED_MED = 660
    SMALL_WHEEL_DIAMETER = 17.5  # 17.5 cm per 360 degrees
    SMALL_WHEEL_ROTATION_MULTIPLIER = 2.54 / SMALL_WHEEL_DIAMETER * 3.14

    name = "submerged"
    left_drive_port = port.A
    right_drive_port = port.B
    drive_pair = left_drive_port + right_drive_port
    front_attachment_port = port.C
    back_attachment_port = None
    light_sensor_port = None

    _default_velocity = int(MAX_SPEED_MED * 0.75)

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

        motor_pair.pair(self.drive_pair, self.left_drive_port, self.right_drive_port)

    def calculate_velocity(self, percentage=0.75):
        return int(self.MAX_SPEED_MED * percentage)

    def inches_to_degrees(self, inches):
        return int(self.SMALL_WHEEL_ROTATION_MULTIPLIER * inches)

    def cm_to_degress(self, cm):
        return int(360 / self.SMALL_WHEEL_DIAMETER * cm)

    async def tank_turn(
        self, target_angle, direction="right", velocity=_default_velocity
    ):
        await runloop.until(motion_sensor.stable)  # wait for gyro to stabilize
        motion_sensor.reset_yaw(0)
        print("Starting yaw == " + str(motion_sensor.tilt_angles()[0]))

        left_velocity = right_velocity = velocity

        # adjust for direction of turn
        if direction == "right":
            right_velocity = right_velocity * -1
        elif direction == "left":
            left_velocity = left_velocity * -1

        target_angle = target_angle * 10  # tilt_angles are measured in decigrees

        motor_pair.move_tank(self.drive_pair, left_velocity, right_velocity)

        # create a callback that will terminate the loop
        def reached_target_yaw():
            yaw = motion_sensor.tilt_angles()[0]
            if abs(yaw) >= target_angle:  # measured in decidegrees
                return True
            else:
                return False

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_yaw)
        motor_pair.stop(self.drive_pair)

        print("Ending yaw == " + str(motion_sensor.tilt_angles()[0]))

    async def drive_straight_cm(
        self, distance_cm, direction="forward", velocity=_default_velocity
    ):
        await runloop.until(motion_sensor.stable)
        motion_sensor.reset_yaw(0)

        motor.reset_relative_position(self.left_drive_port, 0)
        target_degrees = self.cm_to_degress(distance_cm)
        velocity = abs(velocity) if direction == "forward" else abs(velocity) * -1

        def reached_target_distance():
            distance_traveled = motor.relative_position(self.left_drive_port)
            print(distance_traveled)
            if abs(distance_traveled) >= target_degrees:  # measured in decidegrees
                return True
            else:
                return False

        # start motor pair
        motor_pair.move(self.drive_pair, 0, velocity=velocity)

        # wait until target angle is reached, then stop the motor pair
        await runloop.until(reached_target_distance)
        motor_pair.stop(self.drive_pair)


# ------------------------------- GLOBALS --------------------------------------------
PIXEL_LISTENER = (4, 4)
PIXEL_MAIN_START = (0, 0)
bot = Robot()


# ------------------------------- LISTENERS ------------------------------------------
# Note this only works 'correctly' because a) we only have one listener and b) we only
# care about listening for the buttons after the main program runs to completion.
# If we need true async methods, the solution is to use generators and a task runner
# that loops through the next for each generator. See "new robot basics" script for an
# example.
async def btn_listener_attachments():
    light_matrix.set_pixel(
        *PIXEL_LISTENER, 100
    )  # indicates that button listener is running
    while True:
        if button.pressed(button.LEFT):
            motor.run_for_degrees(
                bot.front_attachment_port, 5, bot.calculate_velocity(0.25)
            )
        elif button.pressed(button.RIGHT):
            motor.run_for_degrees(
                bot.front_attachment_port, -5, bot.calculate_velocity(0.25)
            )
        time.sleep_ms(100)


async def btn_listener_motors_turning():
    light_matrix.set_pixel(
        *PIXEL_LISTENER, 100
    )  # indicates that button listener is running
    while True:
        if button.pressed(button.LEFT):
            await bot.tank_turn(1, "left")
        elif button.pressed(button.RIGHT):
            await bot.tank_turn(1, "right")
        time.sleep_ms(100)


# ------------------------------- MAIN FLOW ------------------------------------------
async def main():
    # (0,0) pixel indicates that the main block has started running
    light_matrix.set_pixel(
        *PIXEL_MAIN_START, 100
    )  # indicates that main block has started

    # get attachment into grabbing position
    # motor.reset_relative_position(bot.front_attachment_port,0)
    # await motor.run_to_relative_position(bot.front_attachment_port, -390,bot.calculate_velocity(.25))

    # move forward to the blocks to be grabbed
    # motor_pair.pair(bot.drive_pair, bot.left_drive_port, bot.right_drive_port)
    # await motor_pair.move_for_degrees(bot.drive_pair,90,0,velocity=bot.calculate_velocity(.25))

    # grab the block and lift up
    # await motor.run_to_relative_position(bot.front_attachment_port, -174,bot.calculate_velocity(.25))
    # await motor.run_to_relative_position(bot.front_attachment_port, 0,bot.calculate_velocity(.25))

    # back up
    # motion_sensor.set_yaw_face(motion_sensor.FRONT)
    wait = 400
    velocity = bot.calculate_velocity(0.25)
    # time.sleep_ms(wait)
    # motion_sensor.tilt_angles()
    # motor_pair.move_tank_for_degrees(bot.drive_pair,180,velocity, -1*velocity)
    # motor_pair.move_tank(bot.drive_pair,velocity, -1 *velocity)

    # def reached_target_yaw():
    #     yaw = motion_sensor.tilt_angles()[0]
    #     print("inside the loop" + str(motion_sensor.tilt_angles()) + " " + str(abs(yaw) > 90))
    #     if abs(yaw) > 900:
    #         return True
    #     else:
    #         return False

    # await runloop.until(reached_target_yaw)
    # motor_pair.stop(bot.drive_pair)

    # await bot.tank_turn(90, "right", velocity=velocity)
    # await bot.tank_turn (45, "left", velocity=velocity)
    await bot.drive_straight_cm(13)
    await bot.tank_turn(30, "left")
    await bot.drive_straight_cm(30)
    await bot.drive_straight_cm(10, "back")

    # at the end, keep the program alive and listen for button pushes, for debugging
    await btn_listener_motors_turning()


runloop.run(main())
# runloop.run(main(), btn_listener_motors_turning())
