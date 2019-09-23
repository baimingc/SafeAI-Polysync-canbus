#!/usr/bin/python3
"""Usage: oscc-check.py (-V <vehicle>) [-hdelv] [-b <bustype>] [-c <channel>]

Options:
    -h --help                            Display this information
    -V <vehicle>, --vehicle <vehicle>    Specify your vehicle. Required.
                                         (kia_soul_ev / kia_soul_petrol / kia_niro)
    -d --disable                         Disable modules only, no further checks (overrides enable)
    -e --enable                          Enable modules only, no further checks
    -l --loop                            Repeat all checks, run continuously
    -b --bustype <bustype>               CAN bus type [default: socketcan_native]
                                         (for more see https://python-can.readthedocs.io/en/2.1.0/interfaces.html)
    -c <channel>, --channel <channel>    Specify CAN channel, [default: can0]
    -v --version                         Display version information
"""

# This module lets us sleep intermittently. We're not in a hurry and want to see how the car behaves
# when commands are spaced out a bit.
import time

# This module makes it easier to print colored text to stdout
# `Fore`` is used to set the color and `Style`` is used to reset to default.
import colorama
from colorama import Fore, Style

# This is a requirement for this tool's command line argument handling.
from docopt import docopt

# These are the local modules you would import if you wanted to use this tool as a library rather
# than an executable.
from oscccan.canbus import CanBus
from oscccan.canbus import Report
from oscccan import OsccModule
import numpy as np
import matplotlib.pyplot as plt
import pickle
import collections
import pygame
import random
import math
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_BACKQUOTE
from pygame.locals import K_BACKSPACE
from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_TAB
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_c
from pygame.locals import K_d
from pygame.locals import K_h
from pygame.locals import K_m
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_w
from pygame.locals import K_MINUS
from pygame.locals import K_EQUALS

class DebugModules(object):
    """
    The 'DebugModules' class contains references to each of the OsccModules,
    brake, steering, and throttle. It is used to manage a majority of the stdout reporting this
    tool relies on.
    """

    def __init__(self, bus, brake, steering, throttle):
        """
        Initialize references to modules and CAN bus as well as the 'last_measurement' variable
        that allows this class to track whether expected increases and decreases occurred.
        """
        self.bus = bus
        self.brake = brake
        self.steering = steering
        self.throttle = throttle
        self.last_measurement = None

    def enable(self):
        """
        Enable all OSCC modules.
        """

        while True:

            success = self.enable_module(self.brake)

            if not success:
                continue

            success = self.enable_module(self.steering)

            if not success:
                continue

            success = self.enable_module(self.throttle)

            if not success:
                continue

            break

    def disable(self):
        """
        Disable all OSCC modules.
        """
        self.disable_module(self.brake)
        self.disable_module(self.steering)
        self.disable_module(self.throttle)

    def enable_module(self, module):
        """
        Enable a single OSCC modules. Print status, success and failure reports.
        """

        print(
            Fore.MAGENTA + ' status: ',
            Style.RESET_ALL,
            'attempting to enable',
            module.module_name,
            'module')

        # Attempt to enable the module parameter. Under the hood, this sends the enable brakes CAN
        # frame to OSCC/DriveKit over its CAN gateway.
        self.bus.enable_module(module)

        # Verify the module parameter is enabled by listening to the OSCC/DriveKit CAN gateway for
        # a status message that confirms it. Set the `success` flag so we can report and handle
        # failure
        success = self.bus.check_module_enabled_status(
            module,
            expect=True)

        if success:
            print(Fore.GREEN + ' success:', Style.RESET_ALL,
                  module.module_name, 'module enabled')
        else:
            print(
                Fore.RED +
                ' error:  ',
                Style.RESET_ALL,
                module.module_name,
                'module could not be enabled')

        self.bus.reading_sleep()

        return success

    def disable_module(self, module):
        """
        Disable a single OSCC modules. Print status, success and failure reports.
        """

        print(
            Fore.MAGENTA + ' status: ',
            Style.RESET_ALL,
            'attempting to disable',
            module.module_name,
            'module')

        # Attempt to disable the module parameter. Under the hood, this sends the disable brakes CAN
        # frame to OSCC/DriveKit over its CAN gateway.
        self.bus.disable_module(module)

        self.bus.reading_sleep()

        # Verify the module parameter is disabled by listening to the OSCC/DriveKit CAN gateway for
        # a status message that confirms it. Set the `success` flag so we can report and handle
        # failure
        success = self.bus.check_module_enabled_status(
            module,
            expect=False)

        if success:
            print(Fore.GREEN + ' success:', Style.RESET_ALL,
                  module.module_name, 'module disabled')
            return True
        else:
            print(
                Fore.RED + ' error:  ',
                Style.RESET_ALL,
                module.module_name,
                'module could not be disabled')
            return False

    def command_brake_module(self, cmd_value, expect=None):
        self.bus.send_command(self.brake, cmd_value, timeout=1.0)
        
        #self.bus.reading_sleep(duration = 0.02)

    def read_brake_module(self):
        report = self.bus.check_brake_pressure(timeout=1.0)

        self.last_measurement = report.value

        #self.bus.reading_sleep(duration = 0.02)
        return report.value

    def command_steering_module(self, cmd_value, expect=None):
    #	self.bus.reading_sleep(duration = 0.01)
        self.bus.send_command(self.steering, cmd_value, timeout=1.0)

        #self.bus.reading_sleep(duration = 0.01)

    def read_steering_module(self):
        self.bus.reading_sleep(duration = 0.01)
        report = self.bus.check_steering_wheel_angle(timeout=1.0)
        self.last_measurement = report.value
        #self.bus.reading_sleep(duration = 0.01)
        return report.value

    def command_throttle_module(self, cmd_value, expect=None):

        self.bus.send_command(self.throttle, cmd_value, timeout=1.0)
        #self.bus.reading_sleep(duration = 0.02)

    def read_throttle_module(self):
        report = self.bus.check_wheel_speed(timeout=1.0)

        self.last_measurement = report.value

        self.bus.reading_sleep(duration = 0.02)
        return report.value

class PID:
    """PID Controller
    """

    def __init__(self, P=5e-3, I=5e-5, D=0.0, output_min = -1e3, output_max = 1e3, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.output_min = output_min
        self.output_max = output_max
        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, setPoint, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = setPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if self.output > self.output_max:
                self.output = self.output_max
            elif self.output < self.output_min:
                self.output = self.output_min
    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


def check_vehicle_arg(arg):
    """
    Sanity check the optional vehicle argument.
    """

    vehicles = ['kia_soul_ev', 'kia_soul_petrol', 'kia_niro', None]
    if arg not in vehicles:
        raise ValueError('Unable to target vehicle',
                         arg + '. Options are', vehicles)

done = False
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

width = 200
height = 200


class Steer:
    def __init__(self, screen, x=100, y=100, color=BLACK, radius=80):
        self.color = color
        self.radius = radius
        self.center = [x, y]
        self.screen = screen

    def show(self, angle = 0):
        pygame.draw.circle(self.screen, self.color, [100, 100], self.radius, 5)
        pygame.draw.circle(self.screen, self.color, self.center, 20)
        angle = np.pi*angle/180
        x1 = -np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) + self.center[0]
        y1 = -np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) * np.tan(angle) + self.center[1]
        x2 = np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) + self.center[0]
        y2 = np.sqrt(self.radius ** 2 / (1 + np.tan(angle) ** 2)) * np.tan(angle) + self.center[1]
        R = np.array([[0,1],[-1,0]])
        # coord_origin = np.array([x1-self.center[0], y1-self.center[1]])
        # coord = np.matmul(R, coord_origin) + np.array(self.center)
        # coord = list(coord)
        coord = [self.center[0] + self.radius*np.cos(angle + np.pi/2), self.center[1] + self.radius*np.sin(angle + np.pi/2)]
        pygame.draw.line(self.screen, self.color,[x1,y1], [x2,y2], 10)
        pygame.draw.line(self.screen, self.color, self.center, coord, 10)

def main(args):
    # if args['--version']:
    #     print('oscc-check 0.0.2')
    #     return

    # check_vehicle_arg(args['--vehicle'])

    bus = CanBus(
        vehicle=args['--vehicle'],
        bustype=args['--bustype'],
        channel=args['--channel'])

    brakes = OsccModule(base_arbitration_id=0x70, module_name='brake')
    steering = OsccModule(base_arbitration_id=0x80, module_name='steering')
    throttle = OsccModule(base_arbitration_id=0x90, module_name='throttle')

    modules = DebugModules(bus, brakes, steering, throttle)

    # Initialize module for printing colored text to stdout
    colorama.init()

    if args['--disable']:
        modules.disable()
        return
    elif args['--enable']:
        modules.enable()
        return

    # Each section or step of the following loop is distinguished from the next by this separator.
    # The output begins with this separator for visually consistent output.
    print("|Enable Modules -----------------------------------------------------------------|")

    modules.enable()



    screen = pygame.display.set_mode([width, height])
    pygame.display.set_caption("Autonomous agents")
    clock = pygame.time.Clock()
    steer = Steer(screen=screen)

    kp = 0.5e-2#0.375e-2
    ki = 2e-3#1.5e-3
    kd = 0e-3
    pid = PID(P = kp, I = ki, D = kd, output_min=-0.33, output_max=0.33)
    pid.setWindup(100)

    t_start = time.time()

    current_angle_deque = collections.deque(maxlen=2)
    t_last = t_start
    
    target_angle = modules.read_steering_module()
    throttle_command = 0
    brake_command = 0

    while True:
        t_now = time.time()
        print(" frequency: ", 1/(t_now-t_last))
        t_last = t_now
        mainScreenPressed = pygame.key.get_pressed()

        if mainScreenPressed[K_a] and target_angle > -360:
            target_angle -= 3
        if mainScreenPressed[K_d] and target_angle < 360:
            target_angle += 3

        current_angle = modules.read_steering_module()
        current_angle_deque.append(current_angle)

        # clock.tick(30)
        screen.fill(WHITE)


        steer.show(angle=current_angle)
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            if mainScreenPressed[K_q]:
                exit()


        pid.update(target_angle, np.mean(current_angle_deque))
        control = pid.output
        modules.command_steering_module(control)

        if mainScreenPressed[K_w]:
            throttle_command = min(throttle_command + 8e-3, 0.2)#0.2
        else:
            throttle_command = 0
        
        if mainScreenPressed[K_s]:
            brake_command = min(brake_command + 16e-3, 0.3) #0.3
        else:
            brake_command = 0
            
        modules.command_throttle_module(throttle_command)
        modules.command_brake_module(brake_command)
        

    print("|Disable Modules ----------------------------------------------------------------|")

    modules.disable()

if __name__ == "__main__":
    """
    The program's entry point if run as an executable.
    """

    main(docopt(__doc__))
