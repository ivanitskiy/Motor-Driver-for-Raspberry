"""
Pins
 -- VCC - input voltage (+2 to +6 V)
 -- GND - ground
 -- QA, QB, ... QH - 8 outputs
 -- SER - serial input, hooks up to the serial output of your micro controller
 -- OE - output enable (when high, outputs are in high impedance state)
 -- SRCLK - shift register clock (positive edge triggered);
            it times the input to SER
 -- SRCLR - overriding clear input (low state to clear register)
 -- RCLK - storage register clock (positive edge triggered);
           it is used to latch the register to the outputs
 -- QH' - output for communication to pass bytes on to additional chips
Unless there is a need to use additional digital
pins to control output enable or shift register clear,
connect OE to ground and SRCLR to +5V for normal operation.

pink  GPIO 4  -> 11 motor
blue  GPIO 3  -> 8  ser   //pin 14 on the 75HC595: SER
green GPIO 2  -> 4  clk   //pin 11 on the 75HC595: SRCLK
gray  GPIO 17 -> 12 latch //pin 12 on the 75HC595: RCLK
brown GPIO 27 -> 7  en    //pin 13 on the 75HC595: OE

spec for 75HC595:
SER SRCLK SRCLR RCLK OE  |  FUNCTION
 X    X     X    X    H  |Outputs QA−QH are disabled
 X    X     X    X    L  |Outputs QA−QH are enabled
 X    X     L    X    X  |Shift register is cleared
 L    ↑     H    X    X  |First stage of the shift register goes low
                         |Other stages store the data of previous stage
 H    ↑     H    X    X  |First stage of the shift register goes high
                         |Other stages store the data of previous stage
 X    X     X    ↑    X  |Shift-register data is stored in the storage register

Controlling motors:
    MOTOR1_PWM:
            True = full speed
    forward:
            MOTOR1_A == True
            MOTOR1_B == False
    backward:
            MOTOR1_A == False
            MOTOR1_B == True
    release:
            MOTOR1_A == False
            MOTOR1_B == False
"""

import RPi.GPIO as GPIO


class MotorController:
    """ Base class for motors
    """
    # motor's pin on register
    MOTOR1_A = 2
    MOTOR1_B = 3
    MOTOR2_A = 1
    MOTOR2_B = 4
    MOTOR4_A = 0
    MOTOR4_B = 6
    MOTOR3_A = 5
    MOTOR3_B = 7
    # GPIO pins on raspberry for register
    MOTORLATCH = 17  # gray
    MOTORENABLE = 27  # brown
    MOTORDATA = 3  # blue
    MOTORCLK = 2  # green
    # GPIO pins on raspberry for motors
    MOTOR1_PWM = 4  # pink
    MOTOR2_PWM = 0
    MOTOR3_PWM = 0
    MOTOR4_PWM = 0

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.cleanup()
        # setting register's pin:
        GPIO.setup(MotorController.MOTORLATCH, GPIO.OUT)
        GPIO.setup(MotorController.MOTORENABLE, GPIO.OUT)
        GPIO.setup(MotorController.MOTORDATA, GPIO.OUT)
        GPIO.setup(MotorController.MOTORCLK, GPIO.OUT)
        # setting motor's pin:
        GPIO.setup(MotorController.MOTORCLK, GPIO.OUT)

        self.latch_state = 0
        self.latch_tx()  # reset
        GPIO.output(MotorController.MOTORENABLE, GPIO.LOW)

    def latch_tx(self):
        """Latch registers
        """
        GPIO.output(MotorController.MOTORLATCH, GPIO.LOW)
        GPIO.output(MotorController.MOTORDATA, GPIO.LOW)
        for i in xrange(8):
            GPIO.output(MotorController.MOTORCLK, GPIO.LOW)
            if self.latch_state & self._bv(7-i):
                GPIO.output(MotorController.MOTORDATA, GPIO.HIGH)
            else:
                GPIO.output(MotorController.MOTORDATA, GPIO.LOW)
            GPIO.output(MotorController.MOTORCLK, GPIO.HIGH)
        GPIO.output(MotorController.MOTORLATCH, GPIO.HIGH)

    def _bv(self, _bite):
        """bitwise helper
        """
        return (1 << _bite)


class DCMotor(MotorController):
    """Class for DC motors"""
    def __init__(self, motor=1):
        """ initialize motor
        """
        self.motor = motor
        super(DCMotor, self).__init__()

        if self.motor == 1:
            # set both motor pins to 0
            self.latch_state = self.latch_state & (
                     ~self._bv(DCMotor.MOTOR1_A) & ~self._bv(DCMotor.MOTOR1_B))
            self.latch_tx()
            self.init_pwm(1)
        elif (self.motor == 2):
            self.latch_state = self.latch_state & (
                     ~self._bv(DCMotor.MOTOR2_A) & ~self._bv(DCMotor.MOTOR2_B))
            self.latch_tx()
            self.init_pwm(2)
        elif (self.motor == 3):
            self.latch_state = self.latch_state & (
                     ~self._bv(DCMotor.MOTOR3_A) & ~self._bv(DCMotor.MOTOR3_B))
            self.latch_tx()
            self.init_pwm(3)
        elif (self.motor == 4):
            self.latch_state = self.latch_state & (
                     ~self._bv(DCMotor.MOTOR4_A) & ~self._bv(DCMotor.MOTOR4_B))
            self.latch_tx()
            self.init_pwm(4)

    def init_pwm(self, motor=1):
        """setting frequentfor motor1
        todo: implement PWM hardware or software
        """
        if motor == 1:
            GPIO.setup(DCMotor.MOTOR1_PWM, GPIO.OUT)
            # GPIO.output(DCMotor.MOTOR1_PWM, GPIO.HIGH)
        elif motor == 2:
            GPIO.setup(DCMotor.MOTOR2_PWM, GPIO.OUT)
            # GPIO.output(DCMotor.MOTOR2_PWM, GPIO.HIGH)
        elif motor == 3:
            GPIO.setup(DCMotor.MOTOR3_PWM, GPIO.OUT)
            # GPIO.output(DCMotor.MOTOR3_PWM, GPIO.HIGH)
        elif motor == 4:
            GPIO.setup(DCMotor.MOTOR4_PWM, GPIO.OUT)
            # GPIO.output(DCMotor.MOTOR4_PWM, GPIO.HIGH)

    def run(self, cmd=1):
        """ run the motor
        1  :  forward
        2  :  backward
        -1 :  release
        """
        if self.motor == 1:
            motorpin_a = DCMotor.MOTOR1_A
            motorpin_b = DCMotor.MOTOR1_B
        elif (self.motor == 2):
            motorpin_a = DCMotor.MOTOR2_A
            motorpin_b = DCMotor.MOTOR2_B
        elif (self.motor == 3):
            motorpin_a = DCMotor.MOTOR3_A
            motorpin_b = DCMotor.MOTOR3_B
        elif (self.motor == 4):
            motorpin_a = DCMotor.MOTOR4_A
            motorpin_b = DCMotor.MOTOR4_B

        if cmd == 1:  # FORWARD
            self.latch_state = self.latch_state | self._bv(motorpin_a)
            self.latch_state = self.latch_state & ~self._bv(motorpin_b)
            self.latch_tx()
        elif (cmd == 2):  # BACKWARD
            self.latch_state = self.latch_state & ~self._bv(motorpin_a)
            self.latch_state = self.latch_state | self._bv(motorpin_b)
            self.latch_tx()
        elif (cmd == -1):  # RELEASE
            self.latch_state = self.latch_state & ~self._bv(motorpin_a)
            self.latch_state = self.latch_state & ~self._bv(motorpin_b)
            self.latch_tx()
