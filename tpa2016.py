""" A direct port of the Adafruit library for the Adafruit TPA2016D2 Class D Amplifier Breakout chip
 see https://github.com/adafruit/Adafruit-TPA2016-Library for the original driver.
"""

import RPi.GPIO as GPIO
import smbus

TPA2016_SETUP = 0x1
TPA2016_SETUP_R_EN = 0x80
TPA2016_SETUP_L_EN = 0x40
TPA2016_SETUP_SWS = 0x20
TPA2016_SETUP_R_FAULT = 0x10
TPA2016_SETUP_L_FAULT = 0x08
TPA2016_SETUP_THERMAL = 0x04
TPA2016_SETUP_NOISEGATE = 0x01

TPA2016_ATK = 0x2
TPA2016_REL = 0x3
TPA2016_HOLD = 0x4
TPA2016_GAIN = 0x5
TPA2016_AGCLIMIT = 0x6
TPA2016_AGC = 0x7
TPA2016_AGC_OFF = 0x00
TPA2016_AGC_2 = 0x01
TPA2016_AGC_4 = 0x02
TPA2016_AGC_8 = 0x03

TPA2016_I2CADDR = 0x58

# ic2bus for Pi2+ use 0 for older RasPi
IC2BUS = 1

# shutdown GPIO pin
SHUTD_PIN = 23


class AMP:
    def __init__(self):
        GPIO.setwarnings(False)
        self.i2c = smbus.SMBus(IC2BUS)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SHUTD_PIN, GPIO.OUT)

    def __del__(self):
        GPIO.cleanup()

    @staticmethod
    def turnoff():
        GPIO.output(SHUTD_PIN, GPIO.LOW)

    @staticmethod
    def turnon():
        GPIO.output(SHUTD_PIN, GPIO.HIGH)

    # gain is from -28 (dB) to 30 (dB)
    def set_gain(self, gain):
        if gain > 30:
            gain = 30
        elif gain < -28:
            gain = -28

        self.write(TPA2016_GAIN, gain)

    # for querying the gain, returns in dB
    def get_gain(self):
        return self.read(TPA2016_GAIN)

    # Turn on/off right and left channels
    def enable_channel(self, right, left):

        setup = self.read(TPA2016_SETUP)

        if right:
            setup |= TPA2016_SETUP_R_EN
        else:
            setup &= ~TPA2016_SETUP_R_EN
        if left:
            setup |= TPA2016_SETUP_L_EN
        else:
            setup &= ~TPA2016_SETUP_L_EN

        self.write(TPA2016_SETUP, setup)

    def get_channel_info(self):
        setup = self.read(TPA2016_SETUP)
        return setup & TPA2016_SETUP_L_EN, setup & TPA2016_SETUP_R_EN

    # Set to OFF, 1:2, 1:4 or 1:8
    def set_agc_compression(self, compression):
        if compression > 3:
            return  # only 2 bits!

        agc = self.read(TPA2016_AGC)
        agc &= ~0x03  # mask off bottom 2 bits
        agc |= compression  # set the compression ratio.
        self.write(TPA2016_AGC, agc)

    def get_agc_compression(self):
        agc = self.read(TPA2016_AGC)
        agc &= 0x03
        return agc

    # releasevalue is between 0-31
    def set_release_control(self, release):
        if release > 0x3F:
            return  # only 6 bits!

        self.write(TPA2016_REL, release)

    def get_release_control(self):
        self.read(TPA2016_REL)

    # attackvalue ranges from 0-31
    def set_attack_control(self, attack):
        if attack > 0x3F:
            return  # only 6 bits!

        self.write(TPA2016_ATK, attack)

    def get_attack_control(self):
        self.read(TPA2016_ATK)

    # holdvalue is between 0-31
    def set_hold_control(self, hold):
        if hold > 0x3F:
            return  # only 6 bits!

        self.write(TPA2016_HOLD, hold)

    def get_hold_control(self):
        self.read(TPA2016_HOLD)

    def set_limit_level_on(self):
        agc = self.read(TPA2016_AGCLIMIT)
        agc &= ~0x80  # mask off top bit
        self.write(TPA2016_AGCLIMIT, agc)

    def set_limit_level_off(self):
        agc = self.read(TPA2016_AGCLIMIT)
        agc |= 0x80  # turn on top bit
        self.write(TPA2016_AGCLIMIT, agc)

    def get_limit_level_ctrl(self):
        agc = self.read(TPA2016_AGCLIMIT)
        return agc >> 7

    # limit ranges from 0 (-6.5dBv) to 31 (9dBV)
    def set_limit_level(self, limit):
        if limit > 31:
            return

        agc = self.read(TPA2016_AGCLIMIT)
        agc &= ~0x1F  # mask off bottom 5 bits
        agc |= limit  # set the limit level.

        self.write(TPA2016_AGCLIMIT, agc)

    def get_limit_level(self):
        agc = self.read(TPA2016_AGCLIMIT)
        agc &= 0x1F  # need bottom 5 bits
        return agc

    def set_agc_max_gain(self, maxg):
        if maxg > 12:  # max gain max is 12 (30dB)
            return

        agc = self.read(TPA2016_AGC)
        agc &= ~0xF0  # mask off top 4 bits
        agc |= (maxg << 4)  # set the max gain
        self.write(TPA2016_AGC, agc)

    def get_agc_max_gain(self):
        agc = self.read(TPA2016_AGC)
        agc &= 0xF0  # need top 4 bits
        return agc >> 4

    def write(self, address, data):
        self.i2c.write_byte_data(TPA2016_I2CADDR, address, data)

    def read(self, address):
        return self.i2c.read_byte_data(TPA2016_I2CADDR, address)
