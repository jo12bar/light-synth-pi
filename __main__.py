# __main__.py
#
# Recieves data from the jo12bar/bluesend audio plugin over bluetooth, computes
# the necessary levels that three strips of neopixels should be, and sends data
# to an Arduino over a serial connection.
#
# Currently, just dumps all recieved packets to stdout.
# TODO: Make this work gooder!
#
# Bluetooth stuff based
# on https://github.com/pybluez/pybluez/blob/master/examples/simple/rfcomm-server.py

from bluetooth import *
import gpiozero
import numpy as np
import serial
import subprocess
import scipy.signal as signal
from uuid import uuid4

# Maximum size, in bytes, that a buffer being recieved from Bluesend can be.
MAX_BT_BUFFER_SIZE = 4096

# Address of the serial tty of the connected Arduino
ARDUINO_SERIAL_DEVICE_LOCATION = "/dev/ttyUSB0"

# Baud rate for the Arduino
ARDUINO_BAUD_RATE = 9600

# Sample rate, in Hz
sample_rate = 0.0

# Shutoff button & LED
shutoff_button = gpiozero.Button(23)
shutoff_led = gpiozero.LED(24)

def butter_bandpass(lowcut, highcut, fs, order=5):
    """
    Generates a Butterworth bandpass filter given a sampling rate in Hz.
    Based off of scipy-cookbook.readthedocs.io/items/ButterworthBandpass.html.
    """
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = signal.butter(order, [low, high], btype='band')
    return b, a

def butterworth_bandpass_filter(data, lowcut, highcut, fs, order=5):
    """
    Filter frequencies in the data array using a butterworth bandpass filter
    at a given sample rate.
    """
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = signal.lfilter(b, a, data)
    return y

def overall_signal_power(data):
    """
    Computes the overall power (in dB) of the passed-in signal.
    (Based off of samcarcagno.altervista.ord/blog/basic-sound-processing-python/)
    """
    n = len(data)
    p = np.fft.fft(data) # Take the Fourier transform.

    # Get info about the magnitude of the frequency components by taking the
    # absolute value of the Fourier transform.
    nUniquePts = int(np.ceil((n + 1) / 2.0))
    p = p[0:nUniquePts]
    p = np.abs(p)

    # Scale by the number of points so that the magnitude does not depend on the
    # length of the signal or it's sampling frequency.
    p = p / float(n)

    # Sqaure to get the power.
    p = p**2

    # Double to keep the same energy, as we dropped half the FFT earlier.
    # Odd nfft should exclude the Nyquist point.
    if n % 2 == 0:
        p[1:len(p) - 1] = p[1:len(p) - 1] * 2
    else:
        p[1:len(p)] = p[1:len(p)] * 2

    overall_power = p.sum()
    return overall_power

def scale(in_low, in_high, out_low, out_high, x):
    """
    Scale a number from an input range to an output range.
    """
    numerator = (out_high - out_low) * (x - in_low)
    denominator = in_high - in_low
    return (numerator / denominator) + out_low

def clamp(low, high, x):
    """
    Clamps a number to the interval [low, high]
    """
    return low if x < low else (high if x > high else x)

def shutoff_pressed():
    print("Shutoff button pressed!")
    shutoff_led.off()

def shutoff_released():
    print("Shutoff button released!")
    print("Shutting down...")
    shutoff_led.on()
    shutoff_led.blink(on_time=0.5, off_time=0.5)
    subprocess.run("sudo shutdown -h now", shell=True)

shutoff_button.when_pressed = shutoff_pressed
shutoff_button.when_released = shutoff_released
shutoff_led.on()

arduino = serial.Serial(ARDUINO_SERIAL_DEVICE_LOCATION, ARDUINO_BAUD_RATE)

bt_sock = BluetoothSocket(RFCOMM)
bt_sock.bind(("", PORT_ANY))
bt_sock.listen(1)

port = bt_sock.getsockname()[1]

uuid = str(uuid4())

advertise_service(
        bt_sock,
        "light-synth-pi",
        service_id = uuid,
        service_classes = [uuid, SERIAL_PORT_CLASS],
        profiles = [SERIAL_PORT_PROFILE]
        )

while True:
    print("Waiting for connection on RFCOMM channel %d" % port)
    bluesend_sock, bluesend_info = bt_sock.accept()
    print("Accepted connection from ", bluesend_info)

    sample_rate = np.frombuffer(bluesend_sock.recv(MAX_BT_BUFFER_SIZE), dtype=np.float64)[0]
    print("Sample rate: {}".format(sample_rate))

    try:
        while True:
            data = bluesend_sock.recv(MAX_BT_BUFFER_SIZE)
        
            if (len(data) == 0):
                break

            # When memoryview casts the bytearray to a double, it expects that
            # it's in 8-byte chunks. Bluesend sometimes sends partial chunks of
            # data to save on processing. This is non-configurable (part of
            # Windows). But, math says that we can just keep adding data until
            # it's length is a multiple of eight, and the array of doubles can
            # still be used for DSP calculations.
            while (len(data) % 8 != 0):
                data = b"".join([data, bluesend_sock.recv(MAX_BT_BUFFER_SIZE)])

            # The list of doubles recieved from Bluesend, converted from a binary blob.
            soundFrame = np.frombuffer(data, dtype=np.float64)
            print("Recieved: {}".format(soundFrame))

            low_filtered = butterworth_bandpass_filter(soundFrame, 20., 150., sample_rate)
            print("- Low filtered: {}".format(low_filtered))
            
            mid_filtered = butterworth_bandpass_filter(soundFrame, 142., 800., sample_rate)
            print("- Mid filtered: {}".format(mid_filtered))

            high_filtered = butterworth_bandpass_filter(soundFrame, 750., 4500., sample_rate)
            print("- High filtered: {}".format(high_filtered))

            low_power = overall_signal_power(low_filtered)
            mid_power = overall_signal_power(mid_filtered)
            high_power = overall_signal_power(high_filtered)
            print("- Low power: {} dB, Mid power: {} dB, High power: {} dB".format(low_power, mid_power, high_power))

            # Lower notes generally have more power, so we need to account for that when scaling.
            low_norm = clamp(0, 255, int(scale(0., 0.75, 0., 255., low_power)))
            mid_norm = clamp(0, 255, int(scale(0., 0.5, 0., 255., mid_power)))
            high_norm = clamp(0, 255, int(scale(0., 0.25, 0., 255., high_power)))
            print("- Low norm: {}, Mid norm: {}, High norm: {}".format(low_norm, mid_norm, high_norm))

            arduino.write("{:02X}{:02X}{:02X}".format(low_norm, mid_norm, high_norm).encode("ascii"))

    except IOError:
        pass

    print("Disconnected.")
    bluesend_sock.close()

bt_sock.close()

print("All done!")
