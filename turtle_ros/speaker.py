"""Simple speaker control for Raspberry Pi GPIO.

Features:
- play_tone(pin, frequency_hz, duration_sec) for passive buzzers using PWM
- turn_on(pin) / turn_off(pin) for active buzzers
- play_wav(filepath) using system `aplay` (if available)

This file includes a small DummyGPIO fallback so the module can be imported
on development machines that don't have RPi.GPIO installed.

Wiring notes:
- Passive buzzer: connect buzzer + to GPIO pin, - to GND. Use a series resistor
  if recommended by the buzzer datasheet.
- Active buzzer: connect buzzer V+ to 3.3V or GPIO (depending on buzzer) and
  drive with on/off from the GPIO. Check current limits and use a transistor if
  needed.
"""

import sys
import time
import subprocess
import argparse
import audio_data
import wave
import struct

try:
	import RPi.GPIO as GPIO
	_HAS_RPI = True
except Exception:
	# Fallback dummy GPIO for development / linting on non-RPi machines
	_HAS_RPI = False

	class DummyPWM:
		def __init__(self, pin, freq):
			self.pin = pin
			self.freq = freq
			self._duty = 0

		def start(self, duty):
			self._duty = duty
			print(f"[DummyPWM] start pin={self.pin} freq={self.freq} duty={duty}")

		def ChangeDutyCycle(self, duty):
			self._duty = duty
			print(f"[DummyPWM] ChangeDutyCycle pin={self.pin} duty={duty}")

		def ChangeFrequency(self, freq):
			self.freq = freq
			print(f"[DummyPWM] ChangeFrequency pin={self.pin} freq={freq}")

		def stop(self):
			print(f"[DummyPWM] stop pin={self.pin}")

	class DummyGPIO:
		BCM = 'BCM'
		OUT = 'OUT'

		def setwarnings(self, flag):
			pass

		def setmode(self, mode):
			print(f"[DummyGPIO] setmode({mode})")

		def setup(self, pin, mode):
			print(f"[DummyGPIO] setup(pin={pin}, mode={mode})")

		def output(self, pin, value):
			print(f"[DummyGPIO] output(pin={pin}, value={value})")

		def PWM(self, pin, freq):
			return DummyPWM(pin, freq)

		def cleanup(self):
			print("[DummyGPIO] cleanup()")

	GPIO = DummyGPIO()


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


def play_tone(pin: int, frequency_hz: float, duration_s: float, duty_cycle: float = 50.0):
	"""Play a tone on a passive buzzer connected to `pin` using PWM.

	Args:
		pin: BCM pin number.
		frequency_hz: Frequency of the tone in Hz.
		duration_s: Duration to play in seconds.
		duty_cycle: PWM duty cycle percentage (0-100). 50 is a good default.
	"""
	if frequency_hz <= 0 or duration_s <= 0:
		return

	GPIO.setup(pin, GPIO.OUT)
	pwm = GPIO.PWM(pin, frequency_hz)
	try:
		pwm.start(duty_cycle)
		time.sleep(duration_s)
	finally:
		pwm.stop()


def turn_on(pin: int):
	"""Turn on an active buzzer (set GPIO HIGH)."""
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, True)


def turn_off(pin: int):
	"""Turn off an active buzzer (set GPIO LOW)."""
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, False)


def play_wav(filepath: str):
	"""Play a WAV file using system `aplay` (commonly available on Linux/RPi).

	Falls back to python's wave+stdout print if `aplay` is not present.
	"""
	try:
		subprocess.run(["aplay", filepath], check=True)
	except FileNotFoundError:
		print("aplay not found — cannot play WAV file. Install `alsa-utils` or use another player.")
	except subprocess.CalledProcessError as e:
		print(f"aplay failed: {e}")


def cleanup():
	"""Clean up GPIO state. Call on shutdown."""
	try:
		GPIO.cleanup()
	except Exception:
		pass


def _cli_main():
	parser = argparse.ArgumentParser(description="Test speaker control via GPIO.")
	parser.add_argument("--pin", type=int, default=18, help="BCM GPIO pin for buzzer (default: 18)")
	parser.add_argument("--tone", nargs=2, metavar=("FREQ","DUR"), help="Play tone: frequency(Hz) duration(s)")
	parser.add_argument("--wav", type=str, help="Path to WAV file to play with aplay")
	args = parser.parse_args()

	pin = args.pin
	try:
		if args.tone:
			freq = float(args.tone[0])
			dur = float(args.tone[1])
			print(f"Playing tone {freq}Hz for {dur}s on pin {pin}")
			play_tone(pin, freq, dur)

		if args.wav:
			print(f"Playing WAV: {args.wav}")
			play_wav(args.wav)

	finally:
		cleanup()


if __name__ == "__main__":
	_cli_main()
