from scipy.signal import butter, sos2tf
import numpy as np


def matlab_array(array):
    return "[" + " ".join(f"{x:.16g}" for x in array) + "]"


if __name__ == '__main__':
    sos = None

    # Filter parameters
    sample_rate = 16000
    highpass_hz = 120.0
    lowpass_hz = 5000.0
    order = 4

    # Create filter
    sos = butter(order,
                 [highpass_hz/(sample_rate/2), lowpass_hz/(sample_rate/2)],
                 btype='bandpass',
                 output='sos')

    # Convert SOS -> single TF (Z-domain)
    n, d = sos2tf(sos)

    # Print TF
    print("n = " + matlab_array(n) + ";")

    print("d = " + matlab_array(d) + ";")


