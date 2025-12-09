from scipy.signal import butter, sos2tf


def matlab_array(array):
    return "[" + " ".join(f"{x:.16g}" for x in array) + "]"


if __name__ == '__main__':
    fs = 16000
    hp = 120
    lp = 5000
    order = 4

    filter = butter(order,
                    [hp/(fs/2), lp/(fs/2)],
                    btype='bandpass',
                    output='sos')

    n, d = sos2tf(filter)

    print("n = " + matlab_array(n) + ";")
    print("d = " + matlab_array(d) + ";")
