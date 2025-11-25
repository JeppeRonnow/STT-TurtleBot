import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

class Plot:
    # Plot audio
    @staticmethod
    def plot_audio(file_name, audio, audio_filter, audio_norm, time) -> None:
        plt.figure(figsize=(10,4))
        plt.plot(time, audio, color='blue')
        plt.xlabel("Time [s]")
        plt.ylabel("Amplitude")
        plt.title("Waveform of " + file_name)
        plt.grid(True)
        plt.tight_layout()

        plt.figure(figsize=(10,4))
        plt.plot(time, audio_filter, color='blue')
        plt.xlabel("Time [s]")
        plt.ylabel("Amplitude")
        plt.title("Waveform of filtered audio")
        plt.grid(True)
        plt.tight_layout()

        plt.figure(figsize=(10,4))
        plt.plot(time, audio_norm, color='blue')
        plt.xlabel("Time [s]")
        plt.ylabel("Amplitude")
        plt.title("Waveform of normalized audio")
        plt.grid(True)
        plt.tight_layout()
        plt.show()