import customtkinter as ctk
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class Dashboard(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Configure window
        self.title("TurtleBot Dashboard")
        self.geometry("1400x800")

        # Set theme
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Configure grid layout (left panel wider than right)
        self.grid_columnconfigure(0, weight=2)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Create left panel for robot position
        self.left_panel = ctk.CTkFrame(self, corner_radius=10)
        self.left_panel.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Configure left panel grid
        self.left_panel.grid_rowconfigure(0, weight=0)
        self.left_panel.grid_rowconfigure(1, weight=1)
        self.left_panel.grid_columnconfigure(0, weight=1)

        # Left panel header
        self.position_label = ctk.CTkLabel(
            self.left_panel,
            text="Robot Position",
            font=ctk.CTkFont(size=24, weight="bold"),
        )
        self.position_label.grid(row=0, column=0, padx=20, pady=20, sticky="ew")

        # Robot position display frame
        self.position_frame = ctk.CTkFrame(self.left_panel, corner_radius=10)
        self.position_frame.grid(row=1, column=0, padx=20, pady=(0, 20), sticky="nsew")

        # Position data labels
        self.x_label = ctk.CTkLabel(
            self.position_frame,
            text="X: 0.00 m",
            font=ctk.CTkFont(size=20),
        )
        self.x_label.pack(padx=20, pady=(30, 10))

        self.y_label = ctk.CTkLabel(
            self.position_frame,
            text="Y: 0.00 m",
            font=ctk.CTkFont(size=20),
        )
        self.y_label.pack(padx=20, pady=10)

        self.theta_label = ctk.CTkLabel(
            self.position_frame,
            text="Theta: 0.00°",
            font=ctk.CTkFont(size=20),
        )
        self.theta_label.pack(padx=20, pady=10)

        self.velocity_label = ctk.CTkLabel(
            self.position_frame,
            text="Velocity: 0.00 m/s",
            font=ctk.CTkFont(size=20),
        )
        self.velocity_label.pack(padx=20, pady=10)

        self.status_label = ctk.CTkLabel(
            self.position_frame,
            text="Status: Idle",
            font=ctk.CTkFont(size=20),
            text_color="#4CAF50",
        )
        self.status_label.pack(padx=20, pady=(10, 30))

        # Create right panel for plots
        self.right_panel = ctk.CTkFrame(self, corner_radius=10)
        self.right_panel.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Configure right panel grid (2 rows for 2 plots)
        self.right_panel.grid_rowconfigure(0, weight=1)
        self.right_panel.grid_rowconfigure(1, weight=1)
        self.right_panel.grid_columnconfigure(0, weight=1)

        # Create matplotlib figure with 2 subplots
        self.figure = Figure(figsize=(6, 8), dpi=100)
        self.figure.patch.set_facecolor("#2b2b2b")

        # LDR Plot (top)
        self.ax_ldr = self.figure.add_subplot(211)
        self.ax_ldr.set_facecolor("#1e1e1e")
        self._style_axis(self.ax_ldr)
        self.ax_ldr.set_title("LDR Sensor Data", color="white", fontsize=12)
        self.ax_ldr.set_xlabel("Time (s)", color="white")
        self.ax_ldr.set_ylabel("Light Intensity", color="white")
        self.ax_ldr.set_ylim(0, 100)  # Fixed Y-axis range
        self.ax_ldr.grid(True, alpha=0.3, color="white")

        # Initialize LDR plot with sample data
        self.ldr_time = np.linspace(0, 10, 100)
        self.ldr_data = np.random.rand(100) * 100
        (self.ldr_line,) = self.ax_ldr.plot(
            self.ldr_time, self.ldr_data, color="#FF9800", linewidth=2, label="LDR"
        )
        self.ax_ldr.legend(
            loc="upper right",
            facecolor="#2b2b2b",
            edgecolor="white",
            labelcolor="white",
        )

        # Mic Input Plot (bottom)
        self.ax_mic = self.figure.add_subplot(212)
        self.ax_mic.set_facecolor("#1e1e1e")
        self._style_axis(self.ax_mic)
        self.ax_mic.set_title(
            "Microphone Input (Live vs Filtered)", color="white", fontsize=12
        )
        self.ax_mic.set_xlabel("Time (s)", color="white")
        self.ax_mic.set_ylabel("Amplitude", color="white")
        self.ax_mic.set_ylim(-1.5, 1.5)  # Fixed Y-axis range
        self.ax_mic.grid(True, alpha=0.3, color="white")

        # Initialize Mic plot with sample data
        self.mic_time = np.linspace(0, 1, 1000)
        self.live_audio = np.sin(2 * np.pi * 5 * self.mic_time) + 0.2 * np.random.randn(
            1000
        )
        self.filtered_audio = np.sin(2 * np.pi * 5 * self.mic_time)
        (self.live_line,) = self.ax_mic.plot(
            self.mic_time,
            self.live_audio,
            color="#2196F3",
            linewidth=1,
            alpha=0.7,
            label="Live Audio",
        )
        (self.filtered_line,) = self.ax_mic.plot(
            self.mic_time,
            self.filtered_audio,
            color="#4CAF50",
            linewidth=2,
            label="Filtered Audio",
        )
        self.ax_mic.legend(
            loc="upper right",
            facecolor="#2b2b2b",
            edgecolor="white",
            labelcolor="white",
        )

        # Adjust spacing between subplots
        self.figure.tight_layout(pad=3.0)

        # Create canvas and add to right panel
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.right_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(
            row=0, column=0, rowspan=2, padx=10, pady=10, sticky="nsew"
        )

    def _style_axis(self, ax):
        """Apply consistent dark theme styling to an axis"""
        ax.spines["bottom"].set_color("white")
        ax.spines["top"].set_color("white")
        ax.spines["left"].set_color("white")
        ax.spines["right"].set_color("white")
        ax.tick_params(colors="white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.title.set_color("white")

    def update_robot_position(self, x, y, theta, velocity=None, status=None):
        """
        Update robot position display

        Args:
            x: X position in meters
            y: Y position in meters
            theta: Orientation in degrees
            velocity: Optional velocity in m/s
            status: Optional status string
        """
        self.x_label.configure(text=f"X: {x:.2f} m")
        self.y_label.configure(text=f"Y: {y:.2f} m")
        self.theta_label.configure(text=f"Theta: {theta:.2f}°")

        if velocity is not None:
            self.velocity_label.configure(text=f"Velocity: {velocity:.2f} m/s")

        if status is not None:
            # Color code status
            color = "#4CAF50"  # Green for Idle/OK
            if status.lower() in ["moving", "active"]:
                color = "#2196F3"  # Blue for moving
            elif status.lower() in ["error", "stopped"]:
                color = "#F44336"  # Red for error

            self.status_label.configure(text=f"Status: {status}", text_color=color)

    def update_ldr_plot(self, time_data, ldr_values):
        """
        Update LDR plot with new data

        Args:
            time_data: Array of time values
            ldr_values: Array of LDR sensor values
        """
        self.ldr_line.set_xdata(time_data)
        self.ldr_line.set_ydata(ldr_values)
        self.ax_ldr.relim()
        self.ax_ldr.autoscale_view(scalex=True, scaley=False)  # Only autoscale X-axis
        self.canvas.draw_idle()

    def update_mic_plot(self, time_data, live_audio, filtered_audio):
        """
        Update microphone plot with live and filtered audio

        Args:
            time_data: Array of time values
            live_audio: Array of live audio samples
            filtered_audio: Array of filtered audio samples
        """
        self.live_line.set_xdata(time_data)
        self.live_line.set_ydata(live_audio)
        self.filtered_line.set_xdata(time_data)
        self.filtered_line.set_ydata(filtered_audio)
        self.ax_mic.relim()
        self.ax_mic.autoscale_view(scalex=True, scaley=False)  # Only autoscale X-axis
        self.canvas.draw_idle()


if __name__ == "__main__":
    # Test the GUI
    app = Dashboard()

    # Test counter for animations
    test_counter = [0]

    def test_update():
        """Test function to simulate live data updates"""
        t = test_counter[0]

        # Update robot position with simulated movement
        x = np.sin(t * 0.1) * 5
        y = np.cos(t * 0.1) * 5
        theta = (t * 10) % 360
        velocity = abs(np.sin(t * 0.05)) * 2
        status = "Moving" if velocity > 0.5 else "Idle"
        app.update_robot_position(x, y, theta, velocity, status)

        # Update LDR plot with simulated sensor data
        ldr_time = np.linspace(0, 10, 100)
        ldr_data = 50 + 30 * np.sin(ldr_time + t * 0.1) + 10 * np.random.randn(100)
        app.update_ldr_plot(ldr_time, ldr_data)

        # Update Mic plot with simulated audio
        mic_time = np.linspace(0, 1, 1000)
        live_audio = np.sin(2 * np.pi * 5 * mic_time + t * 0.1) + 0.2 * np.random.randn(
            1000
        )
        filtered_audio = np.sin(2 * np.pi * 5 * mic_time + t * 0.1)
        app.update_mic_plot(mic_time, live_audio, filtered_audio)

        test_counter[0] += 1

        # Schedule next update
        app.after(100, test_update)

    # Start test updates after 500ms
    app.after(500, test_update)

    app.mainloop()
