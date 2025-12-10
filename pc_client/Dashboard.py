import matplotlib.patches as patches
import customtkinter as ctk
import numpy as np
import time

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class Dashboard(ctk.CTk):
    def __init__(self, mqtt_transmitter=None):
        super().__init__()

        # Store MQTT transmitter reference for sending commands
        self.mqtt_transmitter = mqtt_transmitter

        # Configure window
        self.title("TurtleBot Dashboard")
        self.geometry("1400x900")

        # Set theme
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Robot state variables for movement simulation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.is_moving = False
        self.animation_timer = None
        self.last_update_time = time.time()

        # Command history tracking (last 10 commands)
        self.command_history = []  # List of (timestamp, linear, angular) tuples
        self.max_history_entries = 10

        # Configure grid layout (left panel and right panel equal size)
        self.grid_columnconfigure(0, weight=1)  # Left panel weight
        self.grid_columnconfigure(1, weight=1)  # Right panel weight
        self.grid_rowconfigure(0, weight=1)

        # Create left panel for robot position
        self.left_panel = ctk.CTkFrame(self, corner_radius=10)
        self.left_panel.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Configure left panel grid
        self.left_panel.grid_rowconfigure(0, weight=0)  # Header
        self.left_panel.grid_rowconfigure(1, weight=2)  # 2D visualization
        self.left_panel.grid_rowconfigure(2, weight=1)  # Position data
        self.left_panel.grid_columnconfigure(0, weight=1)

        # Create right panel for plots
        self.right_panel = ctk.CTkFrame(self, corner_radius=10)
        self.right_panel.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Configure right panel grid (3 rows: buttons row, command history, audio plots)
        self.right_panel.grid_rowconfigure(0, weight=0)  # Buttons (fixed height)
        self.right_panel.grid_rowconfigure(1, weight=1)  # Command history
        self.right_panel.grid_rowconfigure(2, weight=0)  # Transcription field
        self.right_panel.grid_rowconfigure(3, weight=1)  # Audio plots
        self.right_panel.grid_columnconfigure(0, weight=1)  # Left column for RESET
        self.right_panel.grid_columnconfigure(1, weight=1)  # Right column for STOP


        # -------------------------------------------------- #
        # ------------------- LEFT PANEL ------------------- #
        # -------------------------------------------------- #

        # Left panel header
        self.position_label = ctk.CTkLabel(
            self.left_panel,
            text="Robot Position",
            font=ctk.CTkFont(size=24, weight="bold"),
        )
        self.position_label.grid(row=0, column=0, padx=20, pady=20, sticky="ew")

        # 2D Robot visualization frame
        self.robot_viz_frame = ctk.CTkFrame(self.left_panel, corner_radius=10)
        self.robot_viz_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # Create matplotlib figure for robot visualization
        self.robot_figure = Figure(figsize=(5, 5), dpi=100)
        self.robot_figure.patch.set_facecolor("#2b2b2b")
        self.ax_robot = self.robot_figure.add_subplot(111)
        self.ax_robot.set_facecolor("#1e1e1e")
        self._style_axis(self.ax_robot)

        # Set up grid (-5 to 5 meters, origin at center)
        self.ax_robot.set_xlim(-5, 5)
        self.ax_robot.set_ylim(-5, 5)
        self.ax_robot.set_xlabel("X (m)", color="white")
        self.ax_robot.set_ylabel("Y (m)", color="white")
        self.ax_robot.grid(True, alpha=0.3, color="white", linewidth=0.5)
        self.ax_robot.set_aspect("equal")

        # Adjust layout to maximize plot area
        self.robot_figure.tight_layout(pad=0.5)

        # Draw start marker at origin
        self.ax_robot.plot(0, 0, "g+", markersize=15, markeredgewidth=2, label="Origin")

        # Draw robot as a circle with front indicator
        self.robot_circle, self.robot_indicator = self._create_robot_circle(0.0, 0.0, 0)
        self.ax_robot.add_patch(self.robot_circle)
        (self.robot_indicator_line,) = self.ax_robot.plot(
            self.robot_indicator[0],
            self.robot_indicator[1],
            "r-",
            linewidth=3,
        )

        # Draw trajectory line
        self.trajectory_x = [0]
        self.trajectory_y = [0]
        (self.trajectory_line,) = self.ax_robot.plot(
            self.trajectory_x,
            self.trajectory_y,
            "b-",
            alpha=0.5,
            linewidth=1,
            label="Path",
        )

        self.ax_robot.legend(
            loc="upper right",
            facecolor="#2b2b2b",
            edgecolor="white",
            labelcolor="white",
            fontsize=8,
        )

        # Create canvas for robot visualization
        self.robot_canvas = FigureCanvasTkAgg(
            self.robot_figure, master=self.robot_viz_frame
        )
        self.robot_canvas.draw()
        self.robot_canvas.get_tk_widget().pack(
            fill="both", expand=True, padx=10, pady=10
        )

        # Robot position display frame
        self.position_frame = ctk.CTkFrame(self.left_panel, corner_radius=10)
        self.position_frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")

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


        # --------------------------------------------------- #
        # ------------------- RIGHT PANEL ------------------- #
        # --------------------------------------------------- #
        

        # Create Reset button at top left of right panel
        self.reset_button = ctk.CTkButton(
            self.right_panel,
            text="RESET",
            font=ctk.CTkFont(size=20, weight="bold"),
            fg_color="#6495ED", # Blue
            hover_color="#0047AB",
            height=60,
            command=self.reset_position,
        )
        self.reset_button.grid(row=0, column=0, padx=(10, 5), pady=10, sticky="ew")

        # Create STOP button at top right of right panel
        self.stop_button = ctk.CTkButton(
            self.right_panel,
            text="STOP",
            font=ctk.CTkFont(size=20, weight="bold"),
            fg_color="#F44336",
            hover_color="#D32F2F",
            height=60,
            command=self.emergency_stop,
        )
        self.stop_button.grid(row=0, column=1, padx=(5, 10), pady=10, sticky="ew")

        # Create frame for command history (row 1, spanning both columns)
        self.cmd_history_frame = ctk.CTkFrame(self.right_panel, corner_radius=10)
        self.cmd_history_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        # Command History header
        self.cmd_history_label = ctk.CTkLabel(
            self.cmd_history_frame,
            text="Command History",
            font=ctk.CTkFont(size=18, weight="bold"),
        )
        self.cmd_history_label.pack(padx=10, pady=(10, 5))

        # Scrollable frame for command list
        self.cmd_list_frame = ctk.CTkScrollableFrame(
            self.cmd_history_frame,
            corner_radius=5,
            fg_color="#1e1e1e",
        )
        self.cmd_list_frame.pack(fill="both", expand=True, padx=10, pady=(5, 10))

        # Initialize empty command labels list
        self.cmd_labels = []
        for i in range(self.max_history_entries):
            label = ctk.CTkLabel(
                self.cmd_list_frame,
                text="",
                font=ctk.CTkFont(size=11, family="monospace"),
                anchor="w",
                justify="left",
            )
            label.pack(fill="x", padx=5, pady=2)
            self.cmd_labels.append(label)

        # Transcription display frame (row 2, spanning both columns)
        self.transcription_frame = ctk.CTkFrame(self.right_panel, corner_radius=10)
        self.transcription_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        # Transcription header
        self.transcription_header = ctk.CTkLabel(
            self.transcription_frame,
            text="Last Transcription",
            font=ctk.CTkFont(size=18, weight="bold"),
        )
        self.transcription_header.pack(padx=10, pady=(10, 5))

        # Transcription label
        self.transcription_label = ctk.CTkLabel(
            self.transcription_frame,
            text="None",
            font=ctk.CTkFont(size=14),
            anchor="w",
            justify="left",
        )
        self.transcription_label.pack(padx=10, pady=10)

        # Create frame for audio plots (row 3, spanning both columns)
        self.audio_plot_frame = ctk.CTkFrame(self.right_panel, corner_radius=10)
        self.audio_plot_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        # Audio Plot header
        self.audio_plot_label = ctk.CTkLabel(
            self.audio_plot_frame,
            text="Audio Recordings",
            font=ctk.CTkFont(size=18, weight="bold"),
        )
        self.audio_plot_label.pack(padx=10, pady=10)

        # Create matplotlib figure with 1 subplot (just microphone)
        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.figure.patch.set_facecolor("#2b2b2b")

        # Audio Recordings Plot
        self.ax_mic = self.figure.add_subplot(111)
        self.ax_mic.set_facecolor("#1e1e1e")
        self._style_axis(self.ax_mic)
        self.ax_mic.set_title(
            "Raw & Filtered Audio", color="white", fontsize=12
        )
        self.ax_mic.set_xlabel("Time (s)", color="white")
        self.ax_mic.set_ylabel("Amplitude", color="white")
        self.ax_mic.set_ylim(-1.0, 1.0)  # Linear amplitude range
        self.ax_mic.set_xlim(0, 3)  # Default 3 seconds
        self.ax_mic.grid(True, alpha=0.3, color="white")

        # Initialize empty plot lines for 2 recordings
        (self.raw_line,) = self.ax_mic.plot(
            [], [],
            color="#FF5722",
            linewidth=1,
            alpha=0.6,
            label="Raw",
        )
        (self.filtered_line,) = self.ax_mic.plot(
            [], [],
            color="#2196F3",
            linewidth=1,
            alpha=0.7,
            label="Filtered",
        )

        self.ax_mic.legend(
            loc="upper right",
            facecolor="#2b2b2b",
            edgecolor="white",
            labelcolor="white",
        )

        # Adjust spacing
        self.figure.tight_layout(pad=1.0)

        # Create canvas and add to audio plot frame
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.audio_plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(
            fill="both", expand=True, padx=10, pady=(5, 10)
        )


    # --------------------------------------------------- #
    # -------------------- FUNCTIONS -------------------- #
    # --------------------------------------------------- #

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


    def _create_robot_circle(self, x, y, theta_deg):
        """Create a circle with front indicator representing the robot"""
        # Robot size
        radius = 0.3

        # Convert theta to radians
        theta_rad = np.radians(theta_deg)

        # Create circle
        circle = patches.Circle(
            (x, y),
            radius,
            facecolor="#2196F3",
            edgecolor="white",
            linewidth=2,
        )

        # Create front indicator line
        indicator_x = [x, x + radius * np.sin(theta_rad)]
        indicator_y = [y, y + radius * np.cos(theta_rad)]

        return circle, (indicator_x, indicator_y)


    def emergency_stop(self) -> None:
        print("STOP button pressed")

        # Send stop command (0 velocity) to robot via MQTT
        if self.mqtt_transmitter:
            self.mqtt_transmitter.publish_command(0.0, 0.0)
            self.status_label.configure(text="Status: Stopping", text_color="#F44336") # Red
            time.sleep(0.5)  # Wait a moment to ensure stop command is processed
        else:
            print("Warning: No MQTT transmitter available")

        # Also stop any ongoing animation
        self.update_robot_velocity(0.0, 0.0)


    def update_robot_velocity(self, linear, angular) -> None:
        self.current_linear = linear
        self.current_angular = angular

        # Determine if robot should be moving
        was_moving = self.is_moving
        self.is_moving = (abs(linear) > 0.001 or abs(angular) > 0.001)

        # Update status display
        if self.is_moving:
            status = "Moving"
            color = "#2196F3"  # Blue
        else:
            status = "Idle"
            color = "#4CAF50"  # Green

        self.status_label.configure(text=f"Status: {status}", text_color=color)
        self.velocity_label.configure(text=f"Velocity: {abs(linear):.2f} m/s")

        # Start animation if not already running
        if self.is_moving and not was_moving:
            self.last_update_time = time.time()
            self._animate_movement()
        elif not self.is_moving and self.animation_timer:
            # Stop animation
            self.after_cancel(self.animation_timer)
            self.animation_timer = None


    def _animate_movement(self):
        """Animate robot movement based on current velocities"""
        if not self.is_moving:
            return

        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Update robot position based on velocities
        # Linear velocity moves in direction of current theta
        # theta=0 means pointing up (positive Y), theta increases counter-clockwise
        theta_rad = np.radians(self.robot_theta)
        dx = self.current_linear * np.sin(theta_rad) * dt
        dy = self.current_linear * np.cos(theta_rad) * dt

        # Angular velocity changes theta
        # Positive angular velocity = counter-clockwise (left turn)
        dtheta = -np.degrees(self.current_angular) * dt

        # Update position
        self.robot_x += dx
        self.robot_y += dy
        self.robot_theta += dtheta

        # Normalize theta to [-180, 180]
        while self.robot_theta > 180:
            self.robot_theta -= 360
        while self.robot_theta < -180:
            self.robot_theta += 360

        # Update display
        self.x_label.configure(text=f"X: {self.robot_x:.2f} m")
        self.y_label.configure(text=f"Y: {self.robot_y:.2f} m")
        self.theta_label.configure(text=f"Theta: {self.robot_theta:.2f}°")

        # Update visualization
        self._update_robot_visualization(self.robot_x, self.robot_y, self.robot_theta)

        # Schedule next update (60 FPS)
        self.animation_timer = self.after(16, self._animate_movement)


    def update_robot_position(self, x, y, theta, velocity=None, status=None):
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

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

        # Update 2D visualization
        self._update_robot_visualization(x, y, theta)

        # Clear trajectory if robot is at origin (home)
        if abs(x) < 0.1 and abs(y) < 0.1:
            self.trajectory_x = [0]
            self.trajectory_y = [0]
            self.trajectory_line.set_xdata(self.trajectory_x)
            self.trajectory_line.set_ydata(self.trajectory_y)


    def _update_robot_visualization(self, x, y, theta):
        """Update the 2D robot visualization"""
        # Update robot position (coordinates match grid directly)
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta

        # Dynamic zoom: calculate required limits based on trajectory path
        margin = 1.0  # Minimum margin from edge
        min_range = 5.0  # Minimum range to display

        # Calculate max extent of trajectory
        if len(self.trajectory_x) > 1:
            max_x = max(abs(max(self.trajectory_x)), abs(min(self.trajectory_x)))
            max_y = max(abs(max(self.trajectory_y)), abs(min(self.trajectory_y)))
            max_extent = max(max_x, max_y)
        else:
            max_extent = 0

        # Calculate required range (use larger of min_range or trajectory extent + margin)
        required_range = max(min_range, max_extent + margin)

        # Get current limits
        current_xlim = self.ax_robot.get_xlim()
        current_range = abs(current_xlim[1])

        # Apply same range to both axes to maintain 1:1 ratio
        if required_range != current_range:
            self.ax_robot.set_xlim(-required_range, required_range)
            self.ax_robot.set_ylim(-required_range, required_range)

        # Remove old robot circle
        self.robot_circle.remove()

        # Create new robot circle at new position
        self.robot_circle, indicator = self._create_robot_circle(x, y, theta)
        self.ax_robot.add_patch(self.robot_circle)

        # Update front indicator line
        self.robot_indicator_line.set_xdata(indicator[0])
        self.robot_indicator_line.set_ydata(indicator[1])

        # Update trajectory (keep last 100 points)
        self.trajectory_x.append(x)
        self.trajectory_y.append(y)
        if len(self.trajectory_x) > 100:
            self.trajectory_x.pop(0)
            self.trajectory_y.pop(0)

        self.trajectory_line.set_xdata(self.trajectory_x)
        self.trajectory_line.set_ydata(self.trajectory_y)

        # Redraw canvas
        self.robot_canvas.draw_idle()


    def get_instruction_name(self, linear, angular):
        # Special return code (must match logic.py)
        RETURN_CODE = 69.69
        
        # Check for return command
        if abs(linear - RETURN_CODE) < 0.01 and abs(angular - RETURN_CODE) < 0.01:
            return "return"
        
        # Check for stop (both velocities are 0)
        if abs(linear) < 0.001 and abs(angular) < 0.001:
            return "stop"
        
        # Check for turning (linear is 0, angular is not)
        if abs(linear) < 0.001 and abs(angular) >= 0.001:
            if angular < 0:
                return "turn right"
            else:
                return "turn left"
        
        # Check for forward/backward movement (angular is 0, linear is not)
        if abs(angular) < 0.001 and abs(linear) >= 0.001:
            if linear > 0:
                return "forward"
            else:
                return "backward"
        
        # Mixed movement (both linear and angular)
        if abs(linear) >= 0.001 and abs(angular) >= 0.001:
            direction = "forward" if linear > 0 else "backward"
            turn = "left" if angular > 0 else "right"
            return f"{direction} + turn {turn}"
        
        return "unknown"


    def update_command_history(self, linear, angular):
        # Get current timestamp
        timestamp = time.strftime("%H:%M:%S")
        
        # Add new command to history (at the beginning for newest-first display)
        self.command_history.insert(0, (timestamp, linear, angular))
        
        # Keep only the last max_history_entries commands
        if len(self.command_history) > self.max_history_entries:
            self.command_history = self.command_history[:self.max_history_entries]
        
        # Update the labels
        for i, label in enumerate(self.cmd_labels):
            if i < len(self.command_history):
                timestamp, lin, ang = self.command_history[i]
                instruction_name = self.get_instruction_name(lin, ang)
                # Format: [HH:MM:SS] Linear: +0.20 | Angular: -1.00 - instruction_name
                text = f"[{timestamp}] L: {lin:+.2f} m/s | A: {ang:+.2f} rad/s - {instruction_name}"
                label.configure(text=text, text_color="white")
            else:
                label.configure(text="", text_color="white")


    def update_audio_recordings(self, raw_audio, raw_time, filtered_audio, filtered_time):
        # Update both lines with linear amplitude
        self.raw_line.set_xdata(raw_time)
        self.raw_line.set_ydata(raw_audio)
        self.filtered_line.set_xdata(filtered_time)
        self.filtered_line.set_ydata(filtered_audio)
        
        # Auto-scale axes
        self.ax_mic.relim()
        self.ax_mic.autoscale_view(scalex=True, scaley=True)
        
        # Redraw canvas
        self.canvas.draw_idle()


    def set_recording(self, is_recording) -> None:
        """Update recording status on dashboard"""
        if is_recording:
            self.status_label.configure(text="Status: Recording", text_color="#FF9800")  # Orange
        else:
            self.status_label.configure(text="Status: Idle", text_color="#4CAF50")  # Green


    def set_transcription(self, transcription: str) -> None:
        """Update transcription display on dashboard"""
        self.transcription_label.configure(text=transcription)


    def reset_position(self) -> None:
        """Reset robot position to origin on dashboard"""
        # Send stop command (0 velocity) to robot via MQTT
        if self.mqtt_transmitter:
            self.mqtt_transmitter.reset_position()
            self.status_label.configure(text="Status: Stopping", text_color="#F44336") # Red
            time.sleep(0.5)  # Wait a moment to ensure stop command is processed
        else:
            print("Warning: No MQTT transmitter available")

        # Also stop any ongoing animation
        self.update_robot_position(0.0, 0.0, 0.0)
        self.update_robot_velocity(0.0, 0.0)
        


if __name__ == "__main__":
    # Test the GUI
    app = Dashboard()

    # Test counter for animations
    test_counter = [0]

    def test_update():
        """Test function to simulate live data updates and velocity-based movement"""
        t = test_counter[0]

        # Test velocity-based movement animation
        # Create a sequence: move forward, turn, move backward, stop, repeat
        cycle_length = 200  # Total steps for one complete cycle
        position_in_cycle = t % cycle_length

        if position_in_cycle < 50:  # Moving forward
            linear = 0.2
            angular = 0.0
        elif position_in_cycle < 75:  # Turning
            linear = 0.0
            angular = 1.0  # rad/s
        elif position_in_cycle < 125:  # Moving backward
            linear = -0.15
            angular = 0.0
        elif position_in_cycle < 150:  # Turning other direction
            linear = 0.0
            angular = -1.0
        else:  # Stopped
            linear = 0.0
            angular = 0.0

        # Update robot velocity (this will trigger the animation)
        app.update_robot_velocity(linear, angular)

        # Update Command History with current velocities
        app.update_command_history(linear, angular)

        test_counter[0] += 1

        # Schedule next update
        app.after(100, test_update)

    # Start test updates after 500ms
    app.after(500, test_update)

    app.mainloop()
