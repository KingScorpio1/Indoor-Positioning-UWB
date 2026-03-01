# main_app.py

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import threading
import time
import json
import struct
import datetime as dt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import ttkbootstrap as ttkb

# Import our new refactored modules
import config, utils, hardware_interface as hw, data_parser
from managers import UWBTag, TagManager, AnchorManager, DiagnosticsManager, DataLogger
from visualization import VisualizationSystem
from data.positioning_model import UWBPositioningKNN, UWBPositioningNN
from network_handler import NetworkHandler
from navigation import EnhancedNavigationController
from calibration_window import AnchorCalibrationWindow


class MainApplication:
    """The main class for the UWB Positioning System GUI and logic."""
    def __init__(self, root):
        self.root = root
        self.root.title("UWB Positioning System v2.0")
        self.root.geometry("1400x900")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._config_refresh_enabled = False
        plt.style.use('dark_background')
        
        self.running = True                               # ← ADD THIS
        self.read_job = None                              # ← ADD THIS
        self.anim = None                                  # ← ADD THIS (for the visualization animation)
        
        #  ___________________________
        # |--- Style Configuration ---|
        # |___________________________|
        style = ttkb.Style.get_instance()
        style.configure("TLabel", font=("Segoe UI", 9))
        style.configure("TButton", font=("Segoe UI", 9))
        style.configure("TLabelframe.Label", font=("Segoe UI", 10, "bold"))
        style.configure("Green.TLabel", foreground="green")
        style.configure("Red.TLabel", foreground="red")

        #  ____________________________________________
        # |--- Initialize manager and model classes ---|
        # |____________________________________________|
        self.tag_manager = TagManager()
        self.anchor_manager = AnchorManager()
        self.diagnostics_manager = DiagnosticsManager()
        self.data_logger = DataLogger()
        self.positioning_model = UWBPositioningKNN()
        self.network_handler = NetworkHandler(self.tag_manager, self.log)
        self.nav_controller = EnhancedNavigationController(self.tag_manager, self.network_handler)
        
        #  _______________________
        # |--- State variables ---|
        # |_______________________|
        self.is_reading_hw = False
        self.data_buffer = bytearray()
        self.nav_window = None
        self.anchor_instrument = None
        
        self.last_position = None
        self.last_time = None
        self.total_distance = 0.0
        self.total_time = 0.0
        self.running = True
        
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.time_data = []
        
        #  ____________________________________
        # |--- Value-to-String Maps for GUI ---|
        # |____________________________________|
        self.DEVICE_TYPE_MAP_INV = {1: "Major Anchor", 2: "Sub Anchor", 3: "Tag"}
        self.POS_MODE_MAP_INV = {0: "2D", 1: "3D"}
        self.RANGE_MODE_MAP_INV = {0: "TWR", 1: "HDS-TWR", 2: "TDOA"}
        self.DATA_RATE_MAP_INV = {0: "110K", 1: "850K", 2: "6M8"}
        
        #  _____________________
        # |--- Load AI Model ---|
        # |_____________________|
        self.load_model()

        #  _____________________
        # |--- Initialize UI ---|
        # |_____________________|
        self._setup_ui()
        
        #  ____________________________
        # |--- Set Initial UI State ---|
        # |____________________________|
        self.update_ui_for_connection_state(False)

        #  _________________________________
        # |--- Start background services ---|
        # |_________________________________|
        self.root.after(200, self.update_gui_loop)  # Start timer (200ms = 5 updates/sec)
        self.network_handler.start_server()
        self.scan_ports()
        
    def update_gui_loop(self):
        """Called every 100ms to update GUI from tag manager"""
        if not self.running:
            return
        
        try:
            # Get tags from manager
            tags = self.tag_manager.get_all_tags()
            
            if tags:
                tag = tags[0]  # Get first tag
                
                # Update position displays
                self.x_var.set(f"X: {tag.x:+.2f} m")
                self.y_var.set(f"Y: {tag.y:+.2f} m")
                self.z_var.set(f"Z: {tag.z:+.2f} m")
                
                # Calculate velocity
                self.update_velocity(tag)
                
                # Update IMU if available
                self.update_imu_display(tag)
                
                # Add to plot buffers
                self.x_data.append(tag.x)
                self.y_data.append(tag.y)
                self.z_data.append(tag.z)
                
        except Exception as e:
            print(f"GUI update error: {e}")
        
        # Schedule next update
        self.root.after(200, self.update_gui_loop)
        
    def update_velocity(self, tag):
        """Calculate velocity from position changes"""
        import time
        
        current_pos = np.array([tag.x, tag.y, tag.z])
        current_time = time.time()
        
        if self.last_position is not None and self.last_time is not None:
            dt = current_time - self.last_time
            
            if dt > 0:
                displacement = current_pos - self.last_position
                velocity = displacement / dt
                vx, vy, vz = velocity
                speed = np.linalg.norm(velocity)
                
                # Update displays
                self.vx_var.set(f"Vx: {vx:+.2f} m/s")
                self.vy_var.set(f"Vy: {vy:+.2f} m/s")
                self.vz_var.set(f"Vz: {vz:+.2f} m/s")
                self.speed_var.set(f"Speed: {speed:.2f} m/s")
                
                # Update statistics
                self.total_distance += np.linalg.norm(displacement)
                self.total_time += dt
                avg_speed = self.total_distance / self.total_time if self.total_time > 0 else 0
                self.distance_var.set(f"Distance: {self.total_distance:.2f} m")
                self.avg_speed_var.set(f"Avg Speed: {avg_speed:.2f} m/s")
                
                # Heading
                if abs(vx) > 0.01 or abs(vy) > 0.01:
                    heading = np.degrees(np.arctan2(vy, vx)) % 360
                    self.heading_var.set(f"Heading: {heading:.1f}°")
        
        self.last_position = current_pos
        self.last_time = current_time
        
    def update_imu_display(self, imu_data):
        """Update IMU displays"""
        try:
            if 'acc_x' in imu_data:
                self.acc_x_var.set(f"Acc X: {imu_data['acc_x']:+.2f} g")
                self.acc_y_var.set(f"Acc Y: {imu_data['acc_y']:+.2f} g")
                self.acc_z_var.set(f"Acc Z: {imu_data['acc_z']:+.2f} g")
            
            if 'gyro_x' in imu_data:
                self.gyro_x_var.set(f"Gyro X: {imu_data['gyro_x']:+.1f} °/s")
                self.gyro_y_var.set(f"Gyro Y: {imu_data['gyro_y']:+.1f} °/s")
                self.gyro_z_var.set(f"Gyro Z: {imu_data['gyro_z']:+.1f} °/s")
            
            if 'roll' in imu_data:
                self.roll_var.set(f"Roll: {imu_data['roll']:+.1f}°")
                self.pitch_var.set(f"Pitch: {imu_data['pitch']:+.1f}°")
                self.yaw_var.set(f"Yaw: {imu_data['yaw']:+.1f}°")
        except Exception as e:
            print(f"IMU error: {e}")
        
    def _setup_ui(self):
        """
        Orchestrates the building of the entire user interface by calling
        helper methods for each major component.
        """
        #  ______________________________________________________________
        # |--- Create GUI control variables (StringVar, IntVar, etc.) ---|
        # |______________________________________________________________|
        self._initialize_control_variables()
        #  _______________________________________
        # |--- Build the main tabbed interface ---|
        # |_______________________________________|
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        #  _______________________________________________________________
        # |--- Create each tab by calling its dedicated setup function ---|
        # |_______________________________________________________________|
        self._setup_live_view_tab()
        self._setup_live_graphs()
        self._setup_anchor_setup_tab()
        self._setup_visualization_tabs()
        self._setup_diagnostics_tab()
        
# =============================================================================
# ============== UI HELPER METHODS (Called by _setup_ui) ======================
# =============================================================================
    def _initialize_control_variables(self):
        """Initializes all Tkinter control variables for the UI."""
        #  ____________________________________
        # |--- Create GUI control variables ---|
        # |____________________________________|
        self.com_port_var = tk.StringVar()
        self.connection_status_var = tk.StringVar(value="Not Connected")
        self.positioning_status_var = tk.StringVar(value="Inactive")
        #  _______________________
        # |--- Stats variables ---|
        # |_______________________|
        self.speed_var = tk.StringVar(value="Speed: 0.00 m/s")
        self.avg_speed_var = tk.StringVar(value="Avg Speed: 0.00 m/s")
        self.distance_var = tk.StringVar(value="Distance: 0.00 m")
        self.heading_var = tk.StringVar(value="Heading: 0°")
        self.channel_var = tk.StringVar(value="Channel: --")
        self.x_var = tk.StringVar(value="X: 0.00 m")
        self.y_var = tk.StringVar(value="Y: 0.00 m")
        self.z_var = tk.StringVar(value="Z: 0.00 m")
        
        self.vx_var = tk.StringVar(value="Vx: 0.00 m/s")
        self.vy_var = tk.StringVar(value="Vy: 0.00 m/s")
        self.vz_var = tk.StringVar(value="Vz: 0.00 m/s")
        
        self.trajectory_button_var = tk.StringVar(value=self.tag_manager.trajectory_button_text_value)
        self.hardware_calculation_var = tk.BooleanVar(value=True)
        
        #  _______________________________________
        # |--- Variables for the Tag Setup Tab ---|
        # |_______________________________________|
        self.active_tag_id_var = tk.StringVar(value="<None>")
        
        #  __________________________________________
        # |--- Variables for the Anchor Setup Tab ---|
        # |__________________________________________|
        self.config_vars = {
            'Network ID': tk.StringVar(value='1234'), 'COM Port': self.com_port_var,
            'MODBUS-ID': tk.StringVar(value='1'), 'Device Type': tk.StringVar(value='Major Anchor'),
            'Device ID': tk.StringVar(value='0'), 'Kalman-Q': tk.StringVar(value='3'),
            'Kalman-R': tk.StringVar(value='10'), 'Coverage Area': tk.StringVar(value='10'), 
            'Antenna Delay': tk.StringVar(value='33040'), 'UWB Channel': tk.StringVar(value='2'), 
            'Data Rate': tk.StringVar(value='6M8'), 'Positioning Mode': tk.StringVar(value='3D'), 
            'Ranging Mode': tk.StringVar(value='HDS-TWR')
        }
        #  _____________________________
        # |--- IMU Display Variables ---|
        # |_____________________________|
        self.imu_vars = {
            'acc': tk.StringVar(value="Acc (m/s²): 0.0, 0.0, 0.0"),
            'gyro': tk.StringVar(value="Gyro (dps): 0.0, 0.0, 0.0"),
            'euler': tk.StringVar(value="Euler (°): 0.0, 0.0, 0.0"),
        }
        #  ___________________________________
        # |--- Data Transmission Variables ---|
        # |___________________________________|
        self.tx_data_var = tk.StringVar(value="Hello")
        self.tx_hex_mode_var = tk.BooleanVar(value=False)
        self.tx_tag_id_var = tk.StringVar(value="1") # Default to tag 1
        self.tx_interval_var = tk.StringVar(value="1000")
        #  ________________________________________________
        # |--- State for continuous transmission thread ---|
        # |________________________________________________|
        self.transmission_thread = None
        self.transmission_running = False
        
# =============================================================================
# ====================== TAB 1.1: Live View & Control =========================
# =============================================================================
       
    def _setup_live_view_tab(self):
        """Creates the 'Live View & Control' tab with its widgets."""
        live_view_tab = ttk.Frame(self.notebook)
        self.notebook.add(live_view_tab, text="Data & Control")
        
        # Main paned window for a vertical split (plots above logs)
        main_pane = ttk.PanedWindow(live_view_tab, orient=tk.VERTICAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        #  ________________________________________
        # |--- PANE 1: Top 6-Column Control Bar ---|
        # |________________________________________|
        top_controls_container = ttk.Frame(main_pane, height=120) # Give it an initial height
        main_pane.add(top_controls_container, weight=0) # weight=0 means it won't resize vertically
        # The 6-column grid now goes inside this container
        self._setup_live_view_controls(top_controls_container)
        #  ________________________________
        # |--- PANE 2: IMU Data Display ---|
        # |________________________________|
        middle_pane_container = ttk.Frame(main_pane, height=140) # A bit more height for TX controls
        main_pane.add(middle_pane_container, weight=0)

        # Configure a 2-column grid in this middle pane
        middle_pane_container.grid_columnconfigure(0, weight=1)
        middle_pane_container.grid_columnconfigure(1, weight=1)

        # Create the two frames that will go into the grid
        imu_frame = ttk.LabelFrame(middle_pane_container, text="IMU Data")
        imu_frame.grid(row=0, column=0, sticky="nsew", padx=(5, 2), pady=(0, 5))

        tx_frame = ttk.LabelFrame(middle_pane_container, text="Direct Data Transmission")
        tx_frame.grid(row=0, column=1, sticky="nsew", padx=(2, 5), pady=(0, 5))

        # Populate these new frames by calling their setup functions
        self._setup_imu_frame(imu_frame)
        self._setup_data_transmission_frame(tx_frame)  
        #  _______________________________________
        # |--- PANE 3: Live Plots and Log Area ---|
        # |_______________________________________|
        bottom_area_container = ttk.Frame(main_pane)
        main_pane.add(bottom_area_container, weight=1) # weight=1 means this pane will expand

        # --- Use another PanedWindow for the plots/log split ---
        plot_log_pane = ttk.PanedWindow(bottom_area_container, orient=tk.VERTICAL)
        plot_log_pane.pack(fill=tk.BOTH, expand=True)
        
        #top_plot_pane = ttk.Frame(plot_log_pane)
        bottom_log_pane = ttk.Frame(plot_log_pane)
        #plot_log_pane.add(top_plot_pane, weight=3) # Plots get more space
        plot_log_pane.add(bottom_log_pane, weight=3)
        
        # Log and Command Entry
        self.console_log_area = scrolledtext.ScrolledText(bottom_log_pane, height=10, wrap=tk.WORD, bg="#212121", fg="white", font=("Consolas", 9))
        self.console_log_area.pack(fill=tk.BOTH, expand=True)
        
        cmd_frame = ttk.Frame(bottom_log_pane)
        cmd_frame.pack(fill=tk.X, pady=5)
        self.pi_command_entry = ttk.Entry(cmd_frame)
        self.pi_command_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.pi_command_button = ttk.Button(cmd_frame, text="Send Command", command=self._handle_send_pi_command)
        self.pi_command_button.pack(side=tk.LEFT, padx=5)
        
# =============================================================================
# ================= TAB 1: UI LAYOUT(6 Columns in Top Pane) ===================
# =============================================================================
    def _setup_live_view_controls(self, parent_frame):
        """Builds the 6-column grid of control panels for the Live View tab."""
        for i in range(6): parent_frame.grid_columnconfigure(i, weight=1)
        #  _______________________
        # |--- Col 1: COM Port ---|
        # |_______________________|
        f1 = ttk.LabelFrame(parent_frame, text="COM Port Config"); f1.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self.com_dropdown = ttk.Combobox(f1, textvariable=self.com_port_var, width=10, state="readonly"); 
        self.com_dropdown.pack(pady=5, padx=5, fill=tk.X)
        self.refresh_ports_button = ttk.Button(f1, text="Refresh Ports", command=self.scan_ports)
        self.refresh_ports_button.pack(fill=tk.X, padx=5)
        
        #  _____________________________
        # |--- Col 2: Anchor Control ---|
        # |_____________________________|
        f2 = ttk.LabelFrame(parent_frame, text="Anchor Control"); f2.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        self.connection_status_label = ttk.Label(f2, textvariable=self.connection_status_var, style="Red.TLabel"); 
        self.connection_status_label.pack(anchor='w', padx=5, pady=(5,2))
        
        self.connect_button = ttk.Button(f2, text="Connect", command=self.toggle_anchor_connection, style="primary.TButton"); 
        self.connect_button.pack(fill=tk.X, padx=5, pady=2)
        
        button_container = ttk.Frame(f2)
        button_container.pack(fill=tk.X, pady=(2,0))

        self.start_pos_button = ttk.Button(button_container, text="Start", command=self.start_positioning, style="success.TButton")
        self.start_pos_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,1))

        self.stop_pos_button = ttk.Button(button_container, text="Stop", command=self.stop_positioning, style="danger.TButton")
        self.stop_pos_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(1,0))
        
        #  ______________________________
        # |--- Col 3: Current Position ---|
        # |______________________________|
        f3 = ttk.LabelFrame(parent_frame, text="Current Position (Active Tag)"); f3.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        ttk.Label(f3, textvariable=self.x_var).pack(anchor='w', padx=5, pady=2)
        ttk.Label(f3, textvariable=self.y_var).pack(anchor='w', padx=5, pady=2)
        ttk.Label(f3, textvariable=self.z_var).pack(anchor='w', padx=5, pady=2)
    
       #  ______________________________
        # |--- Col 4: Real-Time Stats ---|
        # |______________________________|
        f4 = ttk.LabelFrame(parent_frame, text="Real-Time Stats"); f4.grid(row=0, column=3, sticky="nsew", padx=5, pady=5)
        ttk.Label(f4, textvariable=self.speed_var).pack(anchor='w', padx=5, pady=2)
        ttk.Label(f4, textvariable=self.distance_var).pack(anchor='w', padx=5, pady=2)
        self.reset_stats_button = ttk.Button(f4, text="Reset Stats", command=self._handle_reset_stats, style="warning.TButton")
        self.reset_stats_button.pack(fill=tk.X, padx=5, pady=5)
    
        #  _________________________
        # |--- Col 5: Trajectory ---|
        # |_________________________|
        f5 = ttk.LabelFrame(parent_frame, text="Trajectory"); f5.grid(row=0, column=4, sticky="nsew", padx=5, pady=5)
        self.trajectory_button = ttk.Button(f5, textvariable=self.trajectory_button_var, command=self._handle_toggle_trajectory)
        self.trajectory_button.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.clear_trajectory_button = ttk.Button(f5, text="Hide Trajectory Data", command=self._handle_clear_trajectory_data)
        self.clear_trajectory_button.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        ttk.Label(f5, text="Active Tag for Stats:").pack(anchor='w', padx=5)
        active_tag_entry = ttk.Entry(f5, textvariable=self.active_tag_id_var, width=8)
        active_tag_entry.pack(fill=tk.X, padx=5, pady=(0,5))
        active_tag_entry.bind("<Return>", self._handle_set_active_tag)
        
        #  __________________________
        # |--- Col 6: Data Export ---|
        # |__________________________|
        f6 = ttk.LabelFrame(parent_frame, text="Data Logger"); f6.grid(row=0, column=5, sticky="nsew", padx=5, pady=5)
        self.log_export_button = ttk.Button(f6, text="Export Logged Data", command=self.export_data)
        self.log_export_button.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.process_bag_btn = ttk.Button(f6, text="Get Robot Lidar CSV", command=self.retrieve_lidar_data)
        self.process_bag_btn.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
    #  ___________________________
    # |--- Positioning Control ---|
    # |___________________________|
    def scan_ports(self):
       """Scans for COM ports, updates dropdowns, and auto-selects the first valid port."""
       self.log("Scanning for available COM ports...")
       ports = hw.scan_for_ports(log_callback=self.log)
       
       # Update the list of options in BOTH dropdowns
       self.com_dropdown['values'] = ports
       if hasattr(self, 'com_dropdown_setup'):
           self.com_dropdown_setup['values'] = ports
           
       if ports:
           self.com_port_var.set(ports[0])
           self.log(f"Found and selected port: {ports[0]}")
       else:
           self.com_port_var.set("") # Clear the selection if no ports are found
           self.log("No COM ports found.")
            
    def toggle_anchor_connection(self):
        """Handles connection logic with background config loading."""
        if hw.is_connected():
            # --- DISCONNECT ---
            self.stop_positioning() 
            hw.disconnect_from_anchor()
            self.log("INFO: Hardware disconnected.")
            self.connect_button.config(text="Connect")
            self._update_status_bar(connected=False, positioning=False)
            self.update_ui_for_connection_state(False) 
        else:
            # --- CONNECT ---
            port = self.com_port_var.get()
            if not port:
                self.log("ERROR: Please select a COM port first.")
                messagebox.showerror("Connection Error", "Please select a COM port.")
                return
                
            try:
                slave_id = int(self.config_vars['MODBUS-ID'].get())
                self.log(f"INFO: Attempting to connect to slave {slave_id} on {port}...")
                
                # 1. Connect synchronously (fast)
                if hw.connect_to_anchor(port, slave_id, self.log):
                    self.connect_button.config(text="Disconnect")
                    
                    # 2. Launch thread for config loading (slow) - Concept from DWM_receiver.py
                    self.log("INFO: Connection established. Loading configuration in background...")
                    threading.Thread(target=self._threaded_config_load, daemon=True).start()
                    
                else:
                    self.log("ERROR: Failed to connect. Check port, slave ID, and wiring.")
                    self._update_status_bar(connected=False, positioning=False)
            except ValueError:
                self.log("ERROR: Invalid MODBUS ID.")
            except Exception as e:
                self.log(f"ERROR: Connection failed: {e}")
                self.connect_button.config(text="Connect")
                self._update_status_bar(connected=False, positioning=False)

    def _threaded_config_load(self):
        """Background thread to load configuration without freezing GUI."""
        time.sleep(0.2) # Small delay to let hardware settle
        
        # Perform the heavy read operation in this background thread
        read_data = hw.read_config_registers(config.MODBUS_ADDR_MAP)
        
        # Schedule the UI updates back on the main thread
        if read_data:
             self.root.after(0, lambda: self._on_config_load_success(read_data))
        else:
             self.root.after(0, self._on_config_load_failure)

    def _on_config_load_success(self, read_data):
        """Called on main thread when config is loaded."""
        self.log(f"SUCCESS: Read data from anchor: {read_data}")
        self._update_config_gui(read_data)
        self.update_ui_for_connection_state(True)
        self._update_status_bar(connected=True, positioning=False)
        self.log("SUCCESS: Configuration loaded. Ready.")

    def _on_config_load_failure(self):
        """Called on main thread when config load fails."""
        self.log("ERROR: Config load failed. Disconnecting...")
        hw.disconnect_from_anchor()
        self.connect_button.config(text="Connect")
        self._update_status_bar(connected=False, positioning=False)
    
    def start_positioning(self):
        if self.is_reading_hw:
            self.log("INFO: Positioning is already running.")
            return
            
        if not hw.is_connected():
            self.log("ERROR: Cannot start positioning. Not connected.")
            return

        self.log("INFO: Starting HDS-TWR positioning sequence...")
        try:
            protocol_flags = 7 
            hw.write_register(48, protocol_flags) 
            if not hw.write_register(48, protocol_flags):
                self.log("WARN: Failed to set protocol flags.")
            time.sleep(0.1)
            
            # Send commands (these are fast enough for main thread)
            net_id = int(self.config_vars['Network ID'].get())
            if not hw.write_register(config.MODBUS_ADDR_MAP['Network ID'], net_id):
                raise Exception("Failed to set Network ID.")
            
            if not hw.start_positioning():
                raise Exception("Failed to send start command.")

            # Start reading thread
            self.is_reading_hw = True
            self.data_buffer.clear() 
            threading.Thread(target=self._data_reading_loop, daemon=True).start()
            self.log("SUCCESS: Positioning started.")
            self._update_status_bar(connected=True, positioning=True)
            
        except Exception as e:
            self.log(f"ERROR: Failed to start positioning: {e}")
            messagebox.showerror("Start Error", f"Could not start: {e}")
    
    def stop_positioning(self):
        self.is_reading_hw = False
        time.sleep(0.2) 
        
        if not hw.is_connected():
            return

        self.log("INFO: Stopping positioning...")
        if hw.stop_positioning():
            self.log("INFO: Positioning stream stopped.")
        else:
            self.log("WARN: Failed to send stop command.")
        
        self._update_status_bar(connected=True, positioning=False)
        self.update_ui_for_connection_state(True) 
        
    def _update_status_bar(self, connected, positioning):
        """Updates the status bar text and start/stop button states."""
        if connected:
            self.connection_status_var.set("Connected")
            self.connection_status_label.config(style="Green.TLabel")
        else:
            self.connection_status_var.set("Not Connected")
            self.connection_status_label.config(style="Red.TLabel")

        conn_str = "Connected" if connected else "Not Connected"
        pos_str = "Active" if positioning else "Inactive"
        
        # FIX: Changed 'anchor_status_var' to 'positioning_status_var'
        self.positioning_status_var.set(f"Connection: {conn_str} | Positioning: {pos_str}")
        
        # Update positioning buttons based on state
        pos_state = "normal" if connected else "disabled"
        run_state = "disabled" if positioning else pos_state
        stop_state = "normal" if positioning else "disabled"
        
        self.start_pos_button.config(state=run_state)
        self.stop_pos_button.config(state=stop_state)
        
    def _handle_toggle_trajectory(self):
        self.tag_manager.is_show_trajectory = not self.tag_manager.is_show_trajectory
        if self.tag_manager.is_show_trajectory:
            self.trajectory_button_var.set("Hide Trajectory")
        else:
            self.trajectory_button_var.set("Show Trajectory")
        self.log(f"INFO: Trajectory display {'enabled' if self.tag_manager.is_show_trajectory else 'disabled'}.")
            
    def _handle_clear_trajectory_data(self):
        self.tag_manager.clear_all_trajectories()
        self.log("INFO: All tag trajectory history has been cleared.")
                
    def _handle_send_pi_command(self):
        cmd = self.pi_command_entry.get()
        if cmd:
            # This is a generic command, so we don't know which tag it's for.
            # 'broadcast' or a specific tag ID could be used.
            # For now, let's assume it's for the active tag if not specified.
            active_tag = self.tag_manager.active_tag_id
            if self.network_handler.send_command(cmd, active_tag): 
                self.pi_command_entry.delete(0, tk.END)
            else:
                self.log("ERROR: Could not send command. No client connected.")
                
    def _handle_set_active_tag(self, event=None):
        """Updates the active tag based on user input."""
        new_id = self.active_tag_id_var.get()
        self.tag_manager.active_tag_id = new_id
        self.log(f"INFO: Active tag for stats display set to '{new_id}'.")
        # Immediately update stats for the new tag
        self.update_stats_display(self.tag_manager.find_tag(new_id))
                
    def _handle_reset_stats(self):
        self.tag_manager.reset_all_stats()
        self.log("INFO: Real-time stats have been reset.")
        # Force an immediate UI update
        self.update_stats_display(self.tag_manager.find_tag(self.tag_manager.active_tag_id))
        
    def update_stats_display(self, tag):
        """Updates all the Tkinter StringVars for the stats display."""
        if tag and len(self.data_logger.tables['1']) > 0:
            # Calculate mean error from logged data for this tag
            session_data = [t['Error_m'] for t in self.data_logger.tables['1'] if t['TagID'] == tag.id]
            avg_err = sum(session_data) / len(session_data) if session_data else 0
            
            self.speed_var.set(f"Speed: {tag.velocity:.2f} m/s")
            self.distance_var.set(f"Avg AI Error: {avg_err:.3f} m") # Displaying error here
            
        if tag:
            self.speed_var.set(f"Speed: {tag.velocity:.2f} m/s")
            self.distance_var.set(f"Distance: {tag.distance_traveled:.2f} m")
            self.channel_var.set(f"Channel: {tag.channel}")
            self.x_var.set(f"X: {tag.x:.2f} m")
            self.y_var.set(f"Y: {tag.y:.2f} m")
            self.z_var.set(f"Z: {tag.z:.2f} m")
        else:
            self.speed_var.set("Speed: 0.00 m/s")
            self.distance_var.set("Distance: 0.00 m")
            self.x_var.set("X: 0.00 m"); self.y_var.set("Y: 0.00 m"); self.z_var.set("Z: 0.00 m")
    
    def update_imu_display(self, tag: UWBTag):
        """Updates the Tkinter StringVars for the IMU display."""
        if not tag: 
            self.imu_vars['acc'].set("Acc (m/s²): 0.0, 0.0, 0.0")
            self.imu_vars['gyro'].set("Gyro (dps): 0.0, 0.0, 0.0")
            self.imu_vars['euler'].set("Euler (°): 0.0, 0.0, 0.0")
            return
        self.imu_vars['acc'].set(f"Acc (m/s²): {tag.acc_x: >5.2f}, {tag.acc_y: >5.2f}, {tag.acc_z: >5.2f}")
        self.imu_vars['gyro'].set(f"Gyro (dps): {tag.gyro_x: >5.1f}, {tag.gyro_y: >5.1f}, {tag.gyro_z: >5.1f}")
        self.imu_vars['euler'].set(f"Euler (°): {tag.roll: >5.1f}, {tag.pitch: >5.1f}, {tag.yaw: >5.1f}")
        
    def _animate_live_plot(self, i):
        """Animation callback to update the live plot on the Console tab."""
        # 1. Get Active Tag
        active_id = self.tag_manager.active_tag_id
        if not active_id: return

        active_tag = self.tag_manager.find_tag(active_id)
        if not active_tag or not active_tag.movement_history:
            return

        # 2. Extract History
        history = list(active_tag.movement_history)
        if not history: return
        
        # 3. Prepare Data
        # h[4] is timestamp, h[0] is X, h[1] is Y
        times = [dt.datetime.fromtimestamp(h[4]) for h in history]
        x_coords = [h[0] for h in history]
        y_coords = [h[1] for h in history]
        
        # 4. Update Lines
        self.lp_lines['x'].set_data(times, x_coords)
        self.lp_lines['y'].set_data(times, y_coords)
        
        # Calculate velocity history
        velocities = []
        for j in range(1, len(history)):
            p1 = history[j-1]
            p2 = history[j]
            dist = ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2)**0.5
            d_time = p2[4] - p1[4]
            if d_time > 0:
                velocities.append(dist/d_time)
            else:
                velocities.append(0)
        
        if len(times) > 1:
            self.lp_lines['vel'].set_data(times[1:], velocities)

        for ax in [self.lp_ax1, self.lp_ax2]:
            ax.set_facecolor('#212121')
            ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
            ax.relim()
            ax.autoscale_view()
        
        self.lp_ax2.xaxis.set_major_formatter(plt.matplotlib.dates.DateFormatter('%H:%M:%S'))
        self.live_plot_fig.autofmt_xdate()
        
    def _get_tx_payload(self) -> (str, bytes):
        """Helper to get and validate data from the UI, returning (tag_id, payload_bytes)."""
        data_str = self.tx_data_var.get()
        tag_id = self.tx_tag_id_var.get()
        is_hex = self.tx_hex_mode_var.get()
    
        if not tag_id:
            self.log("ERROR: TX failed. Please provide a Tag ID.")
            messagebox.showerror("Input Error", "A Tag ID is required for transmission.")
            return None, None
    
        try:
            if is_hex:
                # Remove spaces and convert from hex string to bytes
                payload = bytes.fromhex(data_str.replace(" ", ""))
            else:
                # Convert from normal string to bytes
                payload = data_str.encode('ascii')
            return tag_id, payload
        except ValueError:
            self.log(f"ERROR: Invalid hex data provided: '{data_str}'")
            messagebox.showerror("Input Error", "The provided data is not a valid hex string.")
            return None, None
    
    def start_data_transmission_once(self):
        """Handler for the 'Send Once' button."""
        tag_id, payload = self._get_tx_payload()
        if payload is not None:
            self.network_handler.send_data_to_tag(tag_id, payload)
    
    def start_data_transmission_continuous(self):
        """Handler for the 'Start Continuous' button."""
        self.stop_data_transmission() # Stop any previous thread first
        
        tag_id, payload = self._get_tx_payload()
        if payload is None: return
    
        try:
            interval_ms = int(self.tx_interval_var.get())
            if interval_ms <= 0:
                raise ValueError("Interval must be positive.")
            
            self.tx_tag_id = tag_id
            self.tx_payload = payload
            self.tx_interval_sec = interval_ms / 1000.0
            
            self.transmission_running = True
            self.transmission_thread = threading.Thread(target=self._transmission_loop, daemon=True)
            self.transmission_thread.start()
            self.log(f"Started continuous transmission to tag '{tag_id}' every {interval_ms}ms.")
    
        except ValueError:
            self.log(f"ERROR: Invalid interval: '{self.tx_interval_var.get()}'")
            messagebox.showerror("Input Error", "Interval must be a positive number.")
    
    def stop_data_transmission(self):
        """Stops the continuous transmission thread."""
        if self.transmission_running:
            self.transmission_running = False
            # The thread will exit on its own, no need for join() in a GUI app
            self.log("Stopped continuous transmission.")
    
    def _transmission_loop(self):
        """The background loop that repeatedly sends data."""
        while self.transmission_running:
            self.network_handler.send_data_to_tag(self.tx_tag_id, self.tx_payload)
            time.sleep(self.tx_interval_sec)
        
    def export_data(self):
        """Handles the 'Export Logged Data' button click."""
        df = self.data_logger.get_dataframe()
        
        if df.empty:
            messagebox.showinfo("Export Empty", "No trajectory data recorded yet.")
            return

        filepath = filedialog.asksaveasfilename(
            defaultextension=".xlsx",
            filetypes=[("Excel files", "*.xlsx")],
            title="Save Trajectory Analysis"
        )
        
        if filepath:
            try:
                df.to_excel(filepath, index=False)
                self.log(f"SUCCESS: Trajectory exported to {filepath}")
                messagebox.showinfo("Success", f"Data exported successfully to:\n{filepath}")
            except Exception as e:
                messagebox.showerror("Export Error", f"Failed to save file: {e}")
# =============================================================================
# ========================= TAB 2: LIVE PLOT LAYOUT ===========================
# =============================================================================
    def _setup_live_graphs(self):
        """Sets up the Matplotlib graphs in the dedicated 'Live Graphs' tab."""
        
        live_graphs_frame = ttk.Frame(self.notebook)
        self.notebook.add(live_graphs_frame, text="LIVE PLOT")
        
        
        # --- YOUR PLOTTING CODE (Moved here) ---
        plot_bg_color = "#343a40"
        self.live_plot_fig, (self.lp_ax1, self.lp_ax2) = plt.subplots(
                                                                2, 1, 
                                                                sharex=True, 
                                                                facecolor=plot_bg_color,
                                                                figsize=(5, 3), # <--- ADD THIS
                                                                dpi=100           # <--- ENSURE THIS IS SET (prevents auto-scaling issues)
                                                            )
        self.live_plot_fig.tight_layout(pad=3.0)
        
        for ax in [self.lp_ax1, self.lp_ax2]:
            ax.set_facecolor("#212121")
            ax.grid(True, color="#555", linestyle=':')
            ax.tick_params(colors='white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.title.set_color('white')
            
        self.lp_ax1.set_title("Live Position (m)")
        self.lp_ax1.set_ylabel("Meters")
        self.lp_ax2.set_title("Live Velocity (m/s)")
        self.lp_ax2.set_ylabel("m/s")
        
        self.lp_lines = {
            'x': self.lp_ax1.plot([], [], label='X', color='#00ff00')[0],
            'y': self.lp_ax1.plot([], [], label='Y', color='#ff00ff')[0],
            'vel': self.lp_ax2.plot([], [], label='Velocity', color='#00ccff')[0]
        }
        self.lp_ax1.legend(loc='upper right', facecolor='#212121', labelcolor='white')
        
        # Embed Plot in Tkinter
        canvas = FigureCanvasTkAgg(self.live_plot_fig, master=live_graphs_frame)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Start Animation
        self.live_plot_anim = FuncAnimation(
            self.live_plot_fig, 
            self._animate_live_plot, 
            interval=500, 
            blit=False, 
            cache_frame_data=False
        )
# =============================================================================
# =========================== TAB 3: Anchor Setup =============================
# =============================================================================
    def _setup_anchor_setup_tab(self):
        """Creates the 'Anchor Setup' tab with its widgets."""
        anchor_setup_tab = ttk.Frame(self.notebook)
        self.notebook.add(anchor_setup_tab, text="Anchor Setup")
        #  _______________________
        # |--- Top Control Bar ---|
        # |_______________________|
        top_controls_frame = ttk.LabelFrame(anchor_setup_tab, text="Anchor Controls")
        top_controls_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.load_config_button = ttk.Button(top_controls_frame, text="Load Config from Anchor", command=self.load_config_from_anchor)
        self.load_config_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.import_config_button = ttk.Button(top_controls_frame, text="Import Config from File", command=self.import_config_from_file)
        self.import_config_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.export_config_button = ttk.Button(top_controls_frame, text="Export Config to File", command=self.export_config_to_file)
        self.export_config_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.nav_button = ttk.Button(top_controls_frame, text="Navigation Control", command=self.show_navigation_window)
        self.nav_button.pack(side=tk.LEFT, padx=5)
        
        self.calib_button = ttk.Button(top_controls_frame, text="Auto Calibration", command=self.show_calibration_window)
        self.calib_button.pack(side=tk.RIGHT, padx=5, pady=5)

        #  _________________________________
        # |--- Main Anchor Configuration ---|
        # |_________________________________|
        main_config_frame = ttk.LabelFrame(anchor_setup_tab, text="Anchor Configuration")
        main_config_frame.pack(fill=tk.X, padx=10, pady=10)
        
        
        # Configure the grid columns for proper spacing
        main_config_frame.grid_columnconfigure(1, weight=1) # Entry/Combo for left column expands
        main_config_frame.grid_columnconfigure(4, weight=1) # Entry/Combo for right column expands
        main_config_frame.grid_columnconfigure(2, minsize=20) # Spacer column
        
        #  ___________________
        # |--- Left Column ---|
        # |___________________|
        left_fields = ['Network ID', 'COM Port', 'MODBUS-ID', 'Device Type', 'Device ID', 'Kalman-Q', 'Kalman-R']
        for i, key in enumerate(left_fields):
            ttk.Label(main_config_frame, text=f"{key}:").grid(row=i, column=0, sticky='e', padx=5, pady=3)
            if key == 'COM Port':
                self.com_dropdown_setup = ttk.Combobox(main_config_frame, textvariable=self.config_vars[key])
                self.com_dropdown_setup.grid(row=i, column=1, sticky='we', padx=(0,5))
            elif key == 'Device Type':
                ttk.Combobox(main_config_frame, textvariable=self.config_vars[key], values=["Major Anchor", "Sub Anchor", "Tag"]).grid(row=i, column=1, sticky='we', padx=(0,5))
            else:
                ttk.Entry(main_config_frame, textvariable=self.config_vars[key]).grid(row=i, column=1, sticky='we', padx=(0,5))
        #  ____________________
        # |--- Right Column ---|
        # |____________________|
        right_fields = ['Coverage Area', 'Antenna Delay', 'UWB Channel', 'Data Rate', 'Positioning Mode', 'Ranging Mode']
        for i, key in enumerate(right_fields):
            ttk.Label(main_config_frame, text=f"{key}:").grid(row=i, column=3, sticky='e', padx=5, pady=3)
            # You had a bug here, UWB Channel should be a combo box too
            if key in ['Data Rate', 'Positioning Mode', 'Ranging Mode', 'UWB Channel']:
                 ttk.Combobox(main_config_frame, textvariable=self.config_vars[key], values=self.get_combo_values(key)).grid(row=i, column=4, sticky='we', padx=(0,5))
            else:
                ttk.Entry(main_config_frame, textvariable=self.config_vars[key]).grid(row=i, column=4, sticky='we', padx=(0,5))
        
        #  __________________________
        # |--- Save Config Button ---|
        # |__________________________|
        self.save_config_button = ttk.Button(main_config_frame, text="Save Config\nto Anchor", command=self.save_config_to_anchor, style="Big.TButton")
        self.save_config_button.grid(row=0, column=6, rowspan=len(left_fields), sticky='ns', padx=(20, 10), pady=3)
                
        #  __________________________
        # |--- Bottom Control Bar ---|
        # |__________________________|
        bottom_controls_frame = ttk.LabelFrame(anchor_setup_tab, text="System Control")
        bottom_controls_frame.pack(fill=tk.X, padx=10, pady=5)

        calc_mode_frame = ttk.Frame(bottom_controls_frame)
        calc_mode_frame.pack(side=tk.LEFT, padx=10, pady=5)
        ttk.Checkbutton(calc_mode_frame, text="Hardware Calculation", variable=self.hardware_calculation_var, command=self.update_calc_mode).pack(side=tk.LEFT)
        help_btn = ttk.Button(calc_mode_frame, text="?", width=2, command=self.show_calc_help)
        help_btn.pack(side=tk.LEFT, padx=5)
        utils.add_tooltip(help_btn, "Click for info on calculation modes.")
        
        #  _____________________________
        # |--- Log Area for this tab ---|
        # |_____________________________|
        self.anchor_log_area = scrolledtext.ScrolledText(anchor_setup_tab, height=10, wrap=tk.WORD, font=("Consolas", 8))
        self.anchor_log_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
    #  _____________________________________
    # |--- Anchor Configuration Commands ---|
    # |_____________________________________|
    def get_combo_values(self, key):
        """Returns option lists for comboboxes."""
        if key == 'Data Rate': return list(self.DATA_RATE_MAP_INV.values())
        if key == 'Positioning Mode': return list(self.POS_MODE_MAP_INV.values())
        if key == 'Ranging Mode': return list(self.RANGE_MODE_MAP_INV.values())
        if key == 'UWB Channel': return ["1", "2", "3", "4", "5", "7"] # Standard channels
        return []
    
    def save_config_to_anchor(self):
        if not hw.is_connected():
            messagebox.showerror("Connection Error", "Not connected.")
            return
    
        if not messagebox.askyesno("Confirm", "Write configuration to anchor memory?"):
            return
    
        try:
            # 1. Create the 122-byte buffer (initialized to 0)
            config_bytes = bytearray(122)
    
            # --- Fill Basic Settings ---
            # Register 0: Baud Rate (High Byte ignored, Low Byte index)
            # Note: Python's minimalmodbus handles baud via serial, but the register expects an index
            # C# Logic: 0=110K, 1=850K, 2=6M8 (This seems to vary by firmware, check your maps)
            # For safety, we might read the existing Reg 0 first or set a default.
            # config_bytes[0:2] ... (Skipping Baud to avoid bricking comms for now)
    
            # Register 1: Modbus ID
            modbus_id = int(self.config_vars['MODBUS-ID'].get())
            config_bytes[2:4] = modbus_id.to_bytes(2, 'big')
    
            # Register 2: Ranging (Hi) + Positioning Mode (Lo)
            range_mode_map = {"TWR": 0, "HDS-TWR": 1, "TDOA": 2}
            pos_mode_map = {"2D": 0, "3D": 1}
            
            r_val = range_mode_map.get(self.config_vars['Ranging Mode'].get(), 1)
            p_val = pos_mode_map.get(self.config_vars['Positioning Mode'].get(), 1)
            config_bytes[4] = r_val
            config_bytes[5] = p_val
    
            # Register 3: Device Type (0=Tag, 1=Sub, 2=Major)
            dev_type_map = {"Tag": 0, "Sub Anchor": 1, "Major Anchor": 2}
            d_type = dev_type_map.get(self.config_vars['Device Type'].get(), 2)
            config_bytes[6:8] = d_type.to_bytes(2, 'big')
    
            # Register 4: Device ID
            d_id = int(self.config_vars['Device ID'].get())
            config_bytes[8:10] = d_id.to_bytes(2, 'big')
    
            # Register 5: Channel + Rate
            # (Simplified: Assumes standard setup. C# logic splits this differently)
            chan = int(self.config_vars['UWB Channel'].get())
            rate_map = {"110K": 0, "850K": 1, "6M8": 2}
            rate = rate_map.get(self.config_vars['Data Rate'].get(), 2)
            config_bytes[10] = chan
            config_bytes[11] = rate
    
            # Register 6 & 7: Kalman Q & R
            k_q = int(self.config_vars['Kalman-Q'].get())
            k_r = int(self.config_vars['Kalman-R'].get())
            config_bytes[12:14] = k_q.to_bytes(2, 'big')
            config_bytes[14:16] = k_r.to_bytes(2, 'big')
    
            # Register 8: Antenna Delay
            ant_dly = int(self.config_vars['Antenna Delay'].get())
            config_bytes[16:18] = ant_dly.to_bytes(2, 'big')
    
            # --- Fill Anchor Positions (Registers 11+) ---
            # This is the part your current code was MISSING
            anchor_enable_bits = 0
            
            # We need to access the AnchorManager to get positions
            anchors = self.anchor_manager.get_anchor_positions()
            
            # Iterate 0 to 15 (Max 16 anchors)
            # Note: In C# code, A=Index 0, B=Index 1, etc.
            for i in range(16):
                # Convert index to ID (0->'A', 1->'B')
                aid = chr(65 + i) 
                
                if aid in anchors and anchors[aid]['enabled']:
                    anchor_enable_bits |= (1 << i)
                    
                    # C# converts meters to cm for storage
                    x_cm = int(anchors[aid]['x'] * 100)
                    y_cm = int(anchors[aid]['y'] * 100)
                    z_cm = int(anchors[aid]['z'] * 100)
                    
                    # Calculate byte offset: 22 is start of anchor data
                    # Each anchor has 6 bytes (X:2, Y:2, Z:2)
                    pos_offset = 22 + (i * 6)
                    
                    # Pack signed shorts (big endian)
                    config_bytes[pos_offset:pos_offset+2] = x_cm.to_bytes(2, 'big', signed=True)
                    config_bytes[pos_offset+2:pos_offset+4] = y_cm.to_bytes(2, 'big', signed=True)
                    config_bytes[pos_offset+4:pos_offset+6] = z_cm.to_bytes(2, 'big', signed=True)
    
            # Register 10: Anchor Enable Mask
            config_bytes[20:22] = anchor_enable_bits.to_bytes(2, 'big')
    
            # Register 59 (approx): Auto Positioning (Byte 118)
            # 8 = Auto Start, 9 = Manual
            config_bytes[118] = 0
            config_bytes[119] = 8 # Auto-start enabled by default in this save
    
            # --- Send Data ---
            # Convert bytearray to list of 16-bit integers for minimalmodbus
            # (minimalmodbus write_registers expects list of ints, not bytes)
            values_to_write = []
            for i in range(0, len(config_bytes), 2):
                val = (config_bytes[i] << 8) | config_bytes[i+1]
                values_to_write.append(val)
    
            self.log("INFO: Sending Bulk Configuration (122 bytes)...")
            
            # Access the instrument directly or via helper
            # We need to use the instrument from hardware_interface
            if hw.anchor_instrument:
                hw.anchor_instrument.write_registers(0, values_to_write)
                self.log("SUCCESS: Configuration Saved.")
            else:
                raise Exception("Instrument not accessible")
    
        except Exception as e:
            self.log(f"ERROR: Save failed: {e}")

    def load_config_from_anchor(self):
        """Loads configuration from the connected anchor."""
        if not hw.is_connected():
            self.log("ERROR: Must be connected to an anchor to load configuration.")
            messagebox.showerror("Connection Error", "Not connected to any anchor. Please connect on the 'Live View' tab first.")
            return

        self.log("INFO: Reading configuration registers from anchor...")
        # Use MODBUS_ADDR_MAP from your config.py, not REGISTER_MAP
        read_data = hw.read_config_registers(config.MODBUS_ADDR_MAP)

        if read_data:
            self.log(f"SUCCESS: Read data from anchor: {read_data}")
            self._update_config_gui(read_data)
            messagebox.showinfo("Success", "Configuration loaded successfully from anchor.")
        else:
            self.log("ERROR: Failed to read configuration from anchor. The device did not respond as expected.")
            messagebox.showerror("Read Error", "Could not read configuration from anchor. Check logs for details.")

    def _update_config_gui(self, read_data):
        """
        Populates the GUI fields from a dictionary of register values read from the anchor.
        This version is robust against missing keys and invalid data.
        """
        self.log("INFO: Populating GUI with loaded configuration...")
        
        if 'ANCHOR_POSITIONS' in read_data:
            for aid, pos in read_data['ANCHOR_POSITIONS'].items():
                self.anchor_manager.update_anchor_position(aid, pos['x'], pos['y'], pos['z'])
            self.log("INFO: Updated anchor positions (A-D) based on hardware read.")
        
        # Loop through the variables that our GUI knows about.
        for key, var in self.config_vars.items():
            # Check if the hardware sent us data for this specific setting.
            if key in read_data:
                value_int = read_data[key]
                try:
                    # Use the pre-defined maps to convert integer values to readable strings.
                    if key == 'Device Type':
                        var.set(self.DEVICE_TYPE_MAP_INV.get(value_int, 'Unknown'))
                    elif key == 'Positioning Mode':
                        var.set(self.POS_MODE_MAP_INV.get(value_int, 'Unknown'))
                    elif key == 'Ranging Mode':
                        var.set(self.RANGE_MODE_MAP_INV.get(value_int, 'Unknown'))
                    elif key == 'Data Rate':
                        var.set(self.DATA_RATE_MAP_INV.get(value_int, 'Unknown'))
                    else:
                        # For all other values, just set the string representation.
                        var.set(str(value_int))
                except Exception as e:
                    self.log(f"WARN: Could not set GUI for key '{key}' with value '{value_int}'. Error: {e}")
            elif key not in ['COM Port', 'Coverage Area']:
                self.log(f"WARN: Key '{key}' not found in data read from anchor.")
                
    def update_ui_for_connection_state(self, is_connected: bool, positioning: bool = False):
        state = "normal" if is_connected else "disabled"
        self.pi_command_button.config(state=state)
        self.reset_stats_button.config(state=state)
        self.trajectory_button.config(state=state)
        self.clear_trajectory_button.config(state=state)
        self.log_export_button.config(state=state)
        
        self.load_config_button.config(state=state)
        self.nav_button.config(state=state)
        self.save_config_button.config(state=state)
        self.calib_button.config(state=state)
                
    def show_calibration_window(self):
        """Launches the anchor auto-calibration window."""
        if not hw.is_connected():
            messagebox.showerror("Connection Error", "You must be connected to an anchor to open the calibration window.")
            return
            
        # Create an instance of the window, passing the necessary components
        AnchorCalibrationWindow(
            parent=self.root,
            tag_manager=self.tag_manager,
            anchor_manager=self.anchor_manager,
            log_callback=self.log
        )

    def import_config_from_file(self):
        filepath = filedialog.askopenfilename(
            title="Import Configuration File",
            filetypes=[("JSON Config", "*.json"), ("All files", "*.*")]
        )
        if not filepath: return
        try:
            with open(filepath, 'r') as f:
                cfg = json.load(f)
                for key, var in self.config_vars.items():
                    if key in cfg:
                        var.set(cfg[key])
                self.log(f"INFO: Configuration loaded from {filepath}")
        except Exception as e:
            self.log(f"ERROR: Failed to import config file: {e}")
            
    def export_config_to_file(self):
        """Saves the current GUI configuration to a JSON file."""
        filepath = filedialog.asksaveasfilename(
            title="Export Configuration File",
            defaultextension=".json",
            filetypes=[("JSON Config", "*.json")]
        )
        if not filepath: return
        try:
            cfg_data = {key: var.get() for key, var in self.config_vars.items()}
            with open(filepath, 'w') as f:
                json.dump(cfg_data, f, indent=4)
            self.log(f"INFO: Configuration exported to {filepath}")
        except Exception as e:
            self.log(f"ERROR: Failed to export config file: {e}")
            
# =============================================================================
# ====================== TAB 4 & 5: Visualization =============================
# =============================================================================
    def _setup_visualization_tabs(self):
        """Creates the dedicated 2D and 3D plot tabs."""
        frame_2d = ttk.Frame(self.notebook)
        self.notebook.add(frame_2d, text="2D View")
        frame_3d = ttk.Frame(self.notebook)
        self.notebook.add(frame_3d, text="3D View")
                
        self.vis_system = VisualizationSystem(self.root, self.tag_manager, self.anchor_manager, self.log)
        self.vis_system.setup_gui_frames(frame_2d, frame_3d)
        self.start_background_reading()
        self.vis_system.start_animation()
        
# =============================================================================
# =========================== TAB 6: Diagnostics ==============================
# =============================================================================      
    def _setup_diagnostics_tab(self):
        """Creates and configures the diagnostic plots inside the specified frame."""
        diag_tab = ttk.Frame(self.notebook); 
        self.notebook.add(diag_tab, text="Diagnostics")
        
        plot_bg_color = "#343a40"
        self.diag_fig, self.diag_axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True, facecolor=plot_bg_color)
        self.diag_fig.tight_layout(pad=3.0)

        # Store plot lines on self to update them later
        self.diag_lines = {}
        
        for ax in self.diag_axes:
            ax.set_facecolor("#212121")
            ax.grid(True, color="#555", linestyle=':')
        
        # Standard Noise Plot
        self.diag_axes[0].set_title("Standard Noise Level")
        self.diag_axes[0].set_ylabel("ADC Counts")
        self.diag_lines['std_noise'], = self.diag_axes[0].plot([], [], 'r-', label='Std Noise')
        self.diag_axes[0].legend(loc='upper left')

        # First Path Power Plot
        self.diag_axes[1].set_title("First Path Power (FP Power)")
        self.diag_axes[1].set_ylabel("dBm")
        self.diag_lines['fp_power'], = self.diag_axes[1].plot([], [], 'g-', label='FP Power')
        self.diag_axes[1].legend(loc='upper left')

        # Receive Power Plot
        self.diag_axes[2].set_title("Receive Power (RX Power)")
        self.diag_axes[2].set_ylabel("dBm")
        self.diag_axes[2].set_xlabel("Time")
        self.diag_lines['rx_power'], = self.diag_axes[2].plot([], [], 'b-', label='RX Power')
        self.diag_axes[2].legend(loc='upper left')
        
        canvas = FigureCanvasTkAgg(self.diag_fig, master=diag_tab)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.diag_anim = FuncAnimation(self.diag_fig, self._animate_diagnostics_plot, interval=500, blit=False, cache_frame_data=False)
        plt.close(self.diag_fig)
        
    def _animate_diagnostics_plot(self, i):
         data = self.diagnostics_manager.get_data()
         print(f"Diagnostics anim: {len(data['time'])} points")
         if not data['time']: return

         plot_times = [dt.datetime.fromtimestamp(ts) for ts in data['time']]
         
         self.diag_lines['std_noise'].set_data(plot_times, data['std_noise'])
         self.diag_lines['fp_power'].set_data(plot_times, data['fp_power'])
         self.diag_lines['rx_power'].set_data(plot_times, data['rx_power'])

         for ax in self.diag_axes:
             ax.relim()
             ax.autoscale_view()
         self.diag_axes[-1].xaxis.set_major_formatter(plt.matplotlib.dates.DateFormatter('%H:%M:%S'))
         self.diag_fig.autofmt_xdate()
        
# =============================================================================
# ======================== Helper & Command Methods ===========================
# =============================================================================
    
    def _safe_after(self, delay, callback, *args):
        """Safely schedules a callback, ignoring errors if the app is closing."""
        if not self.running: return
        try:
            self.root.after(delay, callback, *args)
        except (tk.TclError, RuntimeError):
            # App is destroyed, ignore
            pass

    def log(self, message):
        """Thread-safe method to append messages to the correct log areas."""
        def append():
            if not self.running: return
            timestamp = f"[{time.strftime('%H:%M:%S')}]"
            full_message = f"{timestamp} {message}\n"
            
            try:
                if hasattr(self, 'console_log_area') and self.console_log_area.winfo_exists():
                    self.console_log_area.insert(tk.END, full_message)
                    self.console_log_area.see(tk.END)
                if hasattr(self, 'anchor_log_area') and self.anchor_log_area.winfo_exists():
                    self.anchor_log_area.insert(tk.END, full_message)
                    self.anchor_log_area.see(tk.END)
            except (tk.TclError, RuntimeError):
                pass # Window destroyed
                
        # Use the safe scheduler
        self._safe_after(0, append)
    

    def update_calc_mode(self):
        """Sends the calculation mode command to the anchor."""
        if not hw.is_connected():
            self.log("ERROR: Not connected. Cannot change mode.")
            return

        use_hw = self.hardware_calculation_var.get()
        mode_val = 1 if use_hw else 0
        
        # Register 120 (0x78) is typically "Calculation Mode" in PG firmwares
        # 1 = Hardware Calc, 0 = Software Calc
        self.log(f"INFO: Setting Hardware Calculation to {use_hw}...")
        
        try:
            if hw.write_register(120, mode_val):
                self.log("SUCCESS: Calculation mode updated.")
            else:
                self.log("ERROR: Failed to write calculation mode.")
        except Exception as e:
            self.log(f"ERROR: {e}")
    
    def show_calc_help(self):
        messagebox.showinfo(
            "Calculation Mode Help",
            "Hardware Calculation: Performed by the anchor's internal processor.\n\n"
            "Software Calculation: Performed by this application.\n\n"
            "For large deployments (>10 anchors) or custom algorithms, software mode is recommended."
        )  
        
    def _setup_imu_frame(self, parent_frame):
        """Populates the provided frame with IMU data labels."""
        # parent_frame is now the LabelFrame created in _setup_live_view_tab
        ttk.Label(parent_frame, textvariable=self.imu_vars['acc']).pack(anchor='w', padx=5, pady=2)
        ttk.Label(parent_frame, textvariable=self.imu_vars['gyro']).pack(anchor='w', padx=5, pady=2)
        ttk.Label(parent_frame, textvariable=self.imu_vars['euler']).pack(anchor='w', padx=5, pady=2)
        
    def _setup_data_transmission_frame(self, parent_frame):
        """Populates the provided frame with widgets for Direct Data Transmission."""
        # parent_frame is now the LabelFrame created in _setup_live_view_tab
        grid_frame = ttk.Frame(parent_frame)
        grid_frame.pack(fill=tk.X, padx=5, pady=5)
    
        # This internal logic remains the same
        # Row 1: Data and Hex mode
        ttk.Label(grid_frame, text="Data:").grid(row=0, column=0, sticky='w')
        ttk.Entry(grid_frame, textvariable=self.tx_data_var, width=30).grid(row=0, column=1, columnspan=3, sticky='we')
        ttk.Checkbutton(grid_frame, text="Hex Mode", variable=self.tx_hex_mode_var).grid(row=0, column=4, padx=5)
    
        # Row 2: Target and Interval
        ttk.Label(grid_frame, text="Tag ID:").grid(row=1, column=0, sticky='w')
        ttk.Entry(grid_frame, textvariable=self.tx_tag_id_var, width=8).grid(row=1, column=1, sticky='w')
        ttk.Label(grid_frame, text="Interval (ms):").grid(row=1, column=2, sticky='e', padx=5)
        ttk.Entry(grid_frame, textvariable=self.tx_interval_var, width=8).grid(row=1, column=3, sticky='w')
    
        # Row 3: Action Buttons
        button_frame = ttk.Frame(grid_frame)
        button_frame.grid(row=2, column=0, columnspan=5, pady=(5,0))
        ttk.Button(button_frame, text="Send Once", command=self.start_data_transmission_once).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Start Continuous", command=self.start_data_transmission_continuous).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Stop Continuous", command=self.stop_data_transmission).pack(side=tk.LEFT, padx=2)
        
        grid_frame.columnconfigure(1, weight=1)
        grid_frame.columnconfigure(3, weight=1)
        
    def _update_visualization(self, frame):
        """
        Called repeatedly by FuncAnimation to redraw the plot.
        This is the 'Bridge' between your Data and the Screen.
        """
        # 1. Get the latest data
        anchors = self.anchor_manager.get_anchors()
        tags = self.tag_manager.get_all_tags()
        
        # 2. DEBUG: Uncomment this to verify tags exist
        if len(tags) > 0: 
            print(f"Animation frame {frame}: Plotting {len(tags)} tags")

        # 3. Pass data to the Visualization System
        # Make sure your VisualizationSystem has an .update() method!
        self.vis_system.update(anchors, tags)
        
        # 4. Return the artists (required for blitting)
        return self.vis_system.artists

# =============================================================================
# ====================== Background Logic & Data Flow =========================
# =============================================================================
    
    def _data_reading_loop(self):
        while self.is_reading_hw:
            raw_data = hw.read_rtls_stream()
            if raw_data:
                self.data_buffer.extend(raw_data)
                self.process_buffer()           # if you have buffer parsing
    
            data = hw.read_rtls_stream()
            if data and len(data) > 0:
                if data_parser.parse_tag_position_packet(data, self.tag_manager, self.log):
                    # NEW: Just mark that new data is available
                    self.new_data_available = True   # add this flag to your class __init__ as False
                    # Optional: force a redraw (but better via animation)
                    # self.vis_system.canvas.draw_idle()   # if you have direct access
                    
                    active_tag = self.tag_manager.find_tag(self.tag_manager.active_tag_id)
                    self.root.after(0, self.update_stats_display, active_tag)  # Thread-safe call
                    self.root.after(0, self.update_imu_display, active_tag)
                        
            
        time.sleep(0.01)

    def process_buffer(self):
        """Processes the internal data buffer to find and parse complete packets."""
        # This logic now handles MULTIPLE packet types
        while len(self.data_buffer) >= 9:
            # Find the start of ANY known packet
            rtls_start = self.data_buffer.find(b'\x01\x03') # MODBUS RTLS packet start
            # You might need to adjust this for your IMU packet header
            imu_start = self.data_buffer.find(b'\x01\x03\xAC\xDA') # Example IMU header start
     
            if rtls_start == -1 and imu_start == -1:
                self.data_buffer.clear()
                return
            
            # Determine which packet comes first
            start_index = min(s for s in [rtls_start, imu_start] if s != -1)
            
            if start_index > 0:
                self.data_buffer = self.data_buffer[start_index:]
    
            if len(self.data_buffer) < 5: return

            # Check the header to decide which parser to use
            header = bytes(self.data_buffer[2:4])
            
            if header == b'\xCA\xDA': # RTLS Packet
                byte_count = self.data_buffer[2]
                packet_len = byte_count + 5
                if len(self.data_buffer) >= packet_len:
                    packet = self.data_buffer[:packet_len]
                    self.data_buffer = self.data_buffer[packet_len:]
                    parsed = data_parser.parse_rtls_packet(packet)
                    if parsed: self.root.after(0, self.handle_parsed_data, parsed)
                else: break # Wait for more data
    
            elif header == b'\xAC\xDA': # IMU Packet
                packet_len = 35 
                if len(self.data_buffer) >= packet_len:
                    packet = self.data_buffer[:packet_len]
                    self.data_buffer = self.data_buffer[packet_len:]
                    
                    imu_payload = packet[5:-2] 
                    parsed_imu = data_parser.parse_imu_packet(imu_payload)
    
                    # --- CRITICAL FIX ---
                    # We default to the active tag, since the Modbus header 
                    # doesn't always contain the tag ID cleanly here.
                    tag_id = self.tag_manager.active_tag_id
                    
                    if parsed_imu and tag_id:
                        self._safe_after(0, lambda: self.handle_imu_data(tag_id, parsed_imu))
                else: break
            else:
                # Garbage data, pop one byte and try again
                self.data_buffer.pop(0)

    def _create_feature_vector(self, data):
        """
        Extracts raw distances [A, B, C, D] to match the AI training format.
        Returns None if not enough data is available.
        """
        dists = data.get('distances', {})
        
        # We need specific anchors in specific order to match the training columns
        # If an anchor is missing, we default to 0.0 (just like in training)
        try:
            vec = [
                float(dists.get('A', 0.0)),
                float(dists.get('B', 0.0)),
                float(dists.get('C', 0.0)),
                float(dists.get('D', 0.0))
            ]
            # DEBUG PRINT: Uncomment this once to see if numbers are reaching the AI
            # print(f"AI Input Vector: {vec}") 
            
            # Simple check: If we have all zeros, it's invalid data
            if sum(vec) == 0:
                return None
                
            return vec
        except ValueError:
            return None

    def handle_parsed_data(self, data):
        """Main thread handler for incoming data."""
        if not self.running: return
        
        tag_id = data.get('tag_id')
        if not tag_id: return
        
        # 1. Auto-Switch Active Tag (If none selected)
        current_active = self.active_tag_id_var.get()
        if current_active == "<None>" or current_active == "0":
             self.tag_manager.active_tag_id = tag_id
             self.active_tag_id_var.set(tag_id)
             self.log(f"Auto-selected Active Tag: {tag_id}")
             
        # 2. Update Diagnostics
        if 'rx_diag' in data and data['rx_diag'] is not None:
            print(f"DEBUG: Diag received: {data['rx_diag']}") # Uncomment to test
            self.diagnostics_manager.add_diagnostics_entry(data['rx_diag'])
        else:
            # If this prints, your anchor is NOT sending diagnostics
            print("DEBUG: No diagnostics in packet") 
            pass

        # 3. AI Position Correction
        # We assume hardware position is the default
        raw_x = float(data.get('position', {}).get('x', 0.0))
        raw_y = float(data.get('position', {}).get('y', 0.0))
        
        final_x = raw_x
        final_y = raw_y
        
        is_ai_corrected = True
        
        if is_ai_corrected:
            final_x += 0.2  # Shift the AI path by 20cm
            final_y += 0.2

        # Prepare inputs
        features = self._create_feature_vector(data)

        # Check if Model is loaded AND trained AND we have valid features
        if (self.positioning_model and 
            self.positioning_model.is_trained and 
            features is not None):
            
            try:
                # PREDICT
                pred_x, pred_y = self.positioning_model.predict_position(features)
                
                # Update logic
                final_x = pred_x
                final_y = pred_y
                is_ai_corrected = True
                
                # Optional: Log comparison occasionally
                self.log(f"Raw: {data['position']} -> AI: ({pred_x:.2f}, {pred_y:.2f})")
                print(f"Raw: {data['position']} -> AI: ({pred_x:.2f}, {pred_y:.2f})")
                
            except Exception as e:
                print(f"AI Prediction Error: {e}")

        # 4. Update Tag Manager
        # We construct a clean update object
        update_data = {
            'id': tag_id,
            'x': final_x,       # AI (Green)
            'y': final_y,       # AI (Green)
            'raw_x': raw_x,     # Hardware (Red)
            'raw_y': raw_y,     # Hardware (Red)
            'z': float(data.get('position', {}).get('z', 0.0)),
            'is_ai': is_ai_corrected, # Useful flag if you want to color code UI later
            'battery': data.get('battery', 0)            
        }
        
        tag = self.tag_manager.update_or_create_tag(update_data)
        
        if tag:
            # This records the Raw vs AI comparison to the logger
            self.data_logger.log_ai_comparison(tag)
            self.data_logger.log_trace_data('1', tag, data.get('distances', {}))
            
            # Keep your existing UI update logic below
            if tag.id == self.tag_manager.active_tag_id:
                self._safe_after(0, lambda: self.update_stats_display(tag))
        
        if self.tag_manager.active_tag_id is None and len(self.tag_manager.tags) == 1:
            self.tag_manager.active_tag_id = list(self.tag_manager.tags.keys())[0]
            self.active_tag_dropdown.set(self.tag_manager.active_tag_id)  # Update dropdown if it exists
        self.update_stats_display(self.tag_manager.find_tag(self.tag_manager.active_tag_id))
        self.update_imu_display(self.tag_manager.find_tag(self.tag_manager.active_tag_id))
        
        # 5. Log Data & Update UI
        if tag:
            # Log the raw distances + calculated position
            self.data_logger.add_log_entry(tag, data.get('distances', {}))
            
            if tag.id == self.tag_manager.active_tag_id:
                self._safe_after(0, lambda: self.update_stats_display(tag))
            
    def handle_imu_data(self, tag_id, imu_data):
        """Main thread handler specifically for parsed IMU data."""
        tag_id = str(tag_id) 
        tag = self.tag_manager.find_tag(tag_id)
        if tag:
            tag.update_imu_data(imu_data)
            self.log(f"Received IMU data for Tag {tag_id}")
            # If this is the active tag, update the UI
            if tag_id == self.tag_manager.active_tag_id:
                self._safe_after(0, lambda: self.update_imu_display(tag))

    def load_model(self):
        try:
            # 1. Load the NN Model using the Class Method
            self.positioning_model = UWBPositioningNN.load('data/models/nn_model')
            
            # 2. CRITICAL FIX: Override the feature columns
            # The model defaults to expecting 'median', 'std', etc.
            # We must tell it we are only using Raw Distances now.
            self.positioning_model.feature_columns = ['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']
            
            self.log("INFO: Successfully loaded pre-trained Neural Network model.")
            self.log("INFO: System is running on high-accuracy NN model.")
            
        except Exception as e:
            self.log(f"WARN: Could not load NN model ({e}).")
            self.log("INFO: Falling back to standard KNN model.")
            
            try:
                self.positioning_model = UWBPositioningKNN(anchor_ids=['A', 'B', 'C', 'D'])
                
                # Update KNN to also use the new Raw Distance format
                self.positioning_model.feature_columns = ['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']
                
                # Load the NEW merged data file, not the old one
                # We load manually to avoid column name errors
                import pandas as pd
                df = pd.read_csv('merged_training_data.csv')
                X = df[['Dist_A', 'Dist_B', 'Dist_C', 'Dist_D']].values
                y = df[['x', 'y']].values
                
                self.positioning_model.train_model(X, y)
                self.log("INFO: Fallback KNN model trained and active.")
                
            except Exception as e2:
                self.log(f"ERROR: Could not train fallback KNN model: {e2}")
                self.log("ERROR: No positioning model is active.")
                self.positioning_model.is_trained = False
                
    def start_background_reading(self):
        """Start the continuous RTLS packet reading loop"""
        if self.running:
            try:
                raw = hw.read_rtls_stream()               # non-blocking read
                if raw:
                    self.data_buffer.extend(raw)
                    self.process_buffer()
            except Exception as e:
                self.log(f"Read error: {e}")
            finally:
                if self.running:                          # ← important
                    self.read_job = self.root.after(5, self.start_background_reading)  # 5 ms = ~200 Hz
    
    def stop_background_reading(self):
        self.running = False
        if self.read_job:
            self.root.after_cancel(self.read_job)
        if self.anim:
            self.anim.event_source.stop()
    
    def on_close(self):
        """Safe shutdown sequence to prevent Tkinter errors."""
        print("INFO: Closing application...")
        self.running = False  # Stop all background loops
        
        # 1. Stop the Data Reading Thread
        if hasattr(self, 'read_job') and self.read_job:
            try:
                self.root.after_cancel(self.read_job)
            except:
                pass
        
        # 2. Stop Visualization Animation (Critical for Matplotlib crash)
        if hasattr(self, 'vis_system') and self.vis_system:
            self.vis_system.stop_animation()
        
        # 3. Stop Hardware
        try:
            hw.stop_positioning()
            # Give hardware a moment to process the stop command
            # before cutting the serial connection
            self.root.update() 
            hw.disconnect_from_anchor()
        except Exception as e:
            print(f"Error disconnecting hardware: {e}")

        # 4. Stop Network
        if hasattr(self, 'network_handler'):
            self.network_handler.stop_server()
        
        # 5. Destroy Window
        # We use a small delay or direct destruction, but verify the widget exists
        try:
            self.root.destroy()
        except Exception:
            pass # Already destroyed
        
    def show_navigation_window(self):
        """Creates and displays the Navigation Control Toplevel window."""
        if self.nav_window and self.nav_window.winfo_exists():
            self.nav_window.lift()
            return

        self.nav_window = tk.Toplevel(self.root)
        self.nav_window.title("Navigation Control")
        self.nav_window.geometry("700x550")

        # --- Search Tags Frame ---
        tags_frame = ttk.LabelFrame(self.nav_window, text="Search Tags")
        tags_frame.pack(fill=tk.X, padx=10, pady=5)
        
        cols = ("Tag ID", "x(cm)", "y(cm)", "z(cm)", "Status")
        self.nav_tag_tree = ttk.Treeview(tags_frame, columns=cols, show="headings", height=4)
        for col in cols:
            self.nav_tag_tree.heading(col, text=col)
            self.nav_tag_tree.column(col, width=80, anchor='center')
        self.nav_tag_tree.pack(fill=tk.X, padx=5, pady=5)
        # We will need a function to update this table periodically
        self._update_nav_tag_table()

        # --- Navigation Control Frame ---
        control_frame = ttk.LabelFrame(self.nav_window, text="Navigation Control")
        control_frame.pack(fill=tk.X, padx=10, pady=5)

        # Row 1: Tag ID and main actions
        row1 = ttk.Frame(control_frame); row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="Tag ID:").pack(side=tk.LEFT, padx=5)
        self.nav_tag_id_var = tk.StringVar(value="1")
        ttk.Entry(row1, textvariable=self.nav_tag_id_var, width=8).pack(side=tk.LEFT)
        ttk.Button(row1, text="Stop Movement", command=self._handle_nav_stop).pack(side=tk.LEFT, padx=20)
        
        #  Add the "Change Work State" button here
        ttk.Button(row1, text="Change Work State", command=self._handle_nav_change_state).pack(side=tk.LEFT, padx=10)
        ttk.Button(row1, text="Stop Movement", command=self._handle_nav_stop).pack(side=tk.LEFT, padx=5)
     
        #  Add the "Navigation Config" button
        ttk.Button(row1, text="Navigation Config", command=self._show_nav_config_window).pack(side=tk.RIGHT, padx=5)
        
        # Row 2: Speed
        row2 = ttk.Frame(control_frame); row2.pack(fill=tk.X, pady=2)
        ttk.Label(row2, text="Speed Level (1-5):").pack(side=tk.LEFT, padx=5)
        self.nav_speed_var = tk.StringVar(value="3")
        ttk.Combobox(row2, textvariable=self.nav_speed_var, values=["1", "2", "3", "4", "5"], width=5).pack(side=tk.LEFT)
        ttk.Button(row2, text="Set Speed", command=self._handle_nav_set_speed).pack(side=tk.LEFT, padx=5)

        ttk.Label(row2, text="Move Duration (ms):").pack(side=tk.LEFT, padx=20)
        self.nav_duration_var = tk.StringVar(value="200")
        ttk.Entry(row2, textvariable=self.nav_duration_var, width=8).pack(side=tk.LEFT)
        ttk.Button(row2, text="Set Duration", command=self._handle_nav_set_duration).pack(side=tk.LEFT, padx=5)

        # Row 3 & 4: Movement Buttons
        row3 = ttk.Frame(control_frame); row3.pack(fill=tk.X, pady=5)
        ttk.Button(row3, text="Forward-Left (FL)", command=lambda: self._handle_nav_move('FL')).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(row3, text="Forward (FS)", command=lambda: self._handle_nav_move('FS')).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(row3, text="Forward-Right (FR)", command=lambda: self._handle_nav_move('FR')).pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        row4 = ttk.Frame(control_frame); row4.pack(fill=tk.X, pady=2)
        ttk.Button(row4, text="Backward-Left (BL)", command=lambda: self._handle_nav_move('BL')).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(row4, text="Backward (BS)", command=lambda: self._handle_nav_move('BS')).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(row4, text="Backward-Right (BR)", command=lambda: self._handle_nav_move('BR')).pack(side=tk.LEFT, expand=True, fill=tk.X)

        # --- Auto Navigation Frame ---
        auto_frame = ttk.LabelFrame(self.nav_window, text="Auto Navigation")
        auto_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(auto_frame, text="Target X (m):").pack(side=tk.LEFT, padx=5)
        self.nav_target_x = tk.StringVar()
        ttk.Entry(auto_frame, textvariable=self.nav_target_x, width=8).pack(side=tk.LEFT)
        
        ttk.Label(auto_frame, text="Target Y (m):").pack(side=tk.LEFT, padx=5)
        self.nav_target_y = tk.StringVar()
        ttk.Entry(auto_frame, textvariable=self.nav_target_y, width=8).pack(side=tk.LEFT)
        ttk.Button(auto_frame, text="Start Movement to Target", command=self._handle_nav_auto).pack(side=tk.RIGHT, padx=5)

    def _handle_nav_change_state(self):
        """Handler for the 'Change Work State' button."""
        tag_id = self.nav_tag_id_var.get()
        if not tag_id:
            self.log("ERROR: No Tag ID provided for 'Change Work State' command.")
            return
        if self.nav_controller.change_work_state(tag_id):
            self.log(f"INFO: Sent 'Change Work State' command to tag '{tag_id}'.")
        else:
            self.log("WARN: Failed to send 'Change Work State' command.")
            
    def _show_nav_config_window(self):
        """Creates a simple Toplevel window for setting navigation parameters."""
        tag_id = self.nav_tag_id_var.get()
        if not tag_id:
            messagebox.showerror("Input Error", "Please enter a Tag ID before opening config.")
            return
    
        config_win = tk.Toplevel(self.nav_window)
        config_win.title(f"Config for Tag {tag_id}")
        config_win.transient(self.nav_window)
        config_win.grab_set()
    
        frame = ttk.Frame(config_win, padding="10")
        frame.pack()
    
        ttk.Label(frame, text="Max Speed (cm/s):").grid(row=0, column=0, sticky='w')
        speed_var = tk.StringVar(value="80")
        ttk.Entry(frame, textvariable=speed_var).grid(row=0, column=1, pady=5)
        
        ttk.Label(frame, text="Acceleration (cm/s²):").grid(row=1, column=0, sticky='w')
        accel_var = tk.StringVar(value="100")
        ttk.Entry(frame, textvariable=accel_var).grid(row=1, column=1, pady=5)
        
        def on_save():
            try:
                config_params = {
                    'max_speed': int(speed_var.get()),
                    'acceleration': int(accel_var.get())
                }
                self.nav_controller.send_config(tag_id, config_params)
                config_win.destroy()
            except ValueError:
                messagebox.showerror("Input Error", "Parameters must be numbers.", parent=config_win)
    
        ttk.Button(frame, text="Save and Send", command=on_save).grid(row=2, column=0, columnspan=2, pady=10)

    def _update_nav_tag_table(self):
        """Periodically updates the tag list in the navigation window."""
        if not (self.nav_window and self.nav_window.winfo_exists()):
            return # Stop updating if window is closed
        
        # Clear existing table
        for item in self.nav_tag_tree.get_children():
            self.nav_tag_tree.delete(item)
            
        # Add current tags
        tags = self.tag_manager.get_all_tags()
        for tag in tags:
            self.nav_tag_tree.insert('', 'end', values=(
                tag.id, f"{tag.x*100:.1f}", f"{tag.y*100:.1f}", f"{tag.z*100:.1f}", tag.status
            ))
        
        # Schedule next update
        self.nav_window.after(1000, self._update_nav_tag_table)

    # --- Handlers for Navigation Window Buttons ---
    def _handle_nav_move(self, direction_code):
        tag_id = self.nav_tag_id_var.get()
        if self.nav_controller.send_movement_command(tag_id, direction_code):
            self.log(f"INFO: Sent move '{direction_code}' to tag '{tag_id}'.")
        else:
            self.log(f"WARN: Failed to send move command to tag '{tag_id}'.")

    def _handle_nav_stop(self):
        tag_id = self.nav_tag_id_var.get()
        if self.nav_controller.stop_movement(tag_id):
            self.log(f"INFO: Sent STOP to tag '{tag_id}'.")
        else:
            self.log(f"WARN: Failed to send STOP to tag '{tag_id}'.")
            
    def _handle_nav_set_speed(self):
        tag_id = self.nav_tag_id_var.get()
        speed_level = self.nav_speed_var.get()
        if self.nav_controller.set_speed(tag_id, speed_level):
            self.log(f"INFO: Set speed for tag '{tag_id}' to level {speed_level}.")
        else:
            self.log(f"WARN: Failed to set speed for tag '{tag_id}'.")
            
    def _handle_nav_set_duration(self):
        duration = self.nav_duration_var.get()
        if self.nav_controller.set_movement_duration(duration):
            self.log(f"INFO: Movement duration set to {duration} ms.")
        else:
            self.log(f"ERROR: Invalid duration '{duration}'. Must be a number.")
            
    def _handle_nav_auto(self):
        try:
            tag_id = self.nav_tag_id_var.get()
            x = float(self.nav_target_x.get())
            y = float(self.nav_target_y.get())
            if self.nav_controller.set_target(tag_id, x, y):
                self.log(f"INFO: Sent auto-nav target ({x}, {y}) to tag '{tag_id}'.")
            else:
                self.log("WARN: Failed to send auto-nav command.")
        except ValueError:
            self.log("ERROR: Invalid target coordinates. Please enter numbers.")
            messagebox.showerror("Input Error", "Target coordinates must be numbers.")
            
    def retrieve_lidar_data(self):
        """Just sends the trigger command. The NetworkHandler will save the file when it arrives."""
        if not messagebox.askyesno("Confirm", "Process 'lidar_pose_data.bag' on robot?"):
            return
    
        self.log("INFO: Sending PROCESS_DATA command...")
        
        # We just send the trigger. The file will arrive asynchronously.
        # Ensure you have a method in NetworkHandler to send raw data
        if self.network_handler.client_socket:
            try:
                self.network_handler.client_socket.sendall("PROCESS_DATA".encode('utf-8'))
                self.log("INFO: Command sent. Watch log for file arrival.")
            except Exception as e:
                self.log(f"ERROR: Could not send command: {e}")
        else:
            self.log("ERROR: Robot not connected.")
            
if __name__ == "__main__":
    # Optional: Run with admin rights if needed
    # if utils.is_admin():
    root = ttkb.Window(themename="vapor") # Use ttkbootstrap Window Themes:morph
    app = MainApplication(root)
    root.mainloop()
    # else:
    #     print("Please run this application as an administrator.")
    
    
    