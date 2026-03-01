# calibration_window.py

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from datetime import datetime

# Import our refactored, clean modules
import hardware_interface as hw
from managers import TagManager, AnchorManager

class AnchorCalibrationWindow(tk.Toplevel):
    """A dedicated window for managing the anchor auto-calibration process."""

    def __init__(self, parent, tag_manager: TagManager, anchor_manager: AnchorManager, log_callback: callable):
        super().__init__(parent)
        self.title("Auto Calibration")
        self.transient(parent)
        self.grab_set()
        self.geometry("800x600")

        # Store references to the main application's components
        self.tag_manager = tag_manager
        self.anchor_manager = anchor_manager
        self.log_callback = log_callback # Use the main app's logger

        # Internal state
        self.calibration_state = "idle"  # idle, calibrating, done
        self.anchor_positions_temp = {}  # A temporary copy for this window

        self._initialize_temp_positions()
        self._init_ui()
        self._position_updater_loop() # Start the periodic UI update

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _initialize_temp_positions(self):
        """Creates a deep copy of anchor positions to work with, converting m to cm for display."""
        current_positions = self.anchor_manager.get_anchor_positions()
        for anchor_id, data in current_positions.items():
            self.anchor_positions_temp[anchor_id] = {
                'x': data['x'] * 100,
                'y': data['y'] * 100,
                'z': data['z'] * 100,
                'enabled': tk.BooleanVar(value=data.get('enabled', True))
            }

    def _init_ui(self):
        """Builds the user interface for the calibration window."""
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- Control buttons ---
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=(0, 10))
        self.start_btn = ttk.Button(btn_frame, text="Start Calibration", command=self.start_calibration)
        self.stop_btn = ttk.Button(btn_frame, text="Stop Calibration", command=self.stop_calibration)
        self.confirm_btn = ttk.Button(btn_frame, text="Confirm & Save Changes", command=self.confirm_changes)
        self.reset_btn = ttk.Button(btn_frame, text="Reset to Defaults", command=self.reset_coordinates)
        self.start_btn.pack(side=tk.LEFT, padx=2)
        self.stop_btn.pack(side=tk.LEFT, padx=2)
        self.confirm_btn.pack(side=tk.LEFT, padx=2)
        self.reset_btn.pack(side=tk.LEFT, padx=2)

        # --- Base station selection (UI remains the same) ---
        station_frame = ttk.Frame(main_frame)
        station_frame.pack(fill=tk.X, pady=5)
        # ... (all the Comboboxes and Labels for Origin, Direction, etc.)
        ttk.Label(station_frame, text="Origin BS:").pack(side=tk.LEFT, padx=(0, 2))
        self.origin_combo = ttk.Combobox(station_frame, values=['A', 'B', 'C', 'D', 'E', 'F'], width=4)
        self.origin_combo.set('A'); self.origin_combo.pack(side=tk.LEFT, padx=2)
        ttk.Label(station_frame, text="Direction:").pack(side=tk.LEFT, padx=(10, 2))
        self.direction_combo = ttk.Combobox(station_frame, values=['+X', '+Y', '-X', '-Y'], width=4)
        self.direction_combo.set('+X'); self.direction_combo.pack(side=tk.LEFT, padx=2)
        ttk.Label(station_frame, text="Aux Point BS:").pack(side=tk.LEFT, padx=(10, 2))
        self.aux_combo = ttk.Combobox(station_frame, values=['A', 'B', 'C', 'D', 'E', 'F'], width=4)
        self.aux_combo.set('B'); self.aux_combo.pack(side=tk.LEFT, padx=2)
        ttk.Label(station_frame, text="Aux Point Axis:").pack(side=tk.LEFT, padx=(10, 2))
        self.axis_combo = ttk.Combobox(station_frame, values=['X', 'Y'], width=3)
        self.axis_combo.set('X'); self.axis_combo.pack(side=tk.LEFT, padx=2)
        
        # --- Anchor position table ---
        table_frame = ttk.Frame(main_frame)
        table_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        columns = ("station", "x", "y", "z")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings")
        self.tree.heading("station", text="Base Station")
        self.tree.heading("x", text="X (cm)")
        self.tree.heading("y", text="Y (cm)")
        self.tree.heading("z", text="Z (cm)")
        # ... (Column widths, etc.)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.tree.bind("<Button-1>", self._handle_tree_click)
        self.update_table()

        # --- Log display ---
        self.log_display = scrolledtext.ScrolledText(main_frame, height=8, wrap=tk.WORD, font=("Consolas", 8))
        self.log_display.pack(fill=tk.BOTH, expand=True, pady=(5, 0))
        self._log("Calibration window initialized.")
        self.update_button_states()

    def _log(self, message: str):
        """Internal logger that also calls the main app's logger."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        self.log_display.config(state=tk.NORMAL)
        self.log_display.insert(tk.END, log_message)
        self.log_display.see(tk.END)
        self.log_display.config(state=tk.DISABLED)
        # Also send the message to the main application's log
        self.log_callback(f"CALIB: {message}")

    def update_table(self):
        """Refreshes the Treeview with current data from self.anchor_positions_temp."""
        for item in self.tree.get_children():
            self.tree.delete(item)
        for anchor_id, data in self.anchor_positions_temp.items():
            self.tree.insert("", "end", iid=anchor_id, values=(
                f"Station {anchor_id} {'(Enabled)' if data['enabled'].get() else ''}",
                f"{data['x']:.1f}", f"{data['y']:.1f}", f"{data['z']:.1f}"
            ))

    def _handle_tree_click(self, event):
        """Allows enabling/disabling anchors by clicking on a row."""
        item_id = self.tree.identify_row(event.y)
        if item_id and item_id in self.anchor_positions_temp:
            current_state = self.anchor_positions_temp[item_id]['enabled'].get()
            self.anchor_positions_temp[item_id]['enabled'].set(not current_state)
            self._log(f"Station {item_id} toggled {'ON' if not current_state else 'OFF'}.")
            self.update_table()
    
    def _position_updater_loop(self):
        """Periodically syncs the temp positions with the main AnchorManager during calibration."""
        if self.calibration_state == "calibrating":
            self._initialize_temp_positions() # Re-sync from the main source of truth
            self.update_table()
        # Schedule the next check
        self.after(500, self._position_updater_loop)

    def start_calibration(self):
        """Starts the auto-calibration process using the hardware_interface."""
        if not hw.is_connected():
            self._log("Error: Must be connected to an anchor to start calibration.")
            return

        self._log("Sending 'Start Calibration' command...")
        # REFACTORED: Use the clean hardware interface function
        if hw.start_positioning(): # The same command starts positioning and calibration
            self._log("Command sent successfully. Anchor is now in calibration/positioning mode.")
            self.calibration_state = "calibrating"
            self.update_button_states()
        else:
            self._log("Error: Failed to send 'Start' command to anchor.")

    def stop_calibration(self):
        """Stops the calibration process."""
        self._log("Sending 'Stop Calibration' command...")
        # REFACTORED: Use the clean hardware interface function
        if hw.stop_positioning():
            self._log("Calibration/Positioning stopped. You can now save the results.")
            self.calibration_state = "done"
            self.update_button_states()
        else:
            self._log("Error: Failed to send 'Stop' command to anchor.")
            
    def confirm_changes(self):
        """Saves the calibrated positions back to the main AnchorManager."""
        self._log("Saving calibrated positions to the main system...")
        for anchor_id, data in self.anchor_positions_temp.items():
            # Convert cm back to meters before saving
            self.anchor_manager.update_anchor_position(
                anchor_id, data['x'] / 100.0, data['y'] / 100.0, data['z'] / 100.0
            )
        self._log("Anchor positions updated successfully.")
        messagebox.showinfo("Success", "Anchor positions have been updated in the main application.")
        self.destroy() # Close the window after saving

    def reset_coordinates(self):
        """Resets the temporary coordinates in the window to the application defaults."""
        from config import DEFAULT_ANCHOR_POSITIONS # Re-import to ensure freshness
        for anchor_id, data in DEFAULT_ANCHOR_POSITIONS.items():
            if anchor_id in self.anchor_positions_temp:
                self.anchor_positions_temp[anchor_id]['x'] = data['x'] * 100
                self.anchor_positions_temp[anchor_id]['y'] = data['y'] * 100
                self.anchor_positions_temp[anchor_id]['z'] = data['z'] * 100
                self.anchor_positions_temp[anchor_id]['enabled'].set(data.get('enabled', True))
        self.update_table()
        self._log("Coordinates in this window have been reset to defaults.")

    def update_button_states(self):
        """Enable/disable buttons based on the current calibration state."""
        is_idle = self.calibration_state == "idle"
        is_calibrating = self.calibration_state == "calibrating"
        is_done = self.calibration_state == "done"
        
        self.start_btn.config(state="normal" if is_idle else "disabled")
        self.stop_btn.config(state="normal" if is_calibrating else "disabled")
        self.confirm_btn.config(state="normal" if is_done else "disabled")
        self.reset_btn.config(state="normal" if is_idle or is_done else "disabled")

    def on_close(self):
        """Handles window closing safely."""
        if self.calibration_state == "calibrating":
            if messagebox.askyesno("Calibration in Progress",
                                "Calibration is running. Do you want to stop it and close the window?"):
                self.stop_calibration()
                self.destroy()
        else:
            self.destroy()