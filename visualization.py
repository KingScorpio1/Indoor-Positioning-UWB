# visualization.py

import matplotlib
# CHANGE 'Agg' to 'TkAgg' or 'Qt5Agg'
matplotlib.use('Agg')

import numpy as np
import tkinter as tk
from tkinter import ttk, TclError, messagebox
from collections import deque 
import matplotlib.pyplot as plt
from matplotlib.image import imread
from matplotlib.animation import FuncAnimation
from matplotlib.offsetbox import OffsetImage
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

def load_image(path, scale=0.1):
    img = imread(path)
    return OffsetImage(img, zoom=scale)

class VisualizationSystem:
    
    def __init__(self, root, tag_manager, anchor_manager, logger_func):
        self.root = root
        self.tag_manager = tag_manager
        self.anchor_manager = anchor_manager
        self.log = logger_func
        
        # --- Style plots for 'vapor' theme ---
        plt.style.use('dark_background')
        self.plot_fig_color = "#343a40"  # Main figure background (matches ttkb 'vapor')
        self.plot_ax_color = "#212121"   # Inner plot background
        self.plot_text_color = "#CCCCCC" # Light gray for text/ticks
        self.plot_grid_color = "#555"    # Darker gray for grid

        # --- Figure and Axes Setup ---
        self.figure_2d, self.ax2d = plt.subplots(figsize=(8, 6), dpi=100, facecolor=self.plot_fig_color)
        self.ax2d.set_facecolor(self.plot_ax_color)
        
        self.figure_3d = plt.figure(figsize=(8, 6), dpi=100, facecolor=self.plot_fig_color)
        self.ax3d = self.figure_3d.add_subplot(111, projection='3d', facecolor=self.plot_ax_color)
        
        # --- Artist Dictionaries for Multi-Tag Support ---
        # These will store the artists (lines, icons, text) for each tag, keyed by tag_id.
        self.tag_artists_2d = {}
        self.tag_artists_3d = {}
        self.anchor_artists_2d = {}
        self.anchor_artists_3d = {}
        
        # Use a colormap to automatically assign different colors to new tags
        self.color_map = plt.get_cmap('tab10')
        
        self.show_trajectory = True
        self.show_tag_labels = True
        self.show_axes = True
        self.show_ranging_circles = False
        self.tag_size = 10
        self.map_width = 500
        self.map_height = 500
        self.scale_factor = 1.0
        self.mouse_drag_start = None
        self.view_offset = [0, 0]
        self.zoom_level = 1.0
        
        
        # --- UI Control Variables (now part of the class) ---
        self.grid2d_var = tk.BooleanVar(value=True)
        self.xlim_2d = tk.StringVar(value="10") # Sensible default
        self.ylim_2d = tk.StringVar(value="10")
        self.color_var_2d = tk.StringVar(value="white")
        self.scale_var_2d = tk.StringVar(value="linear")
        
        self.grid3d_var = tk.BooleanVar(value=True)
        self.xlim_3d = tk.StringVar(value="10")
        self.ylim_3d = tk.StringVar(value="10")
        self.zlim_3d = tk.StringVar(value="5")
        self.color_var_3d = tk.StringVar(value="white")
        self.scale_var_3d_x = tk.StringVar(value="linear")
        self.scale_var_3d_y = tk.StringVar(value="linear")
        self.scale_var_3d_z = tk.StringVar(value="linear")
        
        self.bg_colors = ["white", "black", "lightgray", "lightblue", "beige"]
        
        # --- UI Variables for Manual View Control ---
        self.origin_x_var = tk.StringVar(value="2.5")
        self.origin_y_var = tk.StringVar(value="2.5")
        self.scale_var = tk.StringVar(value="6.0") # meters
        self.autoscale_on = tk.BooleanVar(value=True)
    
        self._init_static_artists()
        
    def _init_static_artists(self):
        """Initializes the plot elements that do not change frame-to-frame."""
        # --- 2D Plot Setup ---
        self.ax2d.set_title("2D Tracking View", color=self.plot_text_color)
        self.ax2d.set_xlabel("X (m)", color=self.plot_text_color)
        self.ax2d.set_ylabel("Y (m)", color=self.plot_text_color)
        self.ax2d.grid(True, color=self.plot_grid_color, linestyle=':')
        self.ax2d.set_aspect('equal', adjustable='box')
        # Style ticks and spines
        self.ax2d.tick_params(axis='x', colors=self.plot_text_color)
        self.ax2d.tick_params(axis='y', colors=self.plot_text_color)
        for spine in self.ax2d.spines.values():
            spine.set_color(self.plot_text_color)

        # --- 3D Plot Setup ---
        self.ax3d.set_title("3D Tracking View", color=self.plot_text_color)
        self.ax3d.set_xlabel("X (m)", color=self.plot_text_color)
        self.ax3d.set_ylabel("Y (m)", color=self.plot_text_color)
        self.ax3d.set_zlabel("Z (m)", color=self.plot_text_color)
        self.ax3d.grid(True, color=self.plot_grid_color, linestyle=':')
        # Style ticks and panes
        self.ax3d.tick_params(axis='x', colors=self.plot_text_color)
        self.ax3d.tick_params(axis='y', colors=self.plot_text_color)
        self.ax3d.tick_params(axis='z', colors=self.plot_text_color)
        self.ax3d.xaxis.pane.set_edgecolor(self.plot_text_color)
        self.ax3d.yaxis.pane.set_edgecolor(self.plot_text_color)
        self.ax3d.zaxis.pane.set_edgecolor(self.plot_text_color)

        # Initialize anchor artists once
        anchor_positions = self.anchor_manager.get_anchor_positions()
        for anchor_id, pos in anchor_positions.items():
            # 2D Artists
            anchor_color = self.plot_text_color # Use the light gray
            sc_2d = self.ax2d.scatter(pos['x'], pos['y'], c=anchor_color, marker='s', s=100, label=f'Anchor {anchor_id}')
            txt_2d = self.ax2d.text(pos['x'], pos['y'] + 0.2, anchor_id, ha='center', va='bottom', fontsize=9, color=anchor_color)
            self.anchor_artists_2d[anchor_id] = {'scatter': sc_2d, 'text': txt_2d}

            # 3D Artists
            sc_3d = self.ax3d.scatter(pos['x'], pos['y'], pos['z'], c=anchor_color, marker='^', s=100, label=f'Anchor {anchor_id}')
            txt_3d = self.ax3d.text(pos['x'], pos['y'], pos['z'] + 0.2, anchor_id, ha='center', va='bottom', fontsize=9, color=anchor_color)
            self.anchor_artists_3d[anchor_id] = {'scatter': sc_3d, 'text': txt_3d} 
    
    def setup_gui_frames(self, frame2d_parent, frame3d_parent):
        """Creates and places the plot canvases and all controls into the parent Tkinter frames."""
        
        #======================================================================
        # 2D FRAME SETUP
        #======================================================================
                
        canvas2d = FigureCanvasTkAgg(self.figure_2d, master=frame2d_parent)
        canvas2d.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas2d.mpl_connect('button_press_event', self.handle_target_click)
        
        toolbar2d = NavigationToolbar2Tk(canvas2d, frame2d_parent)
        toolbar2d.update()

        controls_2d_frame = ttk.Frame(frame2d_parent)
        controls_2d_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # --- 2D Controls ---
        ttk.Checkbutton(controls_2d_frame, text="Show Grid", variable=self.grid2d_var).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(controls_2d_frame, text="Scale:").pack(side=tk.LEFT)
        scale_combo_2d = ttk.Combobox(controls_2d_frame, textvariable=self.scale_var_2d, values=["linear", "log"], width=6)
        scale_combo_2d.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(controls_2d_frame, text="BG Color:").pack(side=tk.LEFT)
        color_combo_2d = ttk.Combobox(controls_2d_frame, textvariable=self.color_var_2d, values=self.bg_colors, width=10)
        color_combo_2d.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(controls_2d_frame, text="X Limit:").pack(side=tk.LEFT)
        ttk.Entry(controls_2d_frame, textvariable=self.xlim_2d, width=5).pack(side=tk.LEFT, padx=2)
        
        ttk.Label(controls_2d_frame, text="Y Limit:").pack(side=tk.LEFT)
        ttk.Entry(controls_2d_frame, textvariable=self.ylim_2d, width=5).pack(side=tk.LEFT, padx=2)

        ttk.Button(controls_2d_frame, text="Autoscale", command=self.toggle_2d_autoscale).pack(side=tk.LEFT, padx=5)
        
        # Bind callbacks for interactive updates
        self.xlim_2d.trace_add("write", self.update_2d_view_settings)
        self.ylim_2d.trace_add("write", self.update_2d_view_settings)
        scale_combo_2d.bind("<<ComboboxSelected>>", self.update_2d_view_settings)
        color_combo_2d.bind("<<ComboboxSelected>>", self.update_2d_view_settings)
        
        # --- NEW: Frame for Manual View Controls ---
        manual_view_frame = ttk.LabelFrame(frame2d_parent, text="Manual View Control")
        manual_view_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(manual_view_frame, text="Origin X:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(manual_view_frame, textvariable=self.origin_x_var, width=6).pack(side=tk.LEFT)
        
        ttk.Label(manual_view_frame, text="Origin Y:").pack(side=tk.LEFT, padx=5)
        ttk.Entry(manual_view_frame, textvariable=self.origin_y_var, width=6).pack(side=tk.LEFT)
        
        ttk.Label(manual_view_frame, text="Scale (m):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(manual_view_frame, textvariable=self.scale_var, width=6).pack(side=tk.LEFT)
        
        ttk.Button(manual_view_frame, text="Apply View", command=self.apply_manual_view).pack(side=tk.LEFT, padx=10)
        
        ttk.Checkbutton(manual_view_frame, text="Autoscale", variable=self.autoscale_on).pack(side=tk.LEFT, padx=5)

        #======================================================================
        # 3D FRAME SETUP
        #======================================================================
            
        canvas3d = FigureCanvasTkAgg(self.figure_3d, master=frame3d_parent)
        canvas3d.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        toolbar3d = NavigationToolbar2Tk(canvas3d, frame3d_parent)
        toolbar3d.update()
        
        # --- 3D Controls (View Angle) ---
        view_frame_3d = ttk.LabelFrame(frame3d_parent, text="3D View Angle")
        view_frame_3d.pack(fill=tk.X, padx=10, pady=5)
        
        self.elev_slider = tk.Scale(view_frame_3d, from_=0, to=90, orient=tk.HORIZONTAL, label="Elevation")
        self.elev_slider.set(20)
        self.elev_slider.pack(fill=tk.X, expand=True)
        
        self.azim_slider = tk.Scale(view_frame_3d, from_=-180, to=180, orient=tk.HORIZONTAL, label="Azimuth")
        self.azim_slider.set(120)
        self.azim_slider.pack(fill=tk.X, expand=True)

        # --- 3D Controls (Limits, Color) ---
        controls_3d_frame = ttk.Frame(frame3d_parent)
        controls_3d_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Checkbutton(controls_3d_frame, text="Show Grid", variable=self.grid3d_var).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(controls_3d_frame, text="BG Color:").pack(side=tk.LEFT)
        color_combo_3d = ttk.Combobox(controls_3d_frame, textvariable=self.color_var_3d, values=self.bg_colors, width=10)
        color_combo_3d.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Label(controls_3d_frame, text="X Scale:").pack(side=tk.LEFT)
        scale_combo_3d_x = ttk.Combobox(controls_3d_frame, textvariable=self.scale_var_3d_x, values=["linear", "log"], width=6)
        scale_combo_3d_x.pack(side=tk.LEFT)
        ttk.Label(controls_3d_frame, text="Y Scale:").pack(side=tk.LEFT)
        scale_combo_3d_y = ttk.Combobox(controls_3d_frame, textvariable=self.scale_var_3d_y, values=["linear", "log"], width=6)
        scale_combo_3d_y.pack(side=tk.LEFT)
        ttk.Label(controls_3d_frame, text="Z Scale:").pack(side=tk.LEFT)
        scale_combo_3d_z = ttk.Combobox(controls_3d_frame, textvariable=self.scale_var_3d_z, values=["linear", "log"], width=6)
        scale_combo_3d_z.pack(side=tk.LEFT)
        
        ttk.Label(controls_3d_frame, text="X:").pack(side=tk.LEFT)
        xlim_entry_3d = ttk.Entry(controls_3d_frame, textvariable=self.xlim_3d, width=5)
        xlim_entry_3d.pack(side=tk.LEFT)
        ttk.Label(controls_3d_frame, text="Y:").pack(side=tk.LEFT)
        ylim_entry_3d = ttk.Entry(controls_3d_frame, textvariable=self.ylim_3d, width=5)
        ylim_entry_3d.pack(side=tk.LEFT)
        ttk.Label(controls_3d_frame, text="Z:").pack(side=tk.LEFT)
        zlim_entry_3d = ttk.Entry(controls_3d_frame, textvariable=self.zlim_3d, width=5)
        zlim_entry_3d.pack(side=tk.LEFT)
        
        ttk.Button(controls_3d_frame, text="Autoscale", command=self.toggle_3d_autoscale).pack(side=tk.LEFT, padx=5)

        # Bind callbacks for interactive updates
        self.xlim_3d.trace_add("write", self.update_3d_view_settings)
        self.ylim_3d.trace_add("write", self.update_3d_view_settings)
        self.zlim_3d.trace_add("write", self.update_3d_view_settings)
        color_combo_3d.bind("<<ComboboxSelected>>", self.update_3d_view_settings)
        
    def start_animation(self):
        """Starts the animation loops for both plots."""
        # Use blit=False for simplicity and better compatibility, especially with 3D plots.
        self.ani2d = FuncAnimation(self.figure_2d, self._update_plots, interval=100, blit=False, cache_frame_data=False)
        self.ani3d = FuncAnimation(self.figure_3d, self._update_plots, interval=100, blit=False, cache_frame_data=False)
        
    def stop_animation(self):
        """Stops the animation timers to allow clean shutdown."""
        if hasattr(self, 'ani2d'):
            self.ani2d.event_source.stop()
        if hasattr(self, 'ani3d'):
            self.ani3d.event_source.stop()
        print("Visualization animations stopped.")
        
    # visualization.py -> _create_artists_for_tag

    def _create_artists_for_tag(self, tag_id: str):
        color = self.color_map(len(self.tag_artists_2d) % 10)
    
        # 2D Artists
        line_ai, = self.ax2d.plot([], [], 
                                  linestyle='-', 
                                  color='#00FF00', 
                                  linewidth=2, 
                                  alpha=0.8,
                                  label=f'Tag {tag_id} (AI)')
        
        line_raw, = self.ax2d.plot([], [], 
                                   linestyle='--', 
                                   color='#FF4444', 
                                   linewidth=1.5, 
                                   alpha=0.6, 
                                   label=f'Tag {tag_id} (Raw)')
        
        point2d = self.ax2d.scatter([], [], 
                                   s=150, 
                                   color=color, 
                                   marker='o', 
                                   edgecolors='white', 
                                   zorder=10)
        
        text2d = self.ax2d.text(0, 0, tag_id, 
                               ha='center', 
                               va='center', 
                               fontsize=8, 
                               color='white', 
                               fontweight='bold')
        
        self.tag_artists_2d[tag_id] = {
            'line_ai': line_ai, 
            'line_raw': line_raw, 
            'point': point2d, 
            'text': text2d
        }
    
        # 3D Artists
        line3d, = self.ax3d.plot([], [], [], 
                                linestyle='-', 
                                color=color, 
                                alpha=0.8,
                                linewidth=2)
        
        point3d = self.ax3d.scatter([], [], [], 
                                   s=100, 
                                   color=color, 
                                   marker='o')
        
        self.tag_artists_3d[tag_id] = {
            'line': line3d, 
            'point': point3d
        }
    
        # Legend fix: Move outside
        self.ax2d.legend(loc='upper left', 
                        bbox_to_anchor=(1.05, 1), 
                        borderaxespad=0, 
                        facecolor='#212121', 
                        labelcolor='white')
        
        # Adjust figure to make room for legend
        self.figure_2d.subplots_adjust(right=0.75)

    def _update_plots(self, frame_num: int):
        all_tags = self.tag_manager.get_all_tags()
        
        # Calculate Bounding Box (Start with Anchor positions)
        anchors = self.anchor_manager.get_anchor_positions()
        pos_list = list(anchors.values())
        if not pos_list: 
            return
        
        min_x, max_x = min(p['x'] for p in pos_list), max(p['x'] for p in pos_list)
        min_y, max_y = min(p['y'] for p in pos_list), max(p['y'] for p in pos_list)
        min_z, max_z = min(p['z'] for p in pos_list), max(p['z'] for p in pos_list)
    
        for tag in all_tags:
            if tag.id not in self.tag_artists_2d: 
                self._create_artists_for_tag(tag.id)
                
            art2d = self.tag_artists_2d.get(tag.id)
            art3d = self.tag_artists_3d.get(tag.id)
            
            if not art2d or not art3d:
                continue
    
            if tag.status == "Inactive":
                art2d['point'].set_visible(False)
                art2d['text'].set_visible(False)
                art2d['line_ai'].set_visible(False)
                art2d['line_raw'].set_visible(False)
                continue
    
            # Update 2D
            art2d['point'].set_offsets([[tag.x, tag.y]])
            art2d['point'].set_visible(True)
            art2d['text'].set_position((tag.x, tag.y))
            art2d['text'].set_visible(True)
            
            # Draw AI and Raw Trails - FIXED: Check show_trajectory flag
            if self.tag_manager.is_show_trajectory:
                # AI Trail (Green Solid)
                if hasattr(tag, 'history_ai') and len(tag.history_ai['x']) > 1:
                    art2d['line_ai'].set_data(list(tag.history_ai['x']), list(tag.history_ai['y']))
                    art2d['line_ai'].set_visible(True)
                    art2d['line_ai'].set_zorder(1)
                else:
                    art2d['line_ai'].set_visible(False)
                    
                # Raw Trail (Red Dashed)  
                if hasattr(tag, 'history_raw') and len(tag.history_raw['x']) > 1:
                    art2d['line_raw'].set_data(list(tag.history_raw['x']), list(tag.history_raw['y']))
                    art2d['line_raw'].set_visible(True)
                    art2d['line_raw'].set_zorder(2)
                else:
                    art2d['line_raw'].set_visible(False)
            else:
                art2d['line_ai'].set_visible(False)
                art2d['line_raw'].set_visible(False)
    
            # Update 3D
            if hasattr(tag, 'x') and hasattr(tag, 'y') and hasattr(tag, 'z'):
                art3d['point']._offsets3d = ([tag.x], [tag.y], [tag.z])
                
                if self.tag_manager.is_show_trajectory and hasattr(tag, 'movement_history') and len(tag.movement_history) > 1:
                    h = list(tag.movement_history)
                    art3d['line'].set_data([p[0] for p in h], [p[1] for p in h])
                    art3d['line'].set_3d_properties([p[2] for p in h])
                    art3d['line'].set_visible(True)
                else:
                    art3d['line'].set_visible(False)
    
            # Expand bounding box to include tag
            min_x, max_x = min(min_x, tag.x), max(max_x, tag.x)
            min_y, max_y = min(min_y, tag.y), max(max_y, tag.y)
            if hasattr(tag, 'z'):
                min_z, max_z = min(min_z, tag.z), max(max_z, tag.z)
    
        # Apply View Limits
        if self.autoscale_on.get():
            pad = 1.0
            self.ax2d.set_xlim(min_x - pad, max_x + pad)
            self.ax2d.set_ylim(min_y - pad, max_y + pad)
            self.ax3d.set_xlim(min_x - pad, max_x + pad)
            self.ax3d.set_ylim(min_y - pad, max_y + pad)
            self.ax3d.set_zlim(0, max(3, max_z + 1))  # Dynamic Z limit
        
        for tag in all_tags:
            print(f"Tag {tag.id}:")
            print(f"  - Status: {tag.status}")
            print(f"  - Position: ({tag.x}, {tag.y})")
            print(f"  - AI History length: {len(tag.history_ai['x'] if hasattr(tag, 'history_ai') else 0)}")
            print(f"  - Raw History length: {len(tag.history_raw['x'] if hasattr(tag, 'history_raw') else 0)}")
            print(f"  - Show trajectory: {self.tag_manager.is_show_trajectory}")
    
        self.figure_2d.canvas.draw_idle()
        self.figure_3d.canvas.draw_idle()
    
    def _update_2d_plot(self, frame=None):
        """Redraws the 2D plot. Called repeatedly by FuncAnimation."""
        
        # 1. Clear previous frame (This wipes settings, so we must re-apply them!)
        # self.ax2d.clear()
        
        # 2. Re-apply Styles & LIMITS (This was missing!)
        self.ax2d.set_facecolor(self.plot_ax_color)
        self.ax2d.grid(True, color=self.plot_grid_color)
        self.ax2d.set_xlabel("X (meters)", color=self.plot_text_color)
        self.ax2d.set_ylabel("Y (meters)", color=self.plot_text_color)
        
        # --- CRITICAL FIX: HARDCODE THE ROOM SIZE ---
        # Adjust these numbers to match your actual room size in Meters
        self.ax2d.set_xlim(-1, 10) 
        self.ax2d.set_ylim(-1, 10)
        self.ax2d.set_aspect('equal', 'box')

        # 3. Draw Anchors
        anchors = self.anchor_manager.get_anchors()
        for anchor_id, anchor in anchors.items():
            try:
                self.ax2d.scatter(anchor.x, anchor.y, color='red', s=100, marker='s', label="Anchor")
                self.ax2d.text(anchor.x, anchor.y + 0.2, f"A-{anchor_id}", 
                             color='red', fontsize=10, ha='center')
            except:
                pass # Skip invalid anchors

        # 4. Draw Tags (The Blue Dot)
        tags = self.tag_manager.get_all_tags()
        
        # DEBUG: Uncomment if you still see nothing
        # if not tags: print("DEBUG: No tags found in manager.")
        
        for tag_id, tag in tags.items():
            # Safety Check: ensure x/y are valid numbers
            if tag.x is None or tag.y is None:
                continue
                
            # DEBUG: Check if data is in mm (e.g., 1500) or m (1.5)
            print(f"Plotting Tag {tag_id} at ({tag.x}, {tag.y})")
            # A. Draw Raw Path (Red Dashed) - "Where hardware thinks it is"
            if len(tag.history_raw['x']) > 1:
                self.ax2d.plot(
                    list(tag.history_raw['x']), 
                    list(tag.history_raw['y']), 
                    color='#FF4444',    # Bright Red
                    linestyle='--',     # Dashed
                    linewidth=1, 
                    alpha=0.6,
                    label='Raw Input' if tag_id == self.tag_manager.active_tag_id else ""
                )

            # B. Draw AI Path (Green Solid) - "Where AI thinks it is"
            if len(tag.history_ai['x']) > 1:
                self.ax2d.plot(
                    list(tag.history_ai['x']), 
                    list(tag.history_ai['y']), 
                    color='#00FF00',    # Bright Green
                    linestyle='-',      # Solid
                    linewidth=2, 
                    alpha=0.9,
                    label='AI Corrected' if tag_id == self.tag_manager.active_tag_id else ""
                )

            # Plot the dot
            self.ax2d.scatter(tag.x, tag.y, color='#00ff00', s=150, label=f"Tag {tag_id}")
            
            # Add text label
            text = f"Tag {tag_id}\n({tag.x:.2f}, {tag.y:.2f})"
            self.ax2d.text(tag.x + 0.2, tag.y, text, color='#00ff00', fontsize=9)
            
            # Draw History Trail (Optional)
            if len(tag.history['x']) > 1:
                self.ax2d.plot(list(tag.history['x']), list(tag.history['y']), 
                             color='#00ff00', linestyle='--', alpha=0.5)

        # 5. Set Title
        active = self.tag_manager.active_tag_id if self.tag_manager.active_tag_id else "None"
        self.ax2d.set_title(f"Live Position | Active Tag: {active}", color=self.plot_text_color)
        if len(tags) > 0:
            self.ax2d.legend(loc='upper right', fontsize='small', facecolor='#212121', labelcolor='white')
    
    def _update_3d(self, frame):
        if not self.running: return
        # (Simplified 3D update logic similar to 2D)
        pass
    
    def setup_3d_controls(self, parent):
        """Create 3D control panel"""
        control_frame = ttk.LabelFrame(parent, text="3D View Controls")
        
        # Axis ranges
        self.x_range = tk.Scale(control_frame, from_=1, to=100, 
                               orient=tk.HORIZONTAL, label="X Range (m)")
        self.x_range.set(50)
        self.x_range.pack(fill=tk.X)
        self.y_range = tk.Scale(control_frame, from_=1, to=100, 
                               orient=tk.HORIZONTAL, label="Y Range (m)")
        self.y_range.set(50)
        self.y_range.pack(fill=tk.Y)
        self.z_range = tk.Scale(control_frame, from_=1, to=100, 
                               orient=tk.HORIZONTAL, label="Z Range (m)")
        self.z_range.set(50)
        self.z_range.pack(fill=tk.Z)
        
        # Grid steps
        self.x_step = tk.Scale(control_frame, from_=0.1, to=10, resolution=0.1,
                              orient=tk.HORIZONTAL, label="X Grid Step (m)")
        self.x_step.set(1)
        self.x_step.pack(fill=tk.X)
        self.y_step = tk.Scale(control_frame, from_=0.1, to=10, resolution=0.1,
                              orient=tk.HORIZONTAL, label="Y Grid Step (m)")
        self.y_step.set(1)
        self.y_step.pack(fill=tk.Y)
        self.z_step = tk.Scale(control_frame, from_=0.1, to=10, resolution=0.1,
                              orient=tk.HORIZONTAL, label="Z Grid Step (m)")
        self.z_step.set(1)
        self.z_step.pack(fill=tk.Z)

        
        # Trajectory length
        self.traj_length = tk.Scale(control_frame, from_=10, to=1000,
                                   orient=tk.HORIZONTAL, label="Trajectory Points")
        self.traj_length.set(100)
        self.traj_length.pack(fill=tk.X)
        
        # Apply button
        ttk.Button(control_frame, text="Apply Settings",
                  command=self.apply_3d_settings).pack(pady=5)
        return control_frame
    
    def apply_3d_settings(self):
        """Apply 3D view settings"""
        self.ax_3d.set_xlim(0, self.x_range.get())
        self.ax_3d.set_ylim(0, self.y_range.get())
        self.ax_3d.set_zlim(0, self.z_range.get())
        
        # Update grid
        self.ax_3d.set_xticks(np.arange(0, self.x_range.get()+1, self.x_step.get()))
        self.ax_3d.set_yticks(np.arange(0, self.y_range.get()+1, self.y_step.get()))
        self.ax_3d.set_zticks(np.arange(0, self.z_range.get()+1, self.z_step.get()))
        
        # Update trajectory length
        for tag in self.tag_manager.tags.values():
            tag.movement_history = deque(maxlen=self.traj_length.get())
        
        self.figure_3d.canvas.draw_idle()
        
    def apply_manual_view(self):
        """Applies the origin and scale settings from the UI to the 2D plot."""
        try:
            origin_x = float(self.origin_x_var.get())
            origin_y = float(self.origin_y_var.get())
            scale = float(self.scale_var.get())
            
            if scale <= 0:
                self.log("ERROR: Scale must be a positive number.")
                return
    
            # When we apply a manual view, we turn off autoscaling
            self.autoscale_on.set(False)
            
            half_scale = scale / 2.0
            self.ax2d.set_xlim(origin_x - half_scale, origin_x + half_scale)
            self.ax2d.set_ylim(origin_y - half_scale, origin_y + half_scale)
            self.log(f"Manual view applied: Center=({origin_x}, {origin_y}), Scale={scale}m")
            
            # We need to redraw the canvas to see the change immediately
            self.figure_2d.canvas.draw_idle()
            
        except ValueError:
            self.log("ERROR: Invalid number for Origin or Scale.")
            messagebox.showerror("Input Error", "Origin and Scale must be valid numbers.")
    
    def update_2d_view_settings(self, *args):
        """Callback to apply 2D view settings from UI controls."""
        try:
            # Stop autoscaling to allow manual limits
            self.ax2d.autoscale(enable=False)
            self.ax2d.set_xlim(0, float(self.xlim_2d.get()))
            self.ax2d.set_ylim(0, float(self.ylim_2d.get()))
            self.ax2d.set_xscale(self.scale_var_2d.get())
            self.ax2d.set_yscale(self.scale_var_2d.get())
            self.ax2d.set_facecolor(self.color_var_2d.get())
            # The grid is already handled by the animation loop reading the checkbox var
            self.fig2d.canvas.draw_idle()
        except (ValueError, TclError):
            # Ignore errors from invalid/empty entry widgets during typing
            pass

    def update_3d_view_settings(self, *args):
        """Callback to apply 3D view settings from UI controls."""
        try:
            # Stop autoscaling to allow manual limits
            self.ax3d.autoscale(enable=False)
            self.ax3d.set_xlim(0, int(self.xlim_3d.get()))
            self.ax3d.set_ylim(0, int(self.ylim_3d.get()))
            self.ax3d.set_zlim(0, int(self.zlim_3d.get()))
            self.ax3d.set_xscale(self.scale_var_3d_x.get())
            self.ax3d.set_yscale(self.scale_var_3d_y.get())
            self.ax3d.set_zscale(self.scale_var_3d_z.get())
            self.ax3d.set_facecolor(self.color_var_3d.get())
            # The grid and view angle are handled by the animation loop
            self.fig3d.canvas.draw_idle()
        except (ValueError, TclError):
            # Ignore errors from invalid/empty entry widgets during typing
            pass

    def toggle_2d_autoscale(self, *args):
        """Enable or disable autoscaling on the 2D plot."""
        self.ax2d.autoscale(enable=True)
        self.ax2d.relim()
        self.ax2d.autoscale_view()
        self.fig2d.canvas.draw_idle()

    def toggle_3d_autoscale(self, *args):
        """Enable or disable autoscaling on the 3D plot."""
        self.ax3d.autoscale(enable=True)
        self.ax3d.relim()
        self.ax3d.autoscale_view()
        self.fig3d.canvas.draw_idle()
    
    def setup_mouse_handlers(self, canvas):
        """Set up mouse event handlers"""
        canvas.mpl_connect('button_press_event', self.on_mouse_press)
        canvas.mpl_connect('button_release_event', self.on_mouse_release)
        canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        canvas.mpl_connect('scroll_event', self.on_mouse_scroll)
        
    def on_mouse_press(self, event):
        """Handle mouse button press"""
        if event.button == 1:  # Left click
            self.mouse_drag_start = (event.xdata, event.ydata)
            
    def on_mouse_move(self, event):
        """Handle mouse movement"""
        if event.button == 1 and self.mouse_drag_start:
            dx = event.xdata - self.mouse_drag_start[0]
            dy = event.ydata - self.mouse_drag_start[1]
            self.view_offset[0] += dx
            self.view_offset[1] += dy
            self.mouse_drag_start = (event.xdata, event.ydata)
            self._update_2d_plot()
            
    def on_mouse_release(self, event):
        """Handle mouse button release"""
        if event.button == 1:  # Left click
            self.mouse_drag_start = None
            
    def on_mouse_scroll(self, event):
        """Handle mouse scroll for zooming"""
        if event.button == 'up':
            self.zoom_level *= 1.1
        elif event.button == 'down':
            self.zoom_level /= 1.1
        self._update_2d_plot()   
    
    def handle_target_click(self, event):
        """Handle Matplotlib mouse clicks on the 2D plot to set target coordinates."""
        # Check if the click was inside the plot axes
        if event.inaxes == self.ax2d:
            x = event.xdata
            y = event.ydata
            
            # Update the target in the tag_manager and log it
            self.tag_manager.set_target(x, y)
            self.log(f"Target set to: ({x:.1f}, {y:.1f})\n")
            
            # Update target entry fields if they exist
            if hasattr(self.root, 'target_x_entry') and hasattr(self.root, 'target_y_entry'):
                # This check is important because the navigation window might not be open
                try:
                    self.root.target_x_entry.delete(0, tk.END)
                    self.root.target_x_entry.insert(0, f"{x:.1f}")
                    self.root.target_y_entry.delete(0, tk.END)
                    self.root.target_y_entry.insert(0, f"{y:.1f}")
                except tk.TclError:
                    # This handles the case where the nav window has been closed
                    print("Navigation window not available to set target.")
                    
    # def update(self, anchors, tags):
    #     # 1. ALWAYS CLEAR FIRST
    #     self.ax2d.clear()
        
    #     # 2. THEN RE-APPLY LIMITS AND STYLES
    #     self.ax2d.set_xlim(-5, 5)   # meters
    #     self.ax2d.set_ylim(-5, 5)
    #     self.ax2d.set_facecolor(self.plot_ax_color)
    #     self.ax2d.grid(True, color=self.plot_grid_color, linestyle=':')
    #     self.ax2d.set_title("Live Position", color="white")

    #     # 3. Draw Anchors
    #     for anchor in anchors:
    #         # Note: ensure anchors is a list or use anchors.values() if it's a dict
    #         self.ax2d.plot([anchor.x], [anchor.y], 's', color='red', markersize=10)

    #     # 4. Draw Tags and Trails
    #     for tag in tags: # Use tags.values() if tags is a dictionary!
            
    #         # Plot the Trail
    #         if tag.movement_history and len(tag.movement_history) > 1:
    #             trail_x = [h[0] for h in tag.movement_history]
    #             trail_y = [h[1] for h in tag.movement_history]
                
    #             self.ax2d.plot(trail_x, trail_y, '-', color='#00ff00', alpha=0.6, linewidth=2.0)

    #         # Plot the Current Position Dot
    #         self.ax2d.plot(tag.x, tag.y, 'o', color='#00ff00', markersize=12, label=f'Tag {tag.id}')