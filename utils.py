# utils.py

import tkinter as tk

def add_tooltip(widget, text):
    """
    Creates a tooltip for a given widget.
    """
    tooltip = tk.Toplevel(widget)
    tooltip.withdraw()
    tooltip.overrideredirect(True)

    label = tk.Label(tooltip, text=text, justify='left',
                     background="#ffffe0", relief='solid', borderwidth=1,
                     font=("tahoma", "8", "normal"))
    label.pack(ipadx=1)

    def enter(event=None):
        x = y = 0
        x, y, cx, cy = widget.bbox("insert")
        x += widget.winfo_rootx() + 25
        y += widget.winfo_rooty() + 20
        tooltip.wm_geometry(f"+{x}+{y}")
        tooltip.deiconify()

    def leave(event=None):
        tooltip.withdraw()

    widget.bind("<Enter>", enter)
    widget.bind("<Leave>", leave)

def is_admin():
    """Checks for administrator privileges on Windows."""
    try:
        import ctypes
        return ctypes.windll.shell32.IsUserAnAdmin()
    except Exception:
        return False