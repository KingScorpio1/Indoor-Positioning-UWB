# navigation.py

import json

class EnhancedNavigationController:
    """
    Handles the logic for creating and sending navigation commands to the tag.
    It uses a network_handler to send the actual data.
    """
    def __init__(self, tag_manager, network_handler):
        self.tag_manager = tag_manager
        self.network_handler = network_handler
        self.navigation_state = "idle"  # idle, manual, auto
        self.current_speed_cm_s = 50
        self.movement_duration_ms = 200

    def set_target(self, tag_id, x, y):
        """Sets a target for autonomous navigation."""
        self.navigation_state = "auto"
        # The logic to make the tag move to (x,y) would be implemented on the tag/robot itself.
        # This command just tells the tag what its new target is.
        command = {
            'type': 'nav_command',
            'command': 'navigate_to',
            'params': {'x': x, 'y': y}
        }
        return self.network_handler.send_command(json.dumps(command), tag_id)

    def send_movement_command(self, tag_id, direction_code):
        """Sends a direct, manual movement command."""
        # Mapping from the GUI's short codes to a more descriptive format for the robot
        direction_map = {
            'FL': "forward_left", 'FS': "forward", 'FR': "forward_right",
            'BL': "backward_left", 'BS': "backward", 'BR': "backward_right"
        }
        
        if direction_code in direction_map:
            command = {
                'type': 'nav_command',
                'command': 'move',
                'params': {
                    'direction': direction_map[direction_code],
                    'duration_ms': self.movement_duration_ms,
                    'speed_cm_s': self.current_speed_cm_s
                }
            }
            return self.network_handler.send_command(json.dumps(command), tag_id)
        return False

    def stop_movement(self, tag_id):
        """Sends an immediate stop command."""
        command = {'type': 'nav_command', 'command': 'stop'}
        return self.network_handler.send_command(json.dumps(command), tag_id)

    def set_speed(self, tag_id, speed_level):
        """Sets the movement speed."""
        speed_map = {'1': 20, '2': 40, '3': 60, '4': 80, '5': 100} # Speed in cm/s
        self.current_speed_cm_s = speed_map.get(str(speed_level), 60)
        
        command = {
            'type': 'nav_command',
            'command': 'set_speed',
            'params': {'speed_cm_s': self.current_speed_cm_s}
        }
        return self.network_handler.send_command(json.dumps(command), tag_id)

    def set_movement_duration(self, duration_ms):
        """Updates the duration for manual movement commands."""
        try:
            self.movement_duration_ms = int(duration_ms)
            return True
        except ValueError:
            return False
        
    def change_work_state(self, tag_id: str):
        """Sends a command to toggle the tag's work state (e.g., active/standby)."""
        self.log(f"Sending 'Change Work State' command to tag {tag_id}.")
        command = {
            'type': 'nav_command',
            'command': 'toggle_work_state'
        }
        return self.network_handler.send_command(json.dumps(command), tag_id)

    def send_config(self, tag_id: str, config_params: dict):
        """Sends a dictionary of configuration parameters to the tag."""
        self.log(f"Sending configuration to tag {tag_id}: {config_params}")
        command = {
            'type': 'nav_command',
            'command': 'set_config',
            'params': config_params
        }
        return self.network_handler.send_command(json.dumps(command), tag_id)