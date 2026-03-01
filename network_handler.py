# network_handler.py
import socket
import threading
import json
import os
import datetime as dt
from config import TCP_HOST, TCP_PORT

class NetworkHandler:
    def __init__(self, tag_manager, log_callback):
        self.server_socket = None
        self.client_socket = None
        self.is_running = False
        self.tag_manager = tag_manager
        self.log = log_callback # A function to log messages to the GUI
        self.file_buffer = bytearray()
        self.is_receiving_file = False

    def start_server(self):
        """Starts the TCP server in a background thread."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((TCP_HOST, TCP_PORT))
            self.server_socket.listen(1)
            self.log(f"Network server listening on {TCP_HOST}:{TCP_PORT}")
        except Exception as e:
            self.log(f"ERROR: Could not start network server: {e}")
            return

        self.is_running = True
        thread = threading.Thread(target=self._accept_connections, daemon=True)
        thread.start()

    def _accept_connections(self):
        """Loop to accept incoming client connections."""
        while self.is_running:
            try:
                self.server_socket.settimeout(1)
                conn, addr = self.server_socket.accept()
                self.log(f"Connected to client at {addr}")
                self.client_socket = conn
                # Handle this one client
                self.handle_client(conn)
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    self.log(f"Server accept error: {e}")
                break

    def handle_client(self, conn):
        """
        Runs in a background thread. Handles BOTH JSON updates AND File transfers.
        """
        conn.settimeout(None) # Blocking mode
        
        while self.is_running:
            try:
                # Read raw bytes
                data = conn.recv(4096)
                if not data: break
                
                # --- STATE 1: CURRENTLY RECEIVING A FILE ---
                if self.is_receiving_file:
                    self.file_buffer.extend(data)
                    
                    # Check for the end marker
                    if b"FILE_END" in self.file_buffer:
                        self.log("INFO: File transfer complete. Saving...")
                        
                        # Strip the footer
                        clean_data = self.file_buffer.replace(b"FILE_END", b"")
                        
                        # Save to Desktop
                        self._save_file_to_data_folder(clean_data)
                        
                        # Reset State
                        self.file_buffer = bytearray()
                        self.is_receiving_file = False
                        
                    continue # Skip JSON parsing while downloading file

                # --- STATE 2: WAITING FOR DATA ---
                # Check if this is the start of a file
                if b"FILE_START" in data:
                    self.log("INFO: Incoming CSV File detected...")
                    self.is_receiving_file = True
                    
                    # Remove the header and start buffering
                    # Note: The chunk might contain 'FILE_START' + some data
                    start_index = data.find(b"FILE_START") + len(b"FILE_START")
                    self.file_buffer.extend(data[start_index:])
                    
                    # Corner case: File is tiny and start/end are in same chunk
                    if b"FILE_END" in self.file_buffer:
                         clean_data = self.file_buffer.replace(b"FILE_END", b"")
                         self._save_file_to_desktop(clean_data)
                         self.file_buffer = bytearray()
                         self.is_receiving_file = False
                    
                    continue

                # --- STATE 3: NORMAL JSON DATA ---
                # If we are here, it's just regular tag data
                try:
                    text_data = data.decode('utf-8')
                    # (Your existing JSON handling logic goes here)
                    # payload = json.loads(text_data) ...
                    # self.tag_manager.update_tag(...)
                except:
                    # It wasn't a file, and it wasn't valid JSON. Ignore.
                    pass

            except Exception as e:
                self.log(f"Connection Error: {e}")
                break

    def _save_file_to_data_folder(self, data_bytes):
        try:
            # 1. Get the directory where THIS script (network_handler.py) is located
            # This makes it work on G:, E:, C:, or anywhere else.
            base_dir = os.path.dirname(os.path.abspath(__file__))
            
            # 2. Construct the path to the 'data' folder
            # It assumes 'data' is in the same folder as your python scripts
            data_folder = os.path.join(base_dir, "data")
            
            # 3. Create the folder if it doesn't exist (Safety check)
            if not os.path.exists(data_folder):
                os.makedirs(data_folder)
                self.log(f"Created new data directory: {data_folder}")

            # 4. Generate Filename
            timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(data_folder, f"lidar_ground_truth_{timestamp}.csv")
            
            # 5. Save
            text_content = data_bytes.decode('utf-8', errors='ignore')
            
            with open(filename, "w", newline='') as f:
                f.write(text_content)
                
            self.log(f"SUCCESS: Saved file to: {filename}")
            
        except Exception as e:
            self.log(f"ERROR Saving file: {e}")

    def send_command(self, message_json, tag_id):
        """
        Sends a JSON message to the connected client.
        The robot/tag on the other end is responsible for routing based on tag_id.
        """
        if self.client_socket:
            try:
                # We embed the tag_id in the JSON payload
                payload = json.loads(message_json)
                payload['tag_id'] = tag_id
                
                final_message = json.dumps(payload)
                self.client_socket.sendall(final_message.encode('utf-8'))
                self.log(f"Sent to client: {final_message}")
                return True
            except Exception as e:
                self.log(f"ERROR: Failed to send command: {e}")
                return False
        self.log("WARN: Cannot send command, no client connected.")
        return False
    
    def send_data_to_tag(self, tag_id: str, data_payload: bytes) -> bool:
       """
       Constructs and sends a specific AT command to transmit raw data to a tag.
       The format is based on the DWM_receiver.py implementation.
       """
       if not self.client_socket:
           self.log("WARN: Cannot send data, no client connected.")
           return False
           
       try:
           hex_data_string = data_payload.hex()
           command_string = f'AT+DataSend="{hex_data_string}","{tag_id}"\n'
           
           self.client_socket.sendall(command_string.encode('ascii'))
           self.log(f"Sent AT command for tag '{tag_id}' with {len(data_payload)} bytes.")
           return True
       except Exception as e:
           self.log(f"ERROR: Failed to send data via AT command: {e}")
           return False
       
    def stop_server(self):
        """Stops the network server and closes connections."""
        self.is_running = False
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
                self.client_socket.close()
            except OSError:
                pass # Ignore errors if socket is already closed
        if self.server_socket:
            self.server_socket.close()
        self.log("INFO: Network server stopped.")
        
    def request_lidar_csv(self):
        """Sends command to robot and handles the response robustly."""
        if not self.client_socket:
            self.log("ERROR: No robot connected.")
            return None
            
        try:
            # 1. Send Command
            self.log("Sending PROCESS_DATA command to robot...")
            self.client_socket.sendall("PROCESS_DATA".encode('utf-8'))
            
            # 2. Receive Response (With Timeout safety)
            self.client_socket.settimeout(15) # Wait max 15 seconds for processing
            buffer = ""
            
            try:
                while True:
                    chunk = self.client_socket.recv(4096).decode('utf-8')
                    if not chunk: break
                    
                    buffer += chunk
                    if "FILE_END" in buffer:
                        break
            except socket.timeout:
                self.log("ERROR: Timed out waiting for robot response.")
                return None
            finally:
                self.client_socket.settimeout(None) # Reset timeout
            
            # 3. Process the Result
            if "FILE_START" in buffer:
                # Success! Clean the markers
                clean_csv = buffer.replace("FILE_START", "").replace("FILE_END", "")
                return clean_csv
            elif "ERROR" in buffer:
                # Robot sent an error message
                error_msg = buffer.replace("FILE_END", "")
                self.log(f"ROBOT ERROR: {error_msg}")
                return None
            else:
                self.log(f"Unknown response: {buffer[:50]}...")
                return None
                
        except Exception as e:
            self.log(f"Network Transfer Error: {e}")
            return None
