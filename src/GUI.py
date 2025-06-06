import tkinter as tk
from tkinter import ttk, messagebox
import asyncio
import threading
import struct
from bleak import BleakClient, BleakScanner
import logging

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write to device
NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Read from device

class NordicBLEGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Nordic BLE Control - Chronos")
        self.root.geometry("400x500")
        
        self.client = None
        self.connected = False
        self.device_address = None
        
        # Create asyncio event loop for the thread
        self.loop = None
        self.ble_thread = None
        
        self.setup_gui()
        self.start_ble_thread()
    
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Connection section
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=0, column=0, sticky=tk.W)
        
        self.scan_button = ttk.Button(conn_frame, text="Scan & Connect", command=self.scan_and_connect)
        self.scan_button.grid(row=0, column=1, sticky=tk.E)
        
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_button.grid(row=0, column=2, sticky=tk.E, padx=(5, 0))
        
        # Configure column weights
        conn_frame.columnconfigure(0, weight=1)
        
        # Parameters section
        params_frame = ttk.LabelFrame(main_frame, text="Stimulation Parameters", padding="5")
        params_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # DAC Amplitude
        ttk.Label(params_frame, text="DAC Amplitude:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.dac_var = tk.StringVar(value="1000")
        self.dac_entry = ttk.Entry(params_frame, textvariable=self.dac_var, width=10)
        self.dac_entry.grid(row=0, column=1, sticky=tk.W, padx=(5, 0))
        ttk.Label(params_frame, text="(0-65535)").grid(row=0, column=2, sticky=tk.W, padx=(5, 0))
        
        # Pulse Width
        ttk.Label(params_frame, text="Pulse Width:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.pulse_var = tk.StringVar(value="500")
        self.pulse_entry = ttk.Entry(params_frame, textvariable=self.pulse_var, width=10)
        self.pulse_entry.grid(row=1, column=1, sticky=tk.W, padx=(5, 0))
        ttk.Label(params_frame, text="μs (0-65535)").grid(row=1, column=2, sticky=tk.W, padx=(5, 0))
        
        # Frequency
        ttk.Label(params_frame, text="Frequency:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.freq_var = tk.StringVar(value="100")
        self.freq_entry = ttk.Entry(params_frame, textvariable=self.freq_var, width=10)
        self.freq_entry.grid(row=2, column=1, sticky=tk.W, padx=(5, 0))
        ttk.Label(params_frame, text="Hz (0-65535)").grid(row=2, column=2, sticky=tk.W, padx=(5, 0))
        
        # Send button
        self.send_button = ttk.Button(params_frame, text="Send Parameters", 
                                     command=self.send_parameters, state="disabled")
        self.send_button.grid(row=3, column=0, columnspan=3, pady=(10, 0))
        
        # Log section
        log_frame = ttk.LabelFrame(main_frame, text="Log", padding="5")
        log_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # Text widget with scrollbar
        text_frame = ttk.Frame(log_frame)
        text_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.log_text = tk.Text(text_frame, height=15, width=50)
        scrollbar = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Configure main frame weights
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(2, weight=1)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
    
    def log_message(self, message):
        """Add message to log with timestamp"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        # Schedule on main thread
        self.root.after(0, lambda: self._append_to_log(log_entry))
    
    def _append_to_log(self, text):
        """Append text to log (must be called from main thread)"""
        self.log_text.insert(tk.END, text)
        self.log_text.see(tk.END)
    
    def start_ble_thread(self):
        """Start the BLE thread with its own event loop"""
        def run_ble_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        
        self.ble_thread = threading.Thread(target=run_ble_loop, daemon=True)
        self.ble_thread.start()
    
    def run_coroutine(self, coro):
        """Run a coroutine in the BLE thread"""
        if self.loop:
            future = asyncio.run_coroutine_threadsafe(coro, self.loop)
            return future
        return None
    
    def scan_and_connect(self):
        """Scan for Chronos device and connect"""
        self.scan_button.config(state="disabled")
        self.log_message("Scanning for Chronos device...")
        
        future = self.run_coroutine(self._scan_and_connect())
        if future:
            # Check result in a separate thread to avoid blocking GUI
            threading.Thread(target=self._check_connection_result, args=(future,), daemon=True).start()
    
    async def _scan_and_connect(self):
        """Async scan and connect"""
        try:
            devices = await BleakScanner.discover(timeout=10.0)
            chronos_device = None
            
            for device in devices:
                if device.name and "Chronos" in device.name:
                    chronos_device = device
                    break
            
            if not chronos_device:
                self.root.after(0, lambda: self.log_message("Chronos device not found"))
                self.root.after(0, lambda: self.scan_button.config(state="normal"))
                return False
            
            self.device_address = chronos_device.address
            self.root.after(0, lambda: self.log_message(f"Found Chronos: {chronos_device.address}"))
            
            # Connect to device
            self.client = BleakClient(chronos_device.address)
            await self.client.connect()
            
            if self.client.is_connected:
                self.connected = True
                self.root.after(0, self._update_connection_status)
                self.root.after(0, lambda: self.log_message("Connected successfully!"))
                return True
            else:
                self.root.after(0, lambda: self.log_message("Failed to connect"))
                self.root.after(0, lambda: self.scan_button.config(state="normal"))
                return False
                
        except Exception as e:
            self.root.after(0, lambda: self.log_message(f"Connection error: {str(e)}"))
            self.root.after(0, lambda: self.scan_button.config(state="normal"))
            return False
    
    def _check_connection_result(self, future):
        """Check the connection result (runs in separate thread)"""
        try:
            result = future.result(timeout=15.0)  # Wait up to 15 seconds
        except Exception as e:
            self.root.after(0, lambda: self.log_message(f"Connection timeout: {str(e)}"))
            self.root.after(0, lambda: self.scan_button.config(state="normal"))
    
    def _update_connection_status(self):
        """Update GUI connection status (called from main thread)"""
        if self.connected:
            self.status_label.config(text="Status: Connected", foreground="green")
            self.scan_button.config(state="disabled")
            self.disconnect_button.config(state="normal")
            self.send_button.config(state="normal")
        else:
            self.status_label.config(text="Status: Disconnected", foreground="red")
            self.scan_button.config(state="normal")
            self.disconnect_button.config(state="disabled")
            self.send_button.config(state="disabled")
    
    def disconnect(self):
        """Disconnect from device"""
        if self.client and self.connected:
            future = self.run_coroutine(self._disconnect())
            threading.Thread(target=self._check_disconnect_result, args=(future,), daemon=True).start()
    
    async def _disconnect(self):
        """Async disconnect"""
        try:
            await self.client.disconnect()
            self.connected = False
            self.client = None
            self.root.after(0, self._update_connection_status)
            self.root.after(0, lambda: self.log_message("Disconnected"))
        except Exception as e:
            self.root.after(0, lambda: self.log_message(f"Disconnect error: {str(e)}"))
    
    def _check_disconnect_result(self, future):
        """Check disconnect result"""
        try:
            future.result(timeout=5.0)
        except Exception as e:
            self.root.after(0, lambda: self.log_message(f"Disconnect timeout: {str(e)}"))
    
    def send_parameters(self):
        """Send stimulation parameters to device"""
        try:
            # Validate inputs
            dac_amp = int(self.dac_var.get())
            pulse_width = int(self.pulse_var.get())
            frequency = int(self.freq_var.get())
            
            if not (0 <= dac_amp <= 65535):
                raise ValueError("DAC amplitude must be 0-65535")
            if not (0 <= pulse_width <= 65535):
                raise ValueError("Pulse width must be 0-65535")
            if not (0 <= frequency <= 65535):
                raise ValueError("Frequency must be 0-65535")
            
            # Pack data as little-endian uint16 values (matching C struct)
            data = struct.pack('<HHH', dac_amp, pulse_width, frequency)
            
            self.log_message(f"Sending: DAC={dac_amp}μA, Pulse={pulse_width}μs, Freq={frequency}Hz")
            
            # Send data
            future = self.run_coroutine(self._send_data(data))
            threading.Thread(target=self._check_send_result, args=(future,), daemon=True).start()
            
        except ValueError as e:
            messagebox.showerror("Input Error", str(e))
            self.log_message(f"Input error: {str(e)}")
        except Exception as e:
            self.log_message(f"Send error: {str(e)}")
    
    async def _send_data(self, data):
        """Async send data"""
        try:
            if self.client and self.connected:
                await self.client.write_gatt_char(NUS_RX_CHAR_UUID, data)
                self.root.after(0, lambda: self.log_message("Data sent successfully"))
                return True
            else:
                self.root.after(0, lambda: self.log_message("Not connected to device"))
                return False
        except Exception as e:
            self.root.after(0, lambda: self.log_message(f"Send failed: {str(e)}"))
            return False
    
    def _check_send_result(self, future):
        """Check send result"""
        try:
            future.result(timeout=5.0)
        except Exception as e:
            self.root.after(0, lambda: self.log_message(f"Send timeout: {str(e)}"))
    
    def on_closing(self):
        """Handle window closing"""
        if self.connected:
            self.disconnect()
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        self.root.destroy()

def main():
    root = tk.Tk()
    app = NordicBLEGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()