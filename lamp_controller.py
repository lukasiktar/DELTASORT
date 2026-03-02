import serial
import time

class GE483Controller:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        Initialize connection to GE483 multispectral LED bar
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' on Linux, 'COM3' on Windows)
            baudrate: Communication baudrate (fixed at 115200)
        """
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.master_addr = 0x00
        self.slave_addr = 0x10  # Default slave address for first module
        time.sleep(0.5)  # Wait for connection to stabilize
    
    def calculate_crc(self, bytes_list):
        """Calculate XOR CRC for all bytes except HEAD (first byte)"""
        return bytes([sum(bytes_list) & 0xFF])
    
    def send_command(self, command_bytes):
        """Send command and wait for response"""
        self.ser.write(command_bytes)
        time.sleep(0.1)  # Give device time to respond
        response = self.ser.read(8)  # RS485 messages are 8 bytes
        return response
    
    def set_led_intensity(self, channel, intensity):
        """
        Set LED channel intensity
        
        Args:
            channel: LED wavelength group (1-12)
            intensity: Brightness level (0-100%)
        """
        # LED DIMMING command: 0x02
        head = 0x80
        dest = self.slave_addr
        mitt = self.master_addr
        cmd = 0x02
        grp = channel
        dim = intensity
        b3 = 0x00  # Don't care
        
        # Calculate CRC (XOR of all bytes except HEAD)
        crc = dest ^ cmd ^ grp ^ dim ^ b3
        
        # Build command message
        message = bytes([head, dest, mitt, cmd, grp, dim, b3, crc])
        
        #print(f"Sending: {' '.join([f'0x{b:02x}' for b in message])}")
        response = self.send_command(message)
        #print(f"Response: {' '.join([f'0x{b:02x}' for b in response])}")
        
        return response
    
    def turn_on_sunlike_100percent(self):
        """Turn on the SUNLIKE LED (400-700nm) to 100% intensity"""
        print("Turning on SUNLIKE (400-700nm) LED to 100%...")
        return self.set_led_intensity(channel=1, intensity=100)
    
    def close(self):
        """Close serial connection"""
        self.ser.close()

    def read_temp(self):
        head = 0x80
        dest = self.slave_addr
        mitt = self.master_addr
        cmd = 0x08
        grp = 0x00
        dim = 0x00 
        b3 = 0x00  # Don't care
        
        # Calculate CRC (XOR of all bytes except HEAD)
        crc_bytes = [dest, mitt, cmd, grp, dim, b3]
        crc = sum(crc_bytes) & 0xFF
        
        # Build command message
        message = bytes([head, dest, mitt, cmd, grp, dim, b3, crc])
        
        #print(f"Sending: {' '.join([f'0x{b:02x}' for b in message])}")
        response = self.send_command(message)
        #print(f"Response: {' '.join([f'0x{b:02x}' for b in response])}")
        htemp = response[4]
        ltemp = response[5]
        temp = (htemp*256+ltemp)/10
        return temp 

    def turn_off_all(self):
        for i in range(1, 13):
            self.set_led_intensity(i, 0)

    def turn_on_range(self, id_range, intensity=100):
        for i in id_range:
            self.set_led_intensity(i, intensity)



if __name__ == "__main__":
    # Create controller instance
    # Adjust port based on your system:
    # - Linux: '/dev/ttyUSB0' or '/dev/ttyACM0'
    # - Windows: 'COM3' (or appropriate COM port)
    # - macOS: '/dev/tty.usbserial-*'
    
    controller = GE483Controller(port='/dev/ttyUSB0')
    
    try:
        # Turn on SUNLIKE (400-700nm) to 100%
        #print("TURNING ALL OFF")
        #controller.turn_off_all()
        #time.sleep(1.0)
        #for i in range(2, 13):
            #print("TURNING ON CHANNEL", i)
            #controller.set_led_intensity(i, 100)
            #time.sleep(2.0)
            #controller.set_led_intensity(i, 0)
        #controller.turn_on_sunlike_100percent()
        #controller.set_led_intensity(0, 100)
        #controller.turn_on_range(range(2, 13), 100)
        controller.turn_off_all()
        #print("T:", controller.read_temp(), "C")
        #controller.test()
        time.sleep(0.5)
        
        # You can also control other channels:
        # controller.set_led_intensity(channel=2, intensity=50)  # 730nm at 50%
        
    finally:
        controller.close()
        print("Connection closed")
