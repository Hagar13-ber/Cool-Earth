import smbus2
import time

# I2C Address for AltIMU-10 v4 Gyroscope (L3GD20H)
L3GD20H_ADDR = 0x6B

# The I2C bus is typically 1 on all modern Raspberry Pi models
bus = smbus2.SMBus(1)

# --- Register Definitions ---
L3G_CTRL1 = 0x20
L3G_OUT_X_L = 0x28

def init_gyro():
    """Initializes the Gyroscope by waking it up and enabling axes."""
    # 0x0F (00001111 in binary): 
    # Bits 3-0: Enable X, Y, Z axes, and set to Normal Mode (Power on)
    # Bits 7-4: Output Data Rate (ODR) and Bandwidth (Default 95Hz)
    bus.write_byte_data(L3GD20H_ADDR, L3G_CTRL1, 0x0F)

def read_raw_axis(addr, reg):
    """Reads a 16-bit value from two consecutive 8-bit registers."""
    low = bus.read_byte_data(addr, reg)
    high = bus.read_byte_data(addr, reg + 1)
    
    value = (high << 8) | low
    
    # Convert to signed 16-bit integer (Two's complement)
    if value > 32767:
        value -= 65536
    return value

def read_gyro(addr, start_reg):
    """Reads X, Y, and Z axes starting from the given register."""
    x = read_raw_axis(addr, start_reg)
    y = read_raw_axis(addr, start_reg + 2)
    z = read_raw_axis(addr, start_reg + 4)
    return x, y, z

if __name__ == "__main__":
    try:
        init_gyro()
        print("Successfully initialized L3GD20H Gyroscope.")
        print("Reading rotational data... (Press Ctrl+C to stop)\n")
        
        # Real-time data loop
        while True:
            gx, gy, gz = read_gyro(L3GD20H_ADDR, L3G_OUT_X_L)
            
            print(f"Gyro [Raw]: X: {gx:6d} | Y: {gy:6d} | Z: {gz:6d}")
            
            # 0.1s delay for a ~10Hz read rate. 
            time.sleep(0.1)  
            
    except OSError as e:
        print(f"\nI2C Error: {e}")
        print("Ensure the IMU is wired correctly and the address (0x6B) is detected via 'i2cdetect -y 1'.")
    except KeyboardInterrupt:
        print("\nData collection stopped cleanly.")