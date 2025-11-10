import serial

# Change 'COM3' to your ESP32's actual port (Check Device Manager or use 'ls /dev/ttyUSB*' on Linux/Mac)
ESP32_PORT = "COM7"  # Windows Example
# ESP32_PORT = "/dev/ttyUSB0"  # Linux/macOS Example

BAUD_RATE = 115200  # Match the ESP32's baud rate

try:
    ser = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {ESP32_PORT} at {BAUD_RATE} baud.")
except serial.SerialException:
    print(f"❌ Error: Could not open serial port {ESP32_PORT}. Check your connection!")
    exit()

# File to store fingerprint data permanently
FILENAME = "fingerprints.txt"

with open(FILENAME, "a") as file:  # Open in append mode
    print("\n✅ Listening for fingerprint data from ESP32...")

    while True:
        try:
            data = ser.readline().decode().strip()  # Read and clean data from ESP32

            if data:  # If data is received
                print(f"📌 Received: {data}")

                if data.startswith("[SAVE]"):  # Expected format from ESP32
                    fingerprint_id = data.replace("[SAVE] ", "").strip()
                    print(f"💾 Saving fingerprint: {fingerprint_id}")

                    # Write to file
                    file.write(fingerprint_id + "\n")
                    file.flush()  # Ensure data is saved immediately

                    print(f"✅ Saved to {FILENAME}\n")

        except KeyboardInterrupt:  # Stop script with CTRL+C
            print("\n❌ Script stopped by user.")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            break

print("🔄 Closing serial connection...")
ser.close()
