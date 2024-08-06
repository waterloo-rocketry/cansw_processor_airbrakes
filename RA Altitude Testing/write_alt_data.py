import time
import serial

port = "COM22"
ser = serial.Serial(port, 115200, timeout=0.1)  # Increased timeout for stability

alt = []
timestamps = []

# Read and convert altitude data with timestamps
with open("alt_msg_extract.txt") as f:
    for line in f:
        try:
            l = line.strip().split(',')
            number = abs(int(float(l[1]) * 3.281))
            alt.append(number)
            timestamps.append(abs(float(l[0])))

        except ValueError:
            print(f"Skipping invalid line: {line.strip()}")
print(f"Loaded {len(alt)} altitude values.")

timestamps = [i - timestamps[0] for i in timestamps] #timestamp offset

start_time = time.time()
for i in range(len(alt)):
    current_time = time.time()
    elapsed_time = current_time - start_time
    sleep_time = timestamps[i] - elapsed_time

    while (time.time() - current_time) < sleep_time:
        pass

    #format CAN msg
    hex_value = f"{alt[i]:08x}"  # 4 bytes = 8 hex digits
    t = f"{int(timestamps[i]*1000):08x}"  # 4 bytes = 8 hex digits
    msg = f"m571,{t[2:4]},{t[4:6]},{t[6:8]},{hex_value[0:2]},{hex_value[2:4]},{hex_value[4:6]},{hex_value[6:8]};"
    
    try:
        ser.close()
        ser = serial.Serial(port, 115200, timeout=0.1)  # Increased timeout for stability
        ser.write(bytes(msg, 'ascii'))
        ser.flush()
        ser.reset_output_buffer()
        print(f"Sent message: {msg}")
    except KeyboardInterrupt:
        ser.close()
        exit()

ser.close()
