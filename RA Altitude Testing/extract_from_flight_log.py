import matplotlib.pyplot as plt
from scipy.signal import lfilter

readings = []
with open("flight_logs_parsed.txt") as f:
    for line in f:
        l = line.split()
        if(l[1] == "SENSOR_ALTITUDE"):
            readings.append([float(l[5]), float(l[7])/3.281])

with open("alt_msg_extract.txt", 'w') as f:
    for line in readings[45000:45000+1500]:
        f.write(f"{line[0]},{int(round(line[1], 0))}\n")



velocity = []
t = []
last_alt = 0
last_time = 0
threshold = 30
count = 0
for message in readings:
    v = round((message[1] - last_alt) / (message[0] - last_time), 2)
    if v < threshold:
        count += 1
    else:
        count = 0
    t.append(count)
    velocity.append(v)
    last_time = message[0]
    last_alt = message[1]

# Define filter coefficients for 1st and 2nd order IIR lowpass filters
# Assuming a normalized cutoff frequency (0 < cutoff < 1)
cutoff = 0.1  # Example cutoff frequency, adjust as needed
b1, a1 = [cutoff], [1, cutoff - 1]  # 1st order lowpass filter coefficients
b2, a2 = [cutoff**2], [1, -2*(1-cutoff), (1-cutoff)**2]  # 2nd order lowpass filter coefficients

# Apply filters to the velocity data
velocity_filtered_1st = lfilter(b1, a1, velocity)
velocity_filtered_2nd = lfilter(b2, a2, velocity)


t2 = []
count2 = 0
for v in velocity_filtered_2nd:
    if v < threshold:
        count2 += 1
    else:
        count2 = 0
    t2.append(count2)
#print(t2)

plt.figure(figsize=(12, 6))

plt.plot(velocity[40000:], label='Original Velocity')
plt.plot([int(entry > 10)*500 for entry in t2[40000:]] , label='Count')
plt.plot(velocity_filtered_1st[40000:], label='1st Order Lowpass Filtered Velocity')
plt.plot(velocity_filtered_2nd[40000:], label='2nd Order Lowpass Filtered Velocity')

plt.xlabel('Sample Number')
plt.ylabel('Values')
plt.title('Velocity, Count, and Filtered Velocity Values')
plt.legend()

plt.show()
