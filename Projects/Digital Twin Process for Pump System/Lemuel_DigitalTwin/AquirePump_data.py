import time
import nidaqmx
import numpy
import matplotlib.pyplot as plt
import os

# Real-time data acquisition and plotting

channels = 3
channel_names = ["RPM", "Pressure Diff", "Flow"]
ai_channel_list = ["Dev1/ai2", "Dev1/ai4", "Dev1/ai5", "Dev1/ai0"]

plt.ion()
fig, axs = plt.subplots(channels, 1, figsize=(8, 8), sharex=True)
lines = []
data_buffer = [[], [], []]
timestamps = []

# Prepare output log file
log_filename = "output_log.csv"
log_filepath = os.path.join(os.path.dirname(__file__), log_filename)
log_file = open(log_filepath, "w")
log_file.write("Time(s),RPM,PressureDiff,Flow\n")

for i, ax in enumerate(axs):
    line, = ax.plot([], [], label=channel_names[i])
    lines.append(line)
    ax.set_ylabel(channel_names[i])
    ax.legend(loc="upper right")
axs[-1].set_xlabel("Time (s)")

with nidaqmx.Task() as task:
    for ch in ai_channel_list:
        task.ai_channels.add_ai_voltage_chan(
            ch,
            terminal_config=nidaqmx.constants.TerminalConfiguration.DEFAULT,
            min_val=-10.0,
            max_val=10.0,
            units=nidaqmx.constants.VoltageUnits.VOLTS
        )

    start_time = time.time()
    try:
        while True:
            raw = numpy.array(task.read(timeout=1.0))
            # Data conversion
            rpm = 500 * (raw[0] - 0.06)
            pressure_in = 0.417 * (raw[1] + 1.02)
            pressure_out = 0.165 * raw[2]
            pressure_diff = pressure_out - pressure_in
            flow = 15.156 * (raw[3] - 0.16)
            t = time.time() - start_time

            # Append to buffers
            data_buffer[0].append(rpm)
            data_buffer[1].append(pressure_diff)
            data_buffer[2].append(flow)
            timestamps.append(t)

            # Write to log file
            log_file.write(f"{t:.3f},{rpm:.3f},{pressure_diff:.3f},{flow:.3f}\n")
            log_file.flush()

            # Update plots
            for idx, line in enumerate(lines):
                line.set_xdata(timestamps)
                line.set_ydata(data_buffer[idx])
                axs[idx].relim()
                axs[idx].autoscale_view()
            plt.pause(0.01)
    except KeyboardInterrupt:
        print("Real-time plotting stopped.")
        log_file.close()

