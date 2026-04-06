import time
import nidaqmx
import numpy  # using numpy instead of np
import boto3  # AWS SDK for Python

# samples
samples = 100  # example number of samples
channels = 4    # example number of channels (original sensor channels)

data = numpy.zeros((samples, channels))

with nidaqmx.Task() as task:
    task.ai_channels.add_ai_voltage_chan("Dev1/ai2", terminal_config=nidaqmx.constants.TerminalConfiguration.DEFAULT, min_val=-10.0, max_val=10.0, units=nidaqmx.constants.VoltageUnits.VOLTS)
    task.ai_channels.add_ai_voltage_chan("Dev1/ai4", terminal_config=nidaqmx.constants.TerminalConfiguration.DEFAULT, min_val=-10.0, max_val=10.0, units=nidaqmx.constants.VoltageUnits.VOLTS)
    task.ai_channels.add_ai_voltage_chan("Dev1/ai5", terminal_config=nidaqmx.constants.TerminalConfiguration.DEFAULT, min_val=-10.0, max_val=10.0, units=nidaqmx.constants.VoltageUnits.VOLTS)
    task.ai_channels.add_ai_voltage_chan("Dev1/ai0", terminal_config=nidaqmx.constants.TerminalConfiguration.DEFAULT, min_val=-10.0, max_val=10.0, units=nidaqmx.constants.VoltageUnits.VOLTS)

    for i in range(samples):
        data[i, :] = task.read(timeout=nidaqmx.constants.WAIT_INFINITELY)

# RPM data
data[:, 0] = 500 * (data[:, 0] - 0.06)

# pressure in, pressure out (bar)
data[:, 1] = 0.417 * (data[:, 1] + 1.02)
data[:, 2] = 0.165 * data[:, 2]

# flow l/min
data[:, 3] = 15.156 * (data[:, 3] - 0.16)

# Add timestamp column (seconds since epoch)
start_time = time.time()
timestamps = numpy.array([start_time + i for i in range(samples)])  # assuming 1 sec intervals between samples

# Combine timestamp column with data
data_with_time = numpy.column_stack((timestamps, data))

# Save to CSV with timestamp column header
file_name = f"data_{int(start_time)}.csv"
header = "Timestamp,RPM,Pressure In,Pressure Out,Flow"

numpy.savetxt(file_name, data_with_time, delimiter = ",", header = header, comments ='')

# Upload to S3, # Initialize S3 client with your credentials
s3 = boto3.client(
    's3',
    aws_access_key_id="AKIAZZZS2DNDHROGADJZ",           # AWS access key
    aws_secret_access_key="JApEs0Q4ckgJ6uaUlPvO4a8q3xu1yB8D+gLU7O9x"  # AWS secret key
)

bucket_name = "bbdgtwin"  # S3 bucket name in AWS

# Upload the file
s3.upload_file(file_name, bucket_name, file_name)
print(f"Uploaded {file_name} to bucket {bucket_name}")

