import numpy as np
import matplotlib.pyplot as plt
from fmpy import read_model_description, extract, simulate_fmu

fmu_filename = "Lemuel_pump_system.fmu" # REPLACE WITH YOUR FMU FILE


# Read FMU model description and extract the FMU content
_ = read_model_description(fmu_filename)
_ = extract(fmu_filename)

# Create time vector: 11 steps from 0 to 1 second
time_step = np.linspace(0.0, 1, num=11)

# Define constant input values for testing (REPLACE WITH ACTUAL SENSOR DATA FOR REAL USAGE)
flow_target_values = np.full(time_step.shape, 30)
pump_speed_values = np.full(time_step.shape, 1900)

# Create a structured input array for the simulation (fields: time and two input variables)
input_values = np.zeros(time_step.shape, dtype=[('time', 'f8'),
                                               ('amesim_interface.flowrate_target', 'f8'),
                                               ('amesim_interface.pump_speed', 'f8')])
input_values['time'] = time_step
input_values['amesim_interface.flowrate_target'] = flow_target_values
input_values['amesim_interface.pump_speed'] = pump_speed_values
# IMPORTANT: USE THE NAMES FOR YOUR INPUTS

# Run the FMU simulation using the defined inputs
result = simulate_fmu(fmu_filename, start_time=0.0, stop_time=1, input=input_values)

time = result['time']

# Define output variable names to plot (adapt these according to your model outputs)
# IMPORTANT: USE THE NAMES FOR YOUR OUTPUTS
output_variable_1 = 'amesim_interface.flow_rate'
output_variable_2 = 'amesim_interface.pressure_increase'

# Check if output variables are present in the results before plotting
if output_variable_1 in result.dtype.names and output_variable_2 in result.dtype.names:
    plt.figure(figsize=(10, 5))

    # Plot the first output variable: flow rate
    plt.subplot(2, 1, 1)
    plt.plot(time, result[output_variable_1], label='flow_rate', color='b')
    plt.title('Simulation')
    plt.xlabel('Time (s)')
    plt.ylabel('flow_rate')
    plt.grid()
    plt.legend()

    # Plot the second output variable: pressure increase
    plt.subplot(2, 1, 2)
    plt.plot(time, result[output_variable_2], label='pump_pressure', color='g')
    plt.xlabel('Time (s)')
    plt.ylabel('pump_pressure')
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()
else:
    # Print a message if any output variable was not found
    if output_variable_1 not in result.dtype.names:
        print(f"The output variable '{output_variable_1}' was not found in the results.")
    if output_variable_2 not in result.dtype.names:
        print(f"The output variable '{output_variable_2}' was not found in the results.")
