import serial
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.widgets as widgets
import time

# Serial setup
ser = serial.Serial('COM3', 9600, timeout=1)  # Adjust as needed

Output = []
Pitch = []
Target = []
data_lock = threading.Lock()

MAX_POINTS = 5000
show_output = True

def read_serial():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip().split()
            try:
                with data_lock:
                    for data in line:
                        if ':' in data:
                            Label, num = data.split(':')
                            num = float(num)
                            if Label == 'Output':
                                Output.append(num)
                            elif Label == 'Pitch':
                                Pitch.append(num)
                            elif Label == 'Target':
                                Target.append(num)

                            # Limit the data to MAX_POINTS
                            if len(Output) > MAX_POINTS:
                                Output.pop(0)
                            if len(Pitch) > MAX_POINTS:
                                Pitch.pop(0)
                            if len(Target) > MAX_POINTS:
                                Target.pop(0)
            except Exception as error:
                print('\nReceived:', line)

def send_serial_data():
    while True:
        user_input = input("Enter data to send (e.g., 'Output', 'Pitch', 'Target'): ")
        try:
            ser.write(user_input.encode())
            print(f"Sent: {user_input}")
        except Exception as e:
            print("Error sending data:", e)

serial_thread = threading.Thread(target=read_serial, daemon=True)
send_thread = threading.Thread(target=send_serial_data, daemon=True)

serial_thread.start()
send_thread.start()

fig, ax = plt.subplots(figsize=(16, 12)) 
line_output, = ax.plot([], [], label='Output')
line_pitch, = ax.plot([], [], label='Pitch')
line_target, = ax.plot([], [], label='Target')

ax.set_title("Live Serial Data")
ax.set_xlabel("Sample Index")
ax.set_ylabel("Value")
ax.legend(loc='upper left')

def toggle_output(event):
    global show_output
    show_output = not show_output
    if show_output:
        print("Output graph enabled.")
    else:
        print("Output graph disabled.")

# Create a button to toggle Output visibility
ax_button = plt.axes([0.81, 0.01, 0.18, 0.05])  # Button position [x, y, width, height]
button = widgets.Button(ax_button, 'Toggle Output')
button.on_clicked(toggle_output)

def update(frame):
    with data_lock:
        max_len = max(len(Output), len(Pitch), len(Target), 1)

        # Update the Output graph only if show_output is True
        if show_output:
            line_output.set_data(range(len(Output)), Output)
        else:
            line_output.set_data([], [])

        line_pitch.set_data(range(len(Pitch)), Pitch)
        line_target.set_data(range(len(Target)), Target)

        ax.set_xlim(max(0, max_len - MAX_POINTS), max_len)

        all_data = Output + Pitch + Target
        if all_data:
            min_y = min(all_data)
            max_y = max(all_data)
            ax.set_ylim(min_y - 10, max_y + 10)

    return line_output, line_pitch, line_target

# Animation loop
ani = animation.FuncAnimation(fig, update, interval=100)

try:
    plt.show()
except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()
