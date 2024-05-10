# Sage Santomenna 2024
# read data over serial from an arduino and display it for PID tuning purposes

import serial
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import numpy as np
import time
import collections
from numpy import cross, eye, dot
from scipy.linalg import expm, norm
import random

DRONE_COM_PORT = 18
BAUD = 38400

# x target, y target, x val, y val, fcw motor val, bcw, fccw, bccw
val_names = ["X0", "Y0", "X", "Y", "FCW", "BCW", "FCCW", "BCCW"]

def trim_string_between(string, start_char, end_char):
    return string[string.find(start_char) + 1:string.find(end_char, string.find(start_char) + 1)] if start_char in string and end_char in string else ""


def init_plot():
    # fig, axes = plt.subplots(1, 2)
    fig, axes = plt.subplots(1, 2,figsize=(10,5))
    plt.show(block=False)
    plt.draw() 
    backgrounds = [fig.canvas.copy_from_bbox(ax.bbox) for ax in axes]

    return fig, axes, backgrounds

port = serial.Serial(port=f"COM{DRONE_COM_PORT}", baudrate=BAUD)
try:
    port.open()
except serial.SerialException:
    port.reset_input_buffer()

# read to the end of a line before starting
port.read_until()

def read_serial():
    pre = port.read_until(b"\r\n")
    try:
        msg = pre.decode("utf-8").rstrip()
        return msg
    except UnicodeDecodeError:
        return None

fig, axes, backgrounds = init_plot()

max_length = 1000

timestamps = collections.deque(maxlen=max_length)
data_lists = [collections.deque(maxlen=max_length) for _ in val_names]

lines = []

# prepare all of our lines
# x target
line, = axes[0].plot([], [], label=val_names[0], color="red",linestyle="dashed")
lines.append(line)
# y target
line, = axes[0].plot([], [], label=val_names[1], color="blue",linestyle="dashed")
lines.append(line)
# x
line, = axes[0].plot([], [], label=val_names[2], color="red")
lines.append(line)
# y
line, = axes[0].plot([], [], label=val_names[3], color="blue")
lines.append(line)

# motor values
for i in range(4,8):
    line, = axes[1].plot([], [], label=val_names[i],alpha=0.5)
    lines.append(line)

axes[0].legend()
axes[1].legend()
axes[1].xaxis.set_tick_params(labelbottom=False)
axes[0].xaxis.set_tick_params(labelbottom=False)

run_id = random.randrange(1000,9999)


def on_press(event):
    if event.key == 'd':
        print("Saving...")
        plt.savefig(f"out/{P}_{I}_{D}_{run_id}.png")
    if event.key == 'x':
        port.write(bytes("stop","utf8"))
        exit(0)

def on_close(event):
    port.write(bytes("stop","utf8"))
    # plt.savefig(f"out/{P}_{I}_{D}_{run_id}.png")
    exit(0)

fig.canvas.mpl_connect('close_event', on_close)
fig.canvas.mpl_connect('key_press_event', on_press)

read = 0
rejected = 0

P = input("Enter the P value: ")
I = input("Enter the I value: ")
D = input("Enter the D value: ")

fig.suptitle(f"Run {run_id}: P={P}, I={I}, D={D}")

start = bytes(input("Send a character to begin ESC calibration: "),"utf8")
port.write(start)



while True:
    # read data
    msg = read_serial()
    port.reset_input_buffer()
    read += 1
    if msg is None or "#" not in msg or "%" not in msg:
        rejected += 1 
        if rejected < 15:
            print(f"Have rejected {rejected} / {read} messages")
        continue
    msg = trim_string_between(msg, "#","%")
    try:
        vals = [float(v) for v in msg.split(" ") if v.replace(" ", "").replace("#","").replace("%","")]
    except ValueError as e:
        print(f"Malformed message: {msg}")
        print(f"Error: {e}")
        continue

    if len(vals) != len(data_lists):
        print("Didn't get all vals!")
        print(msg)
        continue

    for val, l in zip(vals, data_lists):
        l.append(val)
    #update the graph
    t = time.perf_counter()
    timestamps.append(t)
    for ax, background in zip(axes, backgrounds):
        fig.canvas.restore_region(background)
        # update the plot for ax here
        fig.canvas.blit(ax.bbox)
    i = 0

    for line, data in zip(lines, data_lists):
        i += 1
        ax = axes[0] if i < 4 else axes[1]
        line.set_data(timestamps, data)  # update the line data
        ax.draw_artist(line)
    
    for ax in axes:
        ax.relim()  # recalculate limits
        ax.autoscale_view(True,True,True)  # rescale the plot
        # ax.set_aspect('equal', adjustable='box')
    plt.pause(0.0001)
    fig.canvas.blit(axes[0].bbox)
    fig.canvas.blit(axes[1].bbox)

