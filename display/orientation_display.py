# Sage Santomenna 2024

import serial
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import numpy as np
import time
import collections
from numpy import cross, eye, dot
from scipy.linalg import expm, norm
import quaternion
from quaternion.numba_wrapper import njit
import requests
from collections import namedtuple

MPU_COM_PORT = 12
BAUD = 57600
USE_SERIAL = False  # whether to use serial or web requests to get orientation
ORIENTATION_SERVER_ADDR = "http://localhost:8080/orientation"

X = np.array((1,0,0))
Y = np.array((0,1,0))
Z = np.array((0,0,1))

Orientation = namedtuple("Orientation", ["x","y","z"])

val_names = ["w","i","j","k"]

def trim_string_between(string, start_char, end_char):
    return string[string.find(start_char) + 1:string.find(end_char, string.find(start_char) + 1)] if start_char in string and end_char in string else ""


def _orientation(q: quaternion.quaternion):
    return quaternion.rotate_vectors(q,(X,Y,Z))

orientation = njit(_orientation)

def init_plot():
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set(xlim=(-1,1),ylim=(-1,1),zlim=(-1,1))
    plt.show(block=False)
    plt.draw() 
    background = fig.canvas.copy_from_bbox(ax.bbox)
    return fig, ax, background

serial_ready = False # set to true when serial is set up
port = None # used to store the serial port, if one is made

def setup_serial():
    global port, serial_ready # epic shared mutable state
    port = serial.Serial(port=f"COM{MPU_COM_PORT}", baudrate=BAUD)
    try:
        port.open()
    except serial.SerialException:
        port.reset_input_buffer()
    # # read to the end of a line before starting
    port.read_until()
    serial_ready = True

fig, ax, background = init_plot()

read = 0
rejected = 0

session = requests.Session()

def fetch_orientation():
    if USE_SERIAL:
        return fetch_orientation_serial()
    else:
        return fetch_orientation_web()

# not currently working: does not implement MIN protocol
def fetch_orientation_serial():
    msg = read_serial()
    port.reset_input_buffer()
    read += 1
    # print(msg)
    if msg is None or "#" not in msg or "%" not in msg:
        rejected += 1 
        print(f"[Rejecting] {msg}")
        # print(f"Have rejected {rejected} / {read} messages")
        return None

    msg = trim_string_between(msg, "#","%")
    try:
        vals = [float(v) for v in msg.split(" ") if v.replace(" ", "")]
    except ValueError as e:
        print(f"Malformed message: {msg}")
        print(f"Error: {e}")
        return None

    if len(vals) != len(val_names):
        print("Didn't get all vals!")
        print(msg)
        return None

    q = quaternion.from_float_array(vals)
    x, y, z = orientation(q)
    return Orientation(x,y,z)

def read_serial():
    if not serial_ready:
        setup_serial()
    pre = port.read_until(b"\r\n")
    try:
        msg = pre.decode("utf-8").rstrip()
        return msg
    except UnicodeDecodeError:
        return None

def fetch_orientation_web():
    try:
        res = session.get(ORIENTATION_SERVER_ADDR).json()
        x, y, z = np.array(res["x"]), np.array(res["y"]), np.array(res["z"])
        return Orientation(x,y,z)
    except requests.exceptions.RequestException as e:
        print(f"Failed to fetch orientation: {e}")
        return None

while True:
    # read data
    new_orientation = fetch_orientation()

    if new_orientation is None:
        continue

    x, y, z = new_orientation

    ax.cla()
    ax.set(xlim=(-1,1),ylim=(-1,1),zlim=(-1,1))
    fig.canvas.restore_region(background)
    fig.canvas.blit(ax.bbox)

    ax.quiver(0,0,0,x[0],x[1],x[2],color="red")
    ax.quiver(0,0,0,y[0],y[1],y[2],color="blue")
    ax.quiver(0,0,0,z[0],z[1],z[2],color="green")

    fig.canvas.blit(ax.bbox)
    plt.pause(0.0001)
    # t = time.perf_counter()
