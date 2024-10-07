import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#import vlc
import sys, serial
from PIL import Image, ImageTk
import tkinter as tk
import threading  # Import threading

xsize = 500

# Configure the serial port
ser = serial.Serial(
    port='COM6',  # Update to the correct port
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()

# Declare global variables for Tkinter window and label
root = None
img_label = None

# Function to load images and update the Tkinter label
def update_image(state):
    global img_label
    images = {
        "green": ImageTk.PhotoImage(Image.open("green.png")),
        "yellow": ImageTk.PhotoImage(Image.open("yellow.png")),
        "red": ImageTk.PhotoImage(Image.open("red.png"))
    }
    img_label.config(image=images[state])
    img_label.image = images[state]  # Keep a reference

# Function to create and run the Tkinter window
def create_tkinter_window():
    global root, img_label
    root = tk.Tk()
    root.title("Temperature State")
    img_label = tk.Label(root)  # Placeholder for the image
    img_label.pack()
    root.mainloop()

# Start the Tkinter window in a separate thread
tkinter_thread = threading.Thread(target=create_tkinter_window, daemon=True)
tkinter_thread.start()

def data_gen():
    t = data_gen.t
    while True:
        t += 1
        val = float(ser.readline())
        yield t, val

def run(data):
    # Update the data
    t, y = data
    xdata.append(t)
    ydata.append(y)
    if t > xsize:  # Scroll to the left.
        ax.set_xlim(t - xsize, t)

    #play audio file through vlc media player
    # p = vlc.MediaPlayer("C:\Users\abhi1\OneDrive\Desktop\ELEC291_package\Project1\shboom.m4a") #insert audiofile location path here
    # p.play(shboom.m4a) #insert audio file name here

        

    # Set color based on temperature range and update image in Tkinter window
    color = 'green' if y <= 125 else ('yellow' if 125 < y <= 200 else 'red')
    line.set_data(xdata, ydata)
    line.set_color(color)
    update_image(color)  # Call update_image function here

    return line,

def on_close_figure(event):
    if root:
        root.quit()  # Close the Tkinter window properly
    sys.exit(0)
    
    #p.stop(shboom.m4a) #insert audio file name here
    
data_gen.t = 0
xdata, ydata = [], []
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)
ax = fig.add_subplot(111)
line, = ax.plot([], [], lw=2)
ax.set_ylim(0, 300)
ax.set_xlim(0, xsize)
ax.grid()

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False)

plt.show()
