import tkinter as tk
from tkinter.filedialog import askopenfilename

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import numpy as np
from scipy import signal
from statsmodels.tsa.stattools import acovf

t = []
u = []
y = []
fig = Figure(figsize=(5, 4), dpi=100)
ax = fig.add_subplot(111)

def import_csv_data():
    csv_file_path = askopenfilename()
    sig = np.genfromtxt(csv_file_path, delimiter=",")
    t = sig[:,0]
    u = sig[:,1]

    c = acovf(u).max()
    fs = 143*c
    sos = signal.butter(10, 10, 'lp', fs=100, output='sos')
    y = signal.sosfilt(sos, u)
    zz.set(fs)


    ax.plot(t,u)
    ax.plot(t,y)
    canvas.draw()

root = tk.Tk()
N = tk.StringVar()
zz = tk.DoubleVar()
tk.Label(root, text='The order of the filter:').grid(row=0, column=0)
tk.Spinbox(root, from_=0, to=20, textvariable=N).grid(row=0, column=1)
tk.Label(root, text='Normalized cutoff frequency:').grid(row=0, column=3)
tk.Scale(root, from_=0, to=100, orient=tk.HORIZONTAL, variable=zz).grid(row=0, column=4)
tk.Button(root, text='Show parameters').grid(row=0, column=5)


canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.get_tk_widget().grid(row=1, column=0, columnspan=5)
tk.Button(root, text='Browse Data Set',command=import_csv_data).grid(row=2, column=0)
v = tk.StringVar()
entry = tk.Entry(root, textvariable=v).grid(row=2, column=1)
tk.Button(root, text='Close',command=root.destroy).grid(row=2, column=5)
root.mainloop()