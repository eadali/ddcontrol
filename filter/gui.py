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
fs= 0
fig = Figure(figsize=(5, 4), dpi=100)
ax = fig.add_subplot(111)


def slider_update(a):
    hh.set(zz.get())
    sos = signal.butter(10, 10, 'lp', fs=hh.get(), output='sos')
    y = signal.sosfilt(sos, u[0])
    ax.clear()
    ax.plot(t[0],u[0])
    ax.plot(t[0],y)
    canvas.draw()

def import_csv_data():
    csv_file_path = askopenfilename()
    sig = np.genfromtxt(csv_file_path, delimiter=",")
    t.append(sig[:,0])
    u.append(sig[:,1])
    print(u)

    c = acovf(u[0]).max()
    hh.set(143*c)
    zz.set(hh.get())
    slider_update()




root = tk.Tk()
N = tk.StringVar()
zz = tk.DoubleVar()
hh = tk.DoubleVar()
tk.Label(root, text='The order of the filter:').grid(row=0, column=0)
tk.Spinbox(root, from_=0, to=20, textvariable=N).grid(row=0, column=1)
tk.Label(root, text='Normalized cutoff frequency:').grid(row=0, column=3)
tk.Scale(root, from_=0, to=1000, orient=tk.HORIZONTAL, variable=zz, command=slider_update).grid(row=0, column=4)
tk.Button(root, text='Show parameters').grid(row=0, column=5)


canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.get_tk_widget().grid(row=1, column=0, columnspan=5)
tk.Button(root, text='Browse Data Set',command=import_csv_data).grid(row=2, column=0)
v = tk.StringVar()
entry = tk.Entry(root, textvariable=v).grid(row=2, column=1)
tk.Button(root, text='Close',command=root.destroy).grid(row=2, column=5)
root.mainloop()