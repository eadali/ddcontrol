# import tkinter as tk
# from tkinter.filedialog import askopenfilename
# #import pandas as pd


# def import_csv_data():
#     global v
#     csv_file_path = askopenfilename()
#     print(csv_file_path)
#     v.set(csv_file_path)
#     #df = pd.read_csv(csv_file_path)

# root = tk.Tk()
# tk.Label(root, text='File Path').grid(row=0, column=0)
# v = tk.StringVar()
# entry = tk.Entry(root, textvariable=v).grid(row=0, column=1)
# tk.Button(root, text='Browse Data Set',command=import_csv_data).grid(row=1, column=0)
# tk.Button(root, text='Close',command=root.destroy).grid(row=1, column=1)
# root.mainloop()




import tkinter

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import numpy as np
import filter_design
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt
from statsmodels.tsa.stattools import acovf




t = np.linspace(0, 1, 1000, False)  # 1 second
sig = np.sin(2*np.pi*10*t) + np.random.normal(size=t.shape)
a = filter_design.std(t, sig)
c = acovf(sig).max()
fs = 143*c
sos = signal.butter(10, 10, 'lp', fs=100, output='sos')
filtered = signal.sosfilt(sos, sig)


root = tkinter.Tk()
root.wm_title("Embedding in Tk")

fig = Figure(figsize=(5, 4), dpi=100)
ax = fig.add_subplot(111)
ax.plot(t, sig)
ax.plot(t, filtered)


canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.draw()
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

toolbar = NavigationToolbar2Tk(canvas, root)
toolbar.update()
canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)


def on_key_press(event):
    print("you pressed {}".format(event.key))
    key_press_handler(event, canvas, toolbar)


canvas.mpl_connect("key_press_event", on_key_press)


def _quit():
    root.quit()     # stops mainloop
    root.destroy()  # this is necessary on Windows to prevent
                    # Fatal Python Error: PyEval_RestoreThread: NULL tstate


button = tkinter.Button(master=root, text="Quit", command=_quit)
button.pack(side=tkinter.BOTTOM)

tkinter.mainloop()







# import numpy as np
# from scipy import signal
# from matplotlib import pyplot as plt

# data = np.loadtxt("./filter/foo.csv", delimiter=",")
# t = data[:,0]
# sig = data[:,1]

# # wc = calculate_cutoff_freq(sig, de)
# sos = signal.butter(10, 10, 'lp', fs=100, output='sos')
# filtered = signal.sosfilt(sos, sig)


# plt.plot(t, filtered)
# # plt.set_title('10 Hz and 20 Hz sinusoids')
# plt.axis([0, 1, -5, 5])
# plt.show()