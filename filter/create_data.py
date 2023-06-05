import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

t = np.linspace(0, 1, 1000, False)  # 1 second
sig = np.sin(2*np.pi*10*t) + np.random.normal(size=t.shape)
np.savetxt("foo.csv", sig, delimiter=",")


fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(t, sig)
ax1.set_title('10 Hz and 20 Hz sinusoids')
ax1.axis([0, 1, -5, 5])
# sos = signal.butter(10, 10, 'lp', fs=1000, output='sos')
# filtered = signal.sosfilt(sos, sig)
# ax2.plot(t, filtered)
# ax2.set_title('After 15 Hz high-pass filter')
# ax2.axis([0, 1, -2, 2])
# ax2.set_xlabel('Time [seconds]')
# plt.tight_layout()
plt.show()