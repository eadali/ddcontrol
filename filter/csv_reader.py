import numpy as np
import importlib.util
import sys


def load_csv(path):
    return np.genfromtxt(path, delimiter=",")



