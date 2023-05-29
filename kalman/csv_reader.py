import numpy as np
import importlib.util
import sys


def load_csv(path):
    return np.genfromtxt(path, delimiter=",")


def load_model(path):
    spec = importlib.util.spec_from_file_location("model.Model", path)
    foo = importlib.util.module_from_spec(spec)
    sys.modules["module.Model"] = foo
    spec.loader.exec_module(foo)
    return foo.Model()

path = r"C:\eradali\personal\ddcontrol\kalman\model.py"
load_model(path).say()


