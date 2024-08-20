import numpy as np


a = np.array([0.0, 0.0])
b = np.array([1.0, 0.0])

print(np.linalg.norm(a, ord=1))

print(np.dot(a, b))
