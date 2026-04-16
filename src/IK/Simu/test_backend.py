import matplotlib
import matplotlib.pyplot as plt

print("Default backend:", matplotlib.get_backend())

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot([0, 1], [0, 1], [0, 1])
plt.show(block=False)
plt.pause(1)
print("Worked!")
