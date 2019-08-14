import numpy as np; np.random.seed(5)
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

#x = np.arange(10)
x = [3,3,3,7,8,9,20,30]
#y = np.random.choice(10,10)
y = [1,5,4,1,2,1,1,1]

points = np.array([x, y]).T.reshape(-1,1,2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

print(points[0][0][1])
print(segments)

print("Lenth:" + str(len(segments)))


cm = dict(zip(range(-1,2,1),list("grb")))
colors = list( map( cm.get , np.sign(np.diff(y))  ))

lc = LineCollection(segments, colors=colors, linewidths=2)
fig, ax = plt.subplots()
ax.add_collection(lc)

ax.autoscale()
ax.margins(0.1)
plt.show()

