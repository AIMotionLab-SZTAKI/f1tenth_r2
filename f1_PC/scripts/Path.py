import numpy as np
import math
from scipy.interpolate import splev, splprep, splrep
import scipy.interpolate
import numpy
import matplotlib.pyplot as plt

class Path:
    def __init__(
        self,
        path_points,
    ):
        self.x_points = path_points[:, 0].tolist()
        self.y_points = path_points[:, 1].tolist() ###!!!!!!!!!!!!!!

        if (
            path_points[0, 0] == path_points[-1, 0]
            and path_points[0, 1] == path_points[-1, 1]
        ):
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.1)  # closed s-t vissza 0.001-re
        elif len(self.x_points):
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.1)  # line
        else:
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.1)  # curve

        u = np.arange(0, 1.001, 0.001)
        path = splev(u, tck)

        (X, Y) = path
        s = np.cumsum(np.sqrt(np.sum(np.diff(np.array((X, Y)), axis=1) ** 2, axis=0)))
        self.length = s[-1]

        par = np.linspace(0, self.length, 1001)
        self.par = np.reshape(par, par.size)
        self.tck, self.u, *rest = splprep([X, Y], k=2, s=0.1, u= par)
        x = np.array(par)
        y = np.ones(len(par))*0.8
        step = 0/len(par)
        for i in range(len(par)):
            y[i] += step*i


        #print(y)
        #ax[0].plot(self.x_points, self.y_points, "o")
# Generate the spline representation
        self.speed_tck = splrep(x, y, k=1, s=0.1)  # k is the degree, s is the smoothing factor (0 for interpolation)
# Evaluate the spline at the desired x-values
        self.speed = splev(self.u, self.speed_tck)

    def show(self):
        fig, ax = plt.subplots(2)
        fig.tight_layout(pad= 3)
        #ax[0].set_ylim([-0.1, 0.1])
        ax[0].plot(self.par, self.speed)
        
        ax[0].set_title("Speed")
        ax[1].set_title("Trajectory")
        ax[0].set_xlabel("Distance [m]")
        ax[0].set_ylabel("Speed")
        (self.X_ev,self.Y_ev)=splev(self.u, self.tck)
        ax[1].plot(self.X_ev,self.Y_ev)
        #print(par[-1])
        plt.show()
