import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

# 设置图例字号
mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()

# 设置三维图形模式
ax = fig.gca(projection='3d')
ax.legend()
# 测试数据

def drawStatic(line):
    theta = np.ones(100)
    z = np.linspace(0, 8, 100) / 4
    for L in line:
        x = L[0]*theta
        y = L[1]*theta
        ax.plot(x, y, z, color='orange')
def drawCurve():
    z = np.linspace(0, 6, 100) / 4
    y = np.linspace(-2, 3, 100)
    x = -1.5*np.cos((y+2)*0.2*np.pi)+1.5
    ax.plot(x,y,z,color='red')
# 显示图例
def drawDy1():
    y = np.linspace(1, -2, 100)
    x = -2/3*y+2/3
    z = np.zeros(100)
    for i in range(1,100):
        z[i] = 0.0002*i*i+0.01
    ax.plot(x, y, z, color='blue')
def drawDy2():
    x = np.linspace(1, 3, 100)
    y = -2*x+5
    z = np.zeros(100)
    for i in range(1,100):
        z[i] = 0.15*np.sqrt(i)
    ax.plot(x, y, z, color='blue')
# 显示图形

if __name__=='__main__':
    staticObstacle = [[2.2, 2]]
    drawStatic(staticObstacle)
    drawCurve()
    drawDy1()
    drawDy2()
    drawStatic([[0.5, -1.2]])

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    plt.show()