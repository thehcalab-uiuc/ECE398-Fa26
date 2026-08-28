#
# You do not need to modify this file.
#

import io
import base64
import numpy as np
import matplotlib.pyplot as plt


def plotOdometry3D(implementation, deltaTime, rotationInterval=1.0, show=True):
    """ Plots the odometry as a 3D plot
    
    :param implementation: implementation object to use
    :param deltaTime: delta time to use
    :param rotationInterval: interval in seconds to plot rotations as axes
    :param show: whether to show this output (if True) or just return a b64 image string for PrairieLearn (if False)
    """
    measured = implementation.odometry
    groundtruth = implementation.groundtruthOdometry
    
    odomPos = np.array([p for p, R in measured])
    odomRot = np.array([R for p, R in measured])    
    groundtruthPos = np.array([p for p, R in groundtruth])
    rotationIndices = np.arange(0, len(measured), max(1, int(rotationInterval/deltaTime)))
    
    fig = plt.figure(figsize=(6,4), dpi=100)
    ax = fig.add_subplot(1,1,1, projection='3d')
    ax.plot(odomPos[:,0], odomPos[:,1], odomPos[:,2], 'b-', label='Computed Odometry')
    ax.plot(groundtruthPos[:,0], groundtruthPos[:,1], groundtruthPos[:,2], 'k--', alpha=0.7, label='Groundtruth')
    for rotationIndex in rotationIndices:
        p = odomPos[rotationIndex]
        R = odomRot[rotationIndex]
        ax.quiver(p[0], p[1], p[2], R[0,0], R[1,0], R[2,0], color='r', length=0.2)
        ax.quiver(p[0], p[1], p[2], R[0,1], R[1,1], R[2,1], color='g', length=0.2)
        ax.quiver(p[0], p[1], p[2], R[0,2], R[1,2], R[2,2], color='b', length=0.2)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()
    ax.set_box_aspect([1,1,0.5])

    center = np.array([
        (odomPos[:,0].min() + odomPos[:,0].max())/2.0,
        (odomPos[:,1].min() + odomPos[:,1].max())/2.0,
        (odomPos[:,2].min() + odomPos[:,2].max())/2.0,
    ])
    bounds = np.array([
        odomPos[:,0].max() - odomPos[:,0].min(),
        odomPos[:,1].max() - odomPos[:,1].min(),
        odomPos[:,2].max() - odomPos[:,2].min()
    ])
    ax.set_xlim(center[0] - bounds.max(), center[0] + bounds.max())
    ax.set_ylim(center[1] - bounds.max(), center[1] + bounds.max())
    ax.set_zlim(center[2] - bounds.max(), center[2] + bounds.max())
    plt.tight_layout()

    if show:
        plt.show()
    else:
        buf = io.BytesIO()
        plt.savefig(buf, format="jpg")
        buf.seek(0)
        plt.close()
        b64_string = base64.b64encode(buf.read()).decode("utf-8")
        return "data:image/jpg;base64,{}".format(b64_string)
