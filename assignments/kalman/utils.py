import io
import base64
import numpy as np
import matplotlib.pyplot as plt


def plotOdometry3D(implementation, show=True):
    """ Plots the odometry as a 3D plot
    
    :param implementation: implementation object to use
    :param show: whether to show this output (if True) or just return a b64 image string for PrairieLearn (if False)
    """
    measurements = implementation.measurements
    filterStates = implementation.kalmanFilterStates
    groundtruth = implementation.groundtruthOdometry
    
    noisyPos = np.array(measurements)[:,0:2]
    filterPos = np.array([p[0:2] for p in filterStates])
    groundtruthPos = np.array([p for p, R in groundtruth])

    noisyHeading = np.array(measurements)[:,2]
    filterHeading = np.array([p[2] for p in filterStates])
    
    fig = plt.figure(figsize=(16,4))
    ax1 = fig.add_subplot(1,3,1, projection='3d')
    ax1.plot(noisyPos[:,0], noisyPos[:,1], 'r.', label='GPS', alpha=0.5)
    ax1.plot(filterPos[:,0], filterPos[:,1], 'b-', label='Filtered')
    ax1.plot(groundtruthPos[:,0], groundtruthPos[:,1], 'k--', alpha=0.7, label='Groundtruth')
    ax1.set_title('Robot State (3D)')
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_zlabel('z (m)')
    ax1.legend()
    ax1.set_box_aspect([1,1,0.5])

    center = np.array([
        (groundtruthPos[:,0].min() + groundtruthPos[:,0].max())/2.0,
        (groundtruthPos[:,1].min() + groundtruthPos[:,1].max())/2.0,
    ])
    bounds = np.array([
        groundtruthPos[:,0].max() - groundtruthPos[:,0].min(),
        groundtruthPos[:,1].max() - groundtruthPos[:,1].min(),
    ])
    ax1.set_xlim(center[0] - bounds.max(), center[0] + bounds.max())
    ax1.set_ylim(center[1] - bounds.max(), center[1] + bounds.max())
    ax1.set_zlim(-1, 1)

    ax2 = fig.add_subplot(1,3,2)
    ax2.plot(noisyPos[:,0], noisyPos[:,1], '.', color='orange', label='GPS', alpha=0.5, markersize=2.5)
    ax2.plot(filterPos[:,0], filterPos[:,1], 'b-', label='Filtered')
    ax2.plot(groundtruthPos[:,0], groundtruthPos[:,1], 'k--', alpha=0.7, label='Groundtruth')
    ax2.set_title('Robot Position (2D)')
    ax2.set_xlabel('x (m)')
    ax2.set_ylabel('y (m)')
    ax2.set_xlim(center[0] - bounds.max(), center[0] + bounds.max())
    ax2.set_ylim(center[1] - bounds.max(), center[1] + bounds.max())
    ax2.legend()

    ax3 = fig.add_subplot(1,3,3)
    ax3.plot(np.arange(0, len(noisyHeading)), noisyHeading, '.' , color='orange', label='Compass')
    ax3.plot(np.arange(0, len(filterHeading)), filterHeading, 'b.', label='Filtered', markersize=2.5)
    ax3.set_title('Robot Heading')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Reading (rad)')
    ax3.legend()
    
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
