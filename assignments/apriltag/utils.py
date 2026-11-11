import pybullet
import io
import cv2
import base64
import numpy as np
import matplotlib.pyplot as plt


def createAprilBox(position, orientation=[0,0,0,1], scale=1):
    """ Create an April Tag box"""

    ext = scale
    halfExt = scale / 2.0

    # Create collision shape
    collisionShape = pybullet.createCollisionShape(
        shapeType=pybullet.GEOM_BOX,
        halfExtents=[halfExt]*3
    )

    # Create visual shape
    visualShape = pybullet.createVisualShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="assignments/apriltag/data/cube.obj",   # path to your OBJ file
        meshScale=[ext]*3
    )

    # Create body
    cubeID = pybullet.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collisionShape,
        baseVisualShapeIndex=visualShape,
        basePosition=position,
        baseOrientation=orientation
    )

    # Texture
    textureID = pybullet.loadTexture("assignments/apriltag/data/tag36_11_00000.png")
    pybullet.changeVisualShape( cubeID, -1, textureUniqueId=textureID)

    return cubeID


def rotationalError(a, b):
    """ Rotational error between rotation vectors a and b """
    aR = cv2.Rodrigues(a)[0]
    bR = cv2.Rodrigues(b)[0]
    return float(np.linalg.norm(cv2.Rodrigues(aR @ bR.T)[0]))


def summarize(implementation, show=True):
    """ Summarizes results """

    errorCv2 = np.array([sol['error'] if sol is not None else float('nan') for sol in implementation.cv2Solutions])
    errorHomography = np.array([sol['error'] if sol is not None else float('nan') for sol in implementation.homographySolutions])
    errorIterative = np.array([sol['error'] if sol is not None else float('nan') for sol in implementation.iterativeSolutions])

    print()
    print('cv2 reprojection error: mean = {:.5f}, median = {:.5f}'.format(
        errorCv2.mean(), np.median(errorCv2) 
    ))
    print('Homography reprojection error: mean = {:.5f}, median = {:.5f}'.format(
        errorHomography.mean(), np.median(errorHomography) 
    ))
    print('Iterative LM reprojection error: mean = {:.5f}, median = {:.5f}'.format(
        errorIterative.mean(), np.median(errorIterative) 
    ))

    if errorHomography.sum() > errorIterative.sum():
        print("Homography reprojection error does not exceed iterative solution, as expected")
    else:
        print("Iterative solution performs worse than homography. Incorrect implementation likely.")

    errorPositionHomography = np.array([
        np.linalg.norm(solH['aprilTagInCamera']['translation'] - solCv['aprilTagInCamera']['translation']) if solH is not None and solCv is not None else float('nan') for solH, solCv in zip(implementation.homographySolutions, implementation.cv2Solutions)
    ])
    errorPositionIterative = np.array([
        np.linalg.norm(solI['aprilTagInCamera']['translation'] - solCv['aprilTagInCamera']['translation']) if solI is not None and solCv is not None else float('nan') for solI, solCv in zip(implementation.iterativeSolutions, implementation.cv2Solutions)
    ])

    errorRotationHomography = np.array([
        rotationalError(solH['aprilTagInCamera']['rotation'], solCv['aprilTagInCamera']['rotation']) if solH is not None and solCv is not None else float('nan') for solH, solCv in zip(implementation.homographySolutions, implementation.cv2Solutions)
    ])
    errorRotationIterative = np.array([
        rotationalError(solI['aprilTagInCamera']['rotation'], solCv['aprilTagInCamera']['rotation']) if solI is not None and solCv is not None else float('nan') for solI, solCv in zip(implementation.iterativeSolutions, implementation.cv2Solutions)
    ])
    
    print('Homography mean position error compared to cv2: {:.5f}'.format(errorPositionHomography.mean()))
    print('Homography mean rotation error (rad) compared to cv2: {:.5f}'.format(errorRotationHomography.mean()))
    print('Iterative mean position error compared to cv2: {:.5f}'.format(errorPositionIterative.mean()))
    print('Iterative mean rotation error (rad) compared to cv2: {:.5f}'.format(errorRotationIterative.mean()))

    fig = plt.figure(figsize=(16,4))
    axs = fig.subplots(1,3)
    axs[0].plot(errorHomography, 'r-', label='reprojection error (homography)')
    axs[0].plot(errorIterative, 'b-', label='reprojection error (iterative)')
    axs[0].plot(errorCv2, 'g--', label='reprojection error (cv2)')
    axs[0].legend()
    axs[1].plot(errorPositionHomography, 'r.-', label='position error rel. to OpenCV (homography)')
    axs[1].plot(errorPositionIterative, 'b.-', label='position error rel. to OpenCV (iterative)')
    axs[1].plot(errorRotationHomography, 'rx-', label='rotation error rel. to OpenCV (homography)')
    axs[1].plot(errorRotationIterative, 'bx-', label='rotation error rel. to OpenCV (iterative)')
    axs[1].legend()
    axs[2].plot([sol['cameraInAprilTag']['translation'][2] if sol is not None else float('nan') for sol in implementation.homographySolutions], [sol['cameraInAprilTag']['translation'][0] if sol is not None else float('nan') for sol in implementation.homographySolutions], 'r-', alpha=0.5, label='path (homography)')
    axs[2].plot([sol['cameraInAprilTag']['translation'][2] if sol is not None else float('nan')  for sol in implementation.iterativeSolutions], [sol['cameraInAprilTag']['translation'][0] if sol is not None else float('nan') for sol in implementation.iterativeSolutions], 'b-', linewidth=1, alpha=0.5, label='path (iterative)')
    axs[2].plot([sol['cameraInAprilTag']['translation'][2] if sol is not None else float('nan')  for sol in implementation.cv2Solutions], [sol['cameraInAprilTag']['translation'][0] if sol is not None else float('nan') for sol in implementation.cv2Solutions], 'g--', linewidth=2, alpha=0.5, label='path (cv2)')
    axs[2].plot([0.0], [0.0], 'k.', markersize=10, label='AprilTag')
    axs[2].legend()
    axs[2].set_xlim(5,0)
    axs[2].set_ylim(2.5,-2.5)
    fig.tight_layout()

    if show:
        plt.show()
    else:
        buf = io.BytesIO()
        plt.savefig(buf, format="jpg")
        buf.seek(0)
        plt.close()
        b64_string = base64.b64encode(buf.read()).decode("utf-8")
        return "data:image/jpg;base64,{}".format(b64_string)
