import numpy as np

def rotmat_to_quat_xyzw(R):
    t = np.trace(R)
    if t > 0.0:
        S = np.sqrt(t + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    else:
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            qw = (R[2,1] - R[1,2]) / S
            qx = 0.25 * S
            qy = (R[0,1] + R[1,0]) / S
            qz = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            qw = (R[0,2] - R[2,0]) / S
            qx = (R[0,1] + R[1,0]) / S
            qy = 0.25 * S
            qz = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            qw = (R[1,0] - R[0,1]) / S
            qx = (R[0,2] + R[2,0]) / S
            qy = (R[1,2] + R[2,1]) / S
            qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q) + 1e-12
    return q

def pca_pose_from_points(points, v_prev=None, principal_axis='x'):
    """
    Computes pose from PCA.
    principal_axis: 'x' or 'z' - which axis the main eigenvector aligns with.
    """
    if points is None or points.shape[0] < 50:
        return None, None, None

    c = points.mean(axis=0)
    Q = points - c
    C = (Q.T @ Q) / max(points.shape[0], 1)

    # Solve for eigenvectors
    w, V = np.linalg.eigh(C)
    v1 = V[:, np.argmax(w)]  # Main principal component

    # Continuity with previous frame
    if v_prev is not None and np.dot(v1, v_prev) < 0:
        v1 = -v1

    # Global orientation consistency (keep it pointing generally "up" or "forward")
    if np.dot(v1, np.array([0.0, 0.0, 1.0])) < 0:
        v1 = -v1

    # Normalize the principal vector
    v_main = v1 / (np.linalg.norm(v1) + 1e-12)

    # Reference vector to define the "up" direction for the remaining axes
    # Typically, we want the y-axis or z-axis to stay as close to global up/down as possible
    up_ref = np.array([0.0, -1.0, 0.0]) 

    if principal_axis.lower() == 'x':
        x_axis = v_main
        # Project up_ref onto the plane orthogonal to x_axis to find z
        z_axis = up_ref - np.dot(up_ref, x_axis) * x_axis
        if np.linalg.norm(z_axis) < 1e-6:
            z_axis = np.array([0.0, 0.0, 1.0]) - np.dot(np.array([0.0, 0.0, 1.0]), x_axis) * x_axis
        
        z_axis /= (np.linalg.norm(z_axis) + 1e-12)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= (np.linalg.norm(y_axis) + 1e-12)

    elif principal_axis.lower() == 'z':
        z_axis = v_main
        # Project up_ref (or a forward ref) onto the plane orthogonal to z_axis to find x
        x_ref = np.array([1.0, 0.0, 0.0])
        x_axis = x_ref - np.dot(x_ref, z_axis) * z_axis
        if np.linalg.norm(x_axis) < 1e-6:
            x_axis = np.array([0.0, 1.0, 0.0]) - np.dot(np.array([0.0, 1.0, 0.0]), z_axis) * z_axis
            
        x_axis /= (np.linalg.norm(x_axis) + 1e-12)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= (np.linalg.norm(y_axis) + 1e-12)
    else:
        raise ValueError("principal_axis must be 'x' or 'z'")

    # Build Rotation Matrix: columns are x, y, z
    Rm = np.column_stack((x_axis, y_axis, z_axis))
    
    quat = rotmat_to_quat_xyzw(Rm)
    return c, quat, v1