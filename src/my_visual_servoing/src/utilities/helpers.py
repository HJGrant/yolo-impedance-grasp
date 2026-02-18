import numpy as np
import tf.transformations as tft

def rotmat_to_quat_xyzw(R):
    """Safe conversion from 3x3 rotation matrix to xyzw quaternion."""
    q = tft.quaternion_from_matrix(np.vstack([np.column_stack([R, [0, 0, 0]]), [0, 0, 0, 1]]))
    return q

def clamp_vector_to_up(v_pca, v_up, max_angle_deg=15.0):
    """
    Constrains the PCA vector to stay within a cone of tolerance around v_up.
    If the vector exceeds the angle, it is pulled back to the boundary.
    """
    v_pca = v_pca / (np.linalg.norm(v_pca) + 1e-12)
    v_up = v_up / (np.linalg.norm(v_up) + 1e-12)
    
    dot_prod = np.clip(np.dot(v_pca, v_up), -1.0, 1.0)
    current_angle = np.arccos(dot_prod)
    max_angle_rad = np.radians(max_angle_deg)
    
    if current_angle <= max_angle_rad:
        return v_pca
    
    # Calculate rotation axis to pull vector back toward Up
    axis = np.cross(v_up, v_pca)
    norm = np.linalg.norm(axis)
    if norm < 1e-6:
        return v_up
    axis /= norm
    
    # Create rotation to the limit
    res_quat = tft.quaternion_about_axis(max_angle_rad, axis)
    # Convert v_up to 4D for multiplication
    v_up_4d = [v_up[0], v_up[1], v_up[2], 0]
    # Rotate v_up by max_angle toward v_pca
    matrix = tft.quaternion_matrix(res_quat)
    v_clamped = np.dot(matrix[:3, :3], v_up)
    return v_clamped

def pca_pose_from_points(points, v_prev=None, principal_axis='x', max_tilt_deg=15.0):
    """
    Computes a stabilized pose from points.
    1. Finds principal axis via PCA.
    2. Constrains that axis (x) to remain within tolerance of 'Up'.
    3. Stabilizes rotation around the principal axis.
    """
    if points is None or points.shape[0] < 50:
        return None, None, None

    # 1. Position Centroid
    c = points.mean(axis=0)
    
    # 2. PCA for principal direction
    Q = points - c
    C = (Q.T @ Q) / points.shape[0]
    w, V = np.linalg.eigh(C)
    v1 = V[:, np.argmax(w)]  # Primary axis

    # 3. Reference 'Up' (In camera optical frame, Y is down, so Up is [0, -1, 0])
    up_ref = np.array([0.0, -1.0, 0.0])

    # Ensure PCA vector points generally Up
    if np.dot(v1, up_ref) < 0:
        v1 = -v1

    # Apply continuity with previous frame to prevent 180 flip jitter
    if v_prev is not None and np.dot(v1, v_prev) < 0:
        v1 = -v1
    
    # 4. Apply Geometric Clamp (The tolerance constraint)
    # This prevents the bottle x-axis from "flopping" horizontally
    x_axis = clamp_vector_to_up(v1, up_ref, max_tilt_deg)

    # 5. Build Orthogonal Coordinate System
    # Use PCA raw data to influence Y and Z but force them 90 deg to our clamped X
    z_ref = np.array([0.0, 0.0, 1.0])
    y_axis = np.cross(z_ref, x_axis)
    y_norm = np.linalg.norm(y_axis)
    
    if y_norm < 1e-6: # Handle singularity if x_axis aligns with z_ref
        y_axis = np.array([0.0, 1.0, 0.0])
    else:
        y_axis /= y_norm
        
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)

    # Build Rotation Matrix
    Rm = np.column_stack((x_axis, y_axis, z_axis))
    quat = rotmat_to_quat_xyzw(Rm)

    return c, quat, v1