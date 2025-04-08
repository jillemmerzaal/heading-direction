from scipy.spatial.transform import Rotation as R

def rotate_timeseries_global2local(gyro_global, acc_global, quats):
    """
    Rotate gyroscope and acceleration data from global frame to sensor's local frame.

    Parameters:
        gyro_global (Nx3 array): angular velocity in global frame
        acc_global (Nx3 array): linear acceleration in global frame
        quats (Nx4 array): quaternions representing sensor orientation (w, x, y, z)

    Returns:
        gyro_local (Nx3 array): angular velocity in local frame
        acc_local (Nx3 array): linear acceleration in local frame
    """
    # Convert quaternion from (w, x, y, z) to (x, y, z, w) for scipy
    quats_xyzw = quats[:, [1, 2, 3, 0]]

    # Create Rotation object
    r_global_to_local = R.from_quat(quats_xyzw).inv()

    # Rotate gyroscope and acceleration from global to local
    gyro_local = r_global_to_local.apply(gyro_global)
    acc_local = r_global_to_local.apply(acc_global)

    return gyro_local, acc_local