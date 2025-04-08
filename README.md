# Heading direction
Simple code to "fix" the Xsens link output back to local coordinate system. 

## The issue

The gyroscope data of the shank from the Xsens link system is represented in the **global reference frame**. Meaning that a change in direction will change the signal representation (Figure 1 and Figure 2). This is especially clear with the 5th walk bout that has a 180 degree turn in there (figure 1). All the associated step detection methods (from “toolbox 1”), the signal from a single axis (e.g. ML gyroscope of the shank) is used. Considering that walking in a free living, outdoor environment comes with directional changes, these models are not applicable to real life circumstances without re-expressing the signals in the local reference frame (or using different sensors). 

![Figure 1: the medio-lateral gyroscope signal from the left shank expressed in the global reference frame. The colored background are the different surfaces. ](https://github.com/jillemmerzaal/heading-direction/blob/main/global-gyro.png)

Figure 1: the medio-lateral gyroscope signal from the left shank expressed in the global reference frame. The colored background are the different surfaces. 

![figure 2: The associated walking parcours from figure 1. ](https://github.com/jillemmerzaal/heading-direction/blob/main/walking-course-A.png)

figure 2: The associated walking parcours from figure 1. 

## Possible solution

Since our gyroscope data is in the **global reference frame**, but the sensor quaternions provide the orientation of the shank, we can **rotate the gyroscope data back into the local (sensor) frame**. This is done by applying the inverse (or conjugate) of the quaternion to the global gyroscope vector.

### Steps:

- **Express the Gyroscope Data as a Quaternion**
    
    Since angular velocity is a vector, we represent it as a **pure quaternion**:
    

```math
\omega=(0,\omega_{x},\omega_{y},\omega_{z})
```

where $\omega_{x}$,$\omega_{y}$,$\omega_{z}$ are the global gyroscope readings.

- **Compute the Quaternion Conjugate (Inverse Rotation)**
    
    The sensor's quaternion q=(w,x,y,z) represents the orientation of the shank in the global frame.
    
    The conjugate of $q{^*}$, which undoes this rotation, is:
    
    ```math
    q^{∗}=(w,−x,−y,−z)
    ```
    
    This quaternion converts global data into the local (shank) frame.
    
- **Apply the Rotation**
    
    The corrected gyroscope data in the **local sensor frame** is given by:
    
    ```math
    ω_{local}=q^{*}ωq
    ```
    

![Figure 3: Sensor rotated from global representation to the local representation.  ](https://github.com/jillemmerzaal/heading-direction/blob/main/local-gyro.png)

Figure 3: Sensor rotated from global representation to the local representation.  

This approach has seemed to solve the issue of walking direction. 

## Matlab implementation

```matlab
function [gyro_local, acc_local] = global2local(gyro_global, acc_global, quat)
%GLOBAL2LOCAL % Rotate sensor data from the global frame to the sensor's local frame
% 
% Parameters:
%   gyro_global: Nx3 matrix of angular velocity (rad/s) in the global frame
%   acc_global: Nx3 matrix of linear acceleration (m/s^2) in the global
%   frame
%   quat: Nx4 matrix of quaternions (w, x, y, z) from the sensor
%
% Returns:
%   gyro_local: Nx3 matrix of angular velocity in the local (sensor) frame
%   acc_local: Nx3 matrix of linear acceleration in the local (sensor)
%   frame

    % Convert quaternion to a quaternion object (MATLAB uses [w, x, y, z] format)
    N = size(gyro_global,1);
    gyro_local = zeros(N,3);
    acc_local = zeros(N,3);

    q = quaternion(quat);  % Convert all quaternions at once
    gyro_local = rotatepoint(conj(q), gyro_global); % Apply inverse rotation 
    acc_local = rotatepoint(conj(q), acc_global); 

end

% Example use.
[gyro_local_pelvis, acc_local_pelvis] = global2local(raw_pelvis_gyro, raw_pelvis_acc, quat_pelvis);
[gyro_local_Rshank, acc_local_Rshank] = global2local(raw_Rshank_gyro, raw_Rshank_acc, quat_Rshank);
[gyro_local_Lshank, acc_local_Lshank] = global2local(raw_Lshank_gyro, raw_Lshank_acc, quat_Lshank);
```

## Python implementation

```python
from scipy.spatial.transform import Rotation as R

def global2local(gyro_global, acc_global, quats):
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
```

