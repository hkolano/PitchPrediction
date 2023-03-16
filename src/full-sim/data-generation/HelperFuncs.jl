using Rotations

function convert_to_rpy(quat_vals)
    quat_rot = QuatRotation(quat_vals...)
    euler = RotXYZ(quat_rot)
    vals = [euler.theta1, euler.theta2, euler.theta3] 
end
