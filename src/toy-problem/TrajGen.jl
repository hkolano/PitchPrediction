module TrajGen
# export gen_rand_waypoints, find_trajectory

num_its=50
joint_lims = [[-π/2, π/2], [-π/2, π/2]]
# Velocity limits: 30 degrees/s, from the Alpha documentation for the first 3 joints
vel_lims = [[-0.5326, 0.5326],[-0.5326, 0.5326]]

mutable struct jointState
    θs::Array{Float64}
    dθs::Array{Float64}
end

equil_pt = jointState([-0.18558, 0.0], [0.0, 0.0])
extended_pt = jointState([π/2-.05, 0.0], [0.0, 0.0])

mutable struct Waypoints
    start::jointState 
    goal::jointState
end

raise_wpts = Waypoints(equil_pt, extended_pt)

mutable struct trajParams
    a::Array
    wp::Waypoints
end

function gen_rand_feasible_point()
    θj1 = rand(joint_lims[1][1]:.001:joint_lims[1][2])
    θj2 = rand(joint_lims[2][1]:.001:joint_lims[2][2])
    dθj1 = rand(vel_lims[1][1]:.001:vel_lims[1][2])
    dθj2 = rand(vel_lims[2][1]:.001:vel_lims[2][2])
    return jointState([θj1, θj2], [dθj1, dθj2])
end

function gen_rand_waypoints()
    Waypoints(gen_rand_feasible_point(), gen_rand_feasible_point())    
end

function gen_rand_waypoints_from_equil()
    Waypoints(equil_pt, gen_rand_feasible_point()) 
end

function get_coeffs(pts::Waypoints, T, idx)
    # λ1 = dθ1/(θ2-θ1)
    λ1 = pts.start.dθs[idx]/(pts.goal.θs[idx]-pts.start.θs[idx])
    # λ2 = dθ2/(θ2-θ1)
    λ2 = pts.goal.dθs[idx]/(pts.goal.θs[idx]-pts.start.θs[idx])

    a0 = 0
    a1 = λ1
    a2 = 0
    a3 = -2(3T*λ1+2T*λ2-5)/(T^3)
    a4 = (8T*λ1 + 7T*λ2-15)/(T^4)
    a5 = -3(T*λ1+T*λ2-2)/(T^5)
    return [a0, a1, a2, a3, a4, a5]
end

function pos_scale_at_t(a, t)
    s = a[1] + a[2]*t + a[3]*t^2 + a[4]*t^3 + a[5]*t^4 + a[6]*t^5
end

function vel_scale_at_t(a, t)
    ds = a[2] + 2a[3]*t + 3a[4]*t^2 + 4a[5]*t^3 + 5a[6]*t^4
end

function acc_scale_at_t(a, t)
    dds = 2a[3] + 6a[4]*t + 12a[5]*t^2 + 20a[6]*t^3
end

function get_path!(poses, vels, θ1, θ2, T, a, num_its=num_its)
    dt = T/num_its

    for i = 1:num_its
        t = dt*i
        s = pos_scale_at_t(a, t)
        ds = vel_scale_at_t(a, t)
        poses[i] = θ1 + s*(θ2-θ1)
        vels[i] = ds*(θ2-θ1)
    end
    return poses, vels 
end

function get_desv_at_t(t, p)
    # println("Got request for desv. Params $(p))")
    des_vel = [0.0, 0.0]
    for i = 1:2
        ds = vel_scale_at_t(p.a[i,:], t)
        des_vel[i] = ds*(p.wp.goal.θs[i]-p.wp.start.θs[i])
    end
    return des_vel
end

function check_lim(vals::Array, lims, idx)
    if minimum(vals) < lims[idx][1] || maximum(vals) > lims[idx][2]
        is_in_range = false
    else
        is_in_range = true
    end
    return is_in_range
end

function scale_trajectory(params, dur, poses, vels)
    a = Array{Float64}(undef, 2, 6)
    scale_factor = rand(1:.01:5)
    T = dur*scale_factor
    # println("Scaling factor: $(scale_factor)")
    for i in 1:2
        a[i,:] = get_coeffs(params.wp, T, i)
        (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], params.wp.start.θs[i], params.wp.goal.θs[i], T, a[i,:])
    end
    return [trajParams(a, params.wp), T, poses, vels]
end

function find_trajectory(pts::Waypoints; num_its=num_its, T_init=1.0)
    T = T_init
    poses = Array{Float64}(undef, num_its, 2)
    vels = Array{Float64}(undef, num_its, 2)
    feasible_ct = 0
    a = Array{Float64}(undef, 2, 6)

    while feasible_ct < 2 && T < 10.0
        feasible_ct = 0
        for i in 1:2
            # Get trajectory 
            a[i,:] = get_coeffs(pts, T, i)
            (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], pts.start.θs[i], pts.goal.θs[i], T, a[i,:])

            # Evaluate if possible
            is_in_range_poses = check_lim(poses, joint_lims, i)
            is_in_range_vels = check_lim(vels, vel_lims, i)
            if is_in_range_poses == true && is_in_range_vels == true
                feasible_ct = feasible_ct + 1
            end
        end
        if feasible_ct < 2
            T = T + 0.2
        end
    end

    if feasible_ct == 2
        # println("Trajectory Parameters set")
        # println("Poses: $(poses)")
        return [trajParams(a, pts), T, poses, vels]
        # println("Trajectory parameters set")
    else
        # println("No path between points; try again.")
        return nothing
    end

end

end