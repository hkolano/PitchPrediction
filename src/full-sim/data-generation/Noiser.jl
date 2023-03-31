mutable struct NoiseCache
    noisy_qs 
    noisy_vs
    rand_walks

    function NoiseCache(state)
        new(configuration(state), velocity(state), zeros(6))
    end
end

mutable struct FilterCache
    filtered_vs 
    filtered_state :: MechanismState
    
    function FilterCache(state)
        new(zeros(num_velocities(state.mechanism)), MechanismState(state.mechanism))
    end
end

function add_sensor_noise(state, c, result)
    noisy_poses = similar(configuration(state))
    noisy_vels = similar(velocity(state))
    fill!(noisy_poses, 0.)
    fill!(noisy_vels, 0.)

    c.n_cache.rand_walks[1:3] .+= rand(gyro_rand_walk_dist, 3)
    c.n_cache.rand_walks[4:6] .+= rand(accel_rand_walk_dist, 3)

    last_noisy_R = Rotations.QuatRotation(c.n_cache.noisy_qs[1:4,end])
    add_arm_noise!(noisy_poses, noisy_vels, configuration(state)[8:end], c.n_cache.noisy_qs[8:end,end], 1/ctrl_freq)
    new_noisy_R = add_rotational_noise!(noisy_poses, noisy_vels, velocity(state)[1:3], last_noisy_R, 1/ctrl_freq, c.n_cache.rand_walks)
    add_linear_noise!(noisy_poses, noisy_vels, result.v̇[1:6], c.last_v̇, c.n_cache.noisy_vs[1:6,end], c.n_cache.noisy_qs[5:7,end], last_noisy_R, new_noisy_R, 1/ctrl_freq, c.n_cache.rand_walks)

    c.last_v̇ = result.v̇
    c.n_cache.noisy_vs = cat(c.n_cache.noisy_vs, noisy_vels, dims=2)
    c.n_cache.noisy_qs = cat(c.n_cache.noisy_qs, noisy_poses, dims=2)
    return noisy_poses, noisy_vels
end

function filter_velocity(state, c)
    filtered_velocity = moving_average_filter_velocity(5, c.n_cache.noisy_vs, velocity(state))
    c.f_cache.filtered_vs = cat(c.f_cache.filtered_vs, filtered_velocity, dims=2)
    filtered_vels = similar(velocity(state))
    filtered_vels[:] = filtered_velocity
end


function add_arm_noise!(noisy_poses, noisy_vels, joint_poses, last_noisy_joint_pose, dt)
    noisy_joint_poses = joint_poses + rand(arm_pos_noise_dist, length(joint_poses)) 
    # noisy_joint_poses = joint_poses
    # noisy_joint_poses[2:4] = joint_poses[2:4] + rand(arm_pos_noise_dist, 3)
    noisy_velocity = (noisy_joint_poses - last_noisy_joint_pose)./dt
    noisy_poses[8:end] .= noisy_joint_poses
    noisy_vels[7:end] .= noisy_velocity
end

function add_rotational_noise!(noisy_poses, noisy_vels, body_vels, last_R, dt, rand_walk)
    noisy_vels[1:3] .= body_vels[1:3] + rand(v_ang_vel_noise_dist, 3) + rand_walk[1:3]
    new_R = last_R*exp(skew(noisy_vels[1:3]...)*dt)
    new_noisy_R = Rotations.RotMatrix(SMatrix{3,3}(new_R))
    quat_new_noisy_R = Rotations.QuatRotation(new_noisy_R)
    noisy_poses[1:4] .= [quat_new_noisy_R.w, quat_new_noisy_R.x, quat_new_noisy_R.y, quat_new_noisy_R.z]
    return quat_new_noisy_R
end

function add_linear_noise!(noisy_poses, noisy_vels, v̇, last_v̇, last_body_vels, last_pos, last_R, new_R, dt, rand_walk)
    noisy_linear_accels = last_v̇[4:6] + rand(accel_noise_dist, 3) + rand_walk[4:6]
    noisy_vels[4:6] .= last_body_vels[4:6] + noisy_linear_accels.*dt #+last_v̇[4:6])./2 .*dt
    noisy_vels_in_space = last_R*SVector{3}(last_body_vels[4:6]) #+ new_R*SVector{3}(noisy_vels[4:6]))./2
    noisy_poses[5:7] .= last_pos + noisy_vels_in_space.*dt   
end

function moving_average_filter_velocity(filt_size, noisy_vs, act_vs)
    num_its = size(noisy_vs, 2)
    if num_its < filt_size
        filtered_vels = act_vs
    else
        filtered_vels = sum(noisy_vs[:,end-filt_size+1:end], dims=2) ./ filt_size  
    end
    # @show filtered_vels
    return filtered_vels
end