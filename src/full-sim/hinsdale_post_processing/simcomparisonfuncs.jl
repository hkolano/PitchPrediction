mutable struct delayedQuinticTrajParams
    p::quinticTrajParams    # trajparams object
    startbuffer::Float64     # start buffer time 
    endbuffer::Float64     # end buffer time
end

function calc_rpy(mocap_df)
    roll_vec = Vector{Float64}(undef, 0)
    pitch_vec = Vector{Float64}(undef, 0)
    yaw_vec = Vector{Float64}(undef, 0)

    for i in 1:nrow(mocap_df)
        r = QuatRotation([mocap_df[i, :w_ori], mocap_df[i, :x_ori], mocap_df[i, :y_ori], mocap_df[i, :z_ori]])
        euler = RotZYX(r)
        push!(roll_vec, euler.theta3)
        push!(pitch_vec, euler.theta2)
        push!(yaw_vec, euler.theta1)
    end

    mocap_df[!, "roll"] = roll_vec
    mocap_df[!, "pitch"] = pitch_vec
    mocap_df[!, "yaw"] = yaw_vec
    return mocap_df
end

function get_desv_at_t(t, delayed_p::delayedQuinticTrajParams)
    des_vel = zeros(num_velocities(mech_blue_alpha)-6)
    if t > delayed_p.startbuffer && t < delayed_p.endbuffer
        des_vel = get_desv_at_t(t-delayed_p.startbuffer, delayed_p.p)
    end
    return des_vel
end

function get_desq_at_t(t, delayed_p::delayedQuinticTrajParams)
    if t < delayed_p.startbuffer 
        des_q = delayed_p.p.wp.start.θs 
    elseif t > delayed_p.endbuffer
        des_q = delayed_p.p.wp.goal.θs 
    else
        des_q = get_desq_at_t(t-delayed_p.startbuffer, delayed_p.p)
    end

    return des_q[1:num_velocities(mech_blue_alpha)-6]
end