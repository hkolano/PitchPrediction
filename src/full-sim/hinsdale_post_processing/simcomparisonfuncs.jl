mutable struct delayedQuinticTrajParams
    p::quinticTrajParams    # trajparams object
    startbuffer::Float64     # start buffer time 
    endbuffer::Float64     # end buffer time
end

function get_desv_at_t(t, delayed_p::delayedQuinticTrajParams)
    des_vel = zeros(num_velocities(mech_blue_alpha)-6)
    if t > delayed_p.startbuffer && t < delayed_p.endbuffer
        des_vel = get_desv_at_t(t-delayed_p.startbuffer, delayed_p.p)
    end
    return des_vel
end