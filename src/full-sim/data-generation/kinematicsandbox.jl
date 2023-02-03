function jacobian_transpose_ik!(state::MechanismState,
                                traj::trajParams,
                                c::CtlrCache,
                                t,
                                α=0.001)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    q = copy(configuration(state))

    ζ = get_zeta(traj, t, state, c)
    Δq = α*ζ
    q .=configuration(state) .+ Δq 
    set_configuration!(state, q)
end


function simplest_ik_transpose(state, des_σdot)
    J = calculate_actuated_jacobian(state)
    ζ = get_mp_pinv(J)*des_σdot
end

function skew(x1, x2, x3)
    output = [0 -x3 x2;
            x3 0 -x1;
            -x2 x1 0]
end

function simple_ik_iterator(state)
    des_σdot = [0., 0., 0., 1., 0., 0.]
    mechanism = state.mechanism
    qs = typeof(configuration(state))[]

    new_state = MechanismState(state.mechanism)
    reset_to_equilibrium!(new_state)
    new_state_qs = copy(configuration(new_state))
    prev_state_qs = copy(configuration(new_state))
   
    Δt = 0.01
    prev_ωb = [0., 0., 0.]
    prev_rot = Rotations.QuatRotation(new_state_qs[1:4])

    for t in range(0, stop=1, step=Δt)
        println("Current Config:")
        println(new_state.q)

        # Do inverse kinematics
        ζ = simplest_ik_transpose(new_state, des_σdot)
        println("Desired zeta:")
        println(ζ)

        des_twist_body = RigidBodyDynamics.Twist(body_frame, base_frame, body_frame, SVector{3}(ζ[1:3]), SVector{3}(ζ[4:6])) 
        des_twist_space = transform(des_twist_body, transform_to_root(new_state, vehicle_body))
        println(linear(des_twist_space))
        
        # Integrate rotation
        Ω = (skew(ζ[1:3]...) - skew(prev_ωb...))/2
        new_rot_mat = prev_rot*(I+(Δt/2)*Ω)*inv(I-(Δt/2)*Ω) # MP-R method from https://repozitorij.uni-lj.si/Dokument.php?id=85693&dn=
        new_rot = Rotations.QuatRotation(new_rot_mat)
        new_state_qs[1:4] = [new_rot.w, new_rot.x, new_rot.y, new_rot.z]

        # Integrate velocities 
        new_state_qs[5:7] = prev_state_qs[5:7]+Δt*linear(des_twist_space)

        # euler integrate joint poses 
        new_state_qs[8:end] = prev_state_qs[8:end] + Δt*vcat(ζ[7:end], 0.)
        set_configuration!(new_state, new_state_qs)
        push!(qs, copy(configuration(new_state)))

        prev_ωb = ζ[1:3]
        prev_rot = new_rot
        prev_state_qs = new_state_qs
    end

    ts = collect(range(0, stop=1, length=length(qs)))
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate=1.)
end


twist_body = RigidBodyDynamics.Twist(body_frame, base_frame, body_frame, SVector{3}(ζ[1:3]), SVector{3}(ζ[4:6])) 
twist_space = transform(twist, transform_to_root(new_state, vehicle_body))
