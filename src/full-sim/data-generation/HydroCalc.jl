# Hydrodynamics calculator functions 
using RigidBodyDynamics

# ------------------------------------------------------------------------
#                HYDRODYNAMICS (Grav & Buoy) CALCULATOR
# ------------------------------------------------------------------------
function hydro_calc!(hydro_wrenches::Dict{BodyID, Wrench{Float64}}, t, state::MechanismState)
    buoy_wrenches = []
    grav_wrenches = []
    names = ["cob1", "cob2", "cob3", "cob4", "cob5", "cob6"]
    num_bodies = length(bodies(state.mechanism))-1
    # Iterate through each body 
    for i in 1:num_bodies
        # Get the body
        bod = bodies(state.mechanism)[i+1]
        # Get default frame of the body
        body_default_frame = default_frame(bod)

        # -------- Calculate Buoyancy Wrench-------
        # Get transform between the defualt frame and the center of buoyancy
        # TODO: don't use fixed_transform because it's bad for computation time
        def_to_cob = fixed_transform(bod, body_default_frame, cob_frames[i])
        # Transform buoyancy force vector to the body's default frame (rotation only)
        # NAN on third iteration
        buoy_force_trans = transform(state, buoy_lin_forces[i], body_default_frame)
        # Make the wrench: the buoyancy force through a point, the center of buoyancy.
        buoy_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_cob))), buoy_force_trans)
        push!(buoy_wrenches, buoy_wrench)


        # -------- Calculate Gravity Wrench -------
        def_to_com = fixed_transform(bod, body_default_frame, com_frames[i])
        grav_force_trans = transform(state, grav_lin_forces[i], body_default_frame)
        # println(grav_force_trans)
        # Make the wrench: the buoyancy force through a point, the center of buoyancy.
        grav_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_com))), grav_force_trans)
        # Add wrench to buoy_wrenches
        push!(grav_wrenches, grav_wrench)

        # Add the buoyancy wrench and grav wrench together
        wrench = buoy_wrench + grav_wrench
        # println(wrench)
        # Visualize location of center of buoyancy
        # setelement!(mvis_alpha, Point3D(body_default_frame, translation(inv(def_to_cob))), 0.02, "name")

        # ----- Special calculaitons for the vehicle -----
        if i == 1
            # ----- Grav/buoy for arm base link ----- 
            def_to_armbase_cob = fixed_transform(bod, body_default_frame, cob_frames[end])
            def_to_armbase_com = fixed_transform(bod, body_default_frame, com_frames[end])
            buoy_force_trans_armbase = transform(state, buoy_lin_forces[end], body_default_frame)
            grav_force_trans_armbase = transform(state, grav_lin_forces[end], body_default_frame)
            buoy_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_cob))), buoy_force_trans_armbase)
            grav_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_com))), grav_force_trans_armbase)
            wrench = wrench + buoy_wrench_arm + grav_wrench_arm
            # println("armbase gravity wrench in vehicle frame")
            # println(grav_wrench_arm)
            # setelement!(mvis_alpha, Point3D(body_default_frame, translation(inv(def_to_armbase_com))), 0.02, "armbase_com")
            # println("Wrench without drag:")
            # println(wrench)

            # ----- Drag of the vehicle -----
            # NANs on second iteration
            vel=velocity(state, joints(state.mechanism)[1])
            # println(vel)
            d_lin_coeffs = [4.03, 6.22, 5.18, .07, .07, .07]
            d_nonlin_coeffs = [18.18, 21.66, 36.99, 1.55, 1.55, 1.55]
            tau_d = -d_lin_coeffs .* vel .+ -d_nonlin_coeffs .* vel .* abs.(vel)
            drag_wrench = Wrench(body_default_frame, tau_d[1:3], tau_d[4:6])  
            # println("Vehicle velocity is $(vel)")
            # println("Vehicle drag is $(drag_wrench)")
            
            wrench = wrench + drag_wrench
            # println("Wrench drag:")
            # println(drag_wrench)
        # ----- Drag on the links (quadratic only) ----
        elseif i < num_bodies
            twist_world = twist_wrt_world(state, bod)
            root_transform = transform_to_root(state, bod)
            # COB_point = Point3D(body_default_frame, translation(inv(def_to_cob)))
            twist_body = transform(twist_world, inv(root_transform))
            cob_vel = point_velocity(twist_body, Point3D(body_default_frame, translation(inv(def_to_cob))))
            # @show(i)
            F_d = transpose(-link_drag_coeffs[i-1]) .* abs.(cob_vel.v) .* cob_vel.v
            # println("Drag Force = $(F_d)")
            # println("Current Wrench = $(wrench)")
            # println([@printf(" %5.2f", x) for x in twist_body.linear])
            # println([@printf(" %5.2f", x) for x in F_d])
            # Wrench(frame, angular, linear)
            drag_wrench_at_cob = Wrench(cob_frames[i], [0.0, 0.0, 0.0], [F_d[1], F_d[2], F_d[3]])
            drag_wrench_at_default = transform(drag_wrench_at_cob, inv(def_to_cob))
            # if i == 3
            #     println("Point Velocity= $(cob_vel.v)")
            #     println("to be added: $(drag_wrench_at_default)")
            # end
            wrench = wrench + drag_wrench_at_default
        end
        # Transform the wrench to the root frame and assign it to the body
        hydro_wrenches[BodyID(bod)] = transform(state, wrench, root_frame(state.mechanism))
    end
    
end;


function simple_control!(torques::AbstractVector, t, state::MechanismState, hydro_wrenches::Dict{BodyID, Wrench{Float64}})
    # Get buoyancy buoyancy forces 
    # hydro_wrenches = Dict{BodyID, Wrench{Float64}}()
    # hydro_calc!(hydro_wrenches, t, state) 
    # Calculate inverse dynamics of alpha arm (including buoyancy)
    # tau = inverse_dynamics(state, des_acc, hydro_wrenches)
    # println(des_acc)

    # torques[velocity_range(state, vehicle_joint)] .= [0.0, 0.0, 0.0, 50.0, 0.0, 0.0]
    torques[velocity_range(state, vehicle_joint)] .= [0.0, 0.0, 0.0, 0.0, 0.0, 7.5]

    torques[velocity_range(state, base_joint)] .= -.1 .* velocity(state, base_joint)
    torques[velocity_range(state, shoulder_joint)] .= -.1 .* velocity(state, shoulder_joint)
    torques[velocity_range(state, elbow_joint)] .= -.1 .* velocity(state, elbow_joint)
    torques[velocity_range(state, wrist_joint)] .= -.01 .* velocity(state, wrist_joint)

    # torques[velocity_range(state, vehicle_joint)] = tau[vehicle_joint][1:6]
    # torques[velocity_range(state, base_joint)] .= tau[base_joint][1]
end;