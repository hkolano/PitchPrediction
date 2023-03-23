#=
Hydrodynamics calculator functions; called within SimWExt.jl 

=#
using RigidBodyDynamics

# ------------------------------------------------------------------------
#                HYDRODYNAMICS (Grav & Buoy) CALCULATOR
# ------------------------------------------------------------------------
function hydro_calc!(hydro_wrenches::Dict{BodyID, Wrench{Float64}}, t, state::MechanismState)
    buoy_wrenches = []
    grav_wrenches = []
    num_bodies = length(bodies(state.mechanism))-1
    # Iterate through each body 
    for body_name in body_names
        println("---------------------------")
        @show body_name
        println("---------------------------")
        # Get the body
        bod = body_dict[body_name]
        # Get default frame of the body
        body_default_frame = default_frame(bod)

         # -------- Calculate Buoyancy Wrench-------
        # Get transform between the defualt frame and the center of buoyancy
        # TODO: don't use fixed_transform because it's bad for computation time
        def_to_cob = fixed_transform(bod, body_default_frame, cob_frame_dict[body_name])
        # Transform buoyancy force vector to the body's default frame (rotation only)
        buoy_force_trans = transform(state, buoyancy_force_dict[body_name], body_default_frame)
        # Make the wrench: the buoyancy force through a point, the center of buoyancy.
        buoy_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_cob))), buoy_force_trans)
        push!(buoy_wrenches, buoy_wrench) 
        @show buoy_wrench    
        
        # -------- Calculate Gravity Wrench -------
        def_to_com = fixed_transform(bod, body_default_frame, com_frame_dict[body_name])
        grav_force_trans = transform(state, gravity_force_dict[body_name], body_default_frame)
        # Make the wrench: the buoyancy force through a point, the center of buoyancy.
        # COM = Point3D(body_default_frame, translation(inv(def_to_com)))
        grav_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_com))), grav_force_trans)
        # setelement!(mvis, COM)
        # Add wrench to buoy_wrenches
        push!(grav_wrenches, grav_wrench)
        @show grav_wrench

        wrench = buoy_wrench + grav_wrench

        if body_name == "vehicle"
            # ----- Grav/buoy for arm base link ----- 
            def_to_armbase_cob = fixed_transform(bod, body_default_frame, cob_frame_dict["armbase"])
            def_to_armbase_com = fixed_transform(bod, body_default_frame, com_frame_dict["armbase"])
            buoy_force_trans_armbase = transform(state, buoyancy_force_dict["armbase"], body_default_frame)
            grav_force_trans_armbase = transform(state, gravity_force_dict["armbase"], body_default_frame)
            buoy_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_cob))), buoy_force_trans_armbase)
            grav_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_com))), grav_force_trans_armbase)
            wrench = wrench + buoy_wrench_arm + grav_wrench_arm
            @show buoy_wrench_arm
            @show grav_wrench_arm
            
            # Drag on the vehicle 
            vel = velocity(state, joints(state.mechanism)[1])
            @show vel
            tau_d = -d_lin_coeffs .* vel .+ -d_nonlin_coeffs .* vel .* abs.(vel)
            drag_wrench = Wrench(body_default_frame, tau_d[1:3], tau_d[4:6])
            @show drag_wrench
            wrench = wrench + drag_wrench 
        else 
            twist_world = twist_wrt_world(state, bod)
            root_transform = transform_to_root(state, bod)
            # COB_point = Point3D(body_default_frame, translation(inv(def_to_cob)))
            twist_body = transform(twist_world, inv(root_transform))
            cob_vel = point_velocity(twist_body, Point3D(body_default_frame, translation(inv(def_to_cob))))
            @show cob_vel
            F_d = transpose(-link_drags[body_name]) .* abs.(cob_vel.v) .* cob_vel.v
            @show F_d
            drag_wrench_at_cob = Wrench(cob_frame_dict[body_name], [0., 0., 0.], [F_d[1], F_d[2], F_d[3]])
            @show drag_wrench_at_cob
            drag_wrench_at_default = transform(drag_wrench_at_cob, inv(def_to_cob))
            @show drag_wrench_at_default

            wrench = wrench + drag_wrench_at_default
        end
        hydro_wrenches[BodyID(bod)] = transform(state, wrench, root_frame(state.mechanism))
    end
    
end;