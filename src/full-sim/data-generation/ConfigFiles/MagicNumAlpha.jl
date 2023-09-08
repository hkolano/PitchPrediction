# Include MagicNumBlueROV before this, to add on to the dictionary created there
# Center of Buoyancy vectors for the arm links
cob_vec_dict["shoulder"] = SVector{3, Float64}([-.001, -.003, .032])
cob_vec_dict["upperarm"] = SVector{3, Float64}([.073, 0.0, -.002])
cob_vec_dict["elbow"] = SVector{3, Float64}([.003, .001, -.017])
cob_vec_dict["wrist"] = SVector{3, Float64}([0.0, 0.0, -0.098])
cob_vec_dict["jaw"] = SVector{3, Float64}([0.0, 0.0, 0.0])
# TODO double check these numbers against the documentation
com_vec_dict["armbase"] = SVector{3, Float64}([-.075, -.006, -.003])

# Center of mass vectors for the arm links
com_vec_dict["shoulder"] = SVector{3, Float64}([0.005, -.001, 0.016])
com_vec_dict["upperarm"] = SVector{3, Float64}([.073, 0.0, 0.0])
com_vec_dict["elbow"] = SVector{3, Float64}([.017, -.026, -.002])
com_vec_dict["wrist"] = SVector{3, Float64}([0.0, 0.0, -.098])
com_vec_dict["jaw"] = SVector{3, Float64}([0.0, 0.0, 0.0])
com_vec_dict["armbase"] = SVector{3, Float64}([-.075, -.006, -.003])

# Calculate buoyancy forces
link_volumes = Dict("shoulder" =>   0.018, # volume in L
                    "upperarm" =>   0.203,
                    "elbow" =>      0.025,
                    "wrist" =>      0.155,
                    "armbase" =>    0.202,
                    "jaw" =>        0.02) # reasonable estimate 
# f = 997 (kg/m^3) * 9.81 (m/s^2) * V_in_L *.001 (m^3) = kg m / s^2
for (k,v) in link_volumes
    buoyancy_mag_dict[k] = v*rho*9.81*.001
end

# Calculate gravitational forces 
link_masses = Dict("shoulder" =>    0.194, 
                    "upperarm" =>   0.429,
                    "elbow" =>      0.115, 
                    "wrist" =>      0.333,
                    "armbase" =>    0.341,
                    "jaw" =>        0.05)
for (k,v) in link_masses
    grav_mag_dict[k] = v*9.81
end

# Drag forces on arm
# link_drags = Dict("shoulder" => [0.26 0.26 0.3]*rho, 
#                     "upperarm" => [0.3 1.6 1.6]*rho,
#                     "elbow" => [0.26 0.3 0.26]*rho,
#                     "wrist" => [1.8 1.8 0.3]*rho, 
#                     "jaw" => [.05, .05, .05]*rho)

link_drags = Dict("shoulder" => [0.26 0.26 0.3], 
                    "upperarm" => [0.3 1.6 1.6],
                    "elbow" => [0.26 0.3 0.26],
                    "wrist" => [1.8 1.8 0.3], 
                    "jaw" => [.05, .05, .05])


# Arm position joint limits
joint_lim_dict = Dict("base" => [-175*pi/180, 175*pi/180], 
                    "shoulder" => [0, 200*pi/180], 
                    "elbow" => [0, 200*pi/180], 
                    "wrist" => [-165*pi/180, 165*pi/180],
                    "jaw" => [0., 0.022])

θb = deg2rad(30)
θc = deg2rad(50)
vel_lim_dict = Dict("base" => [-θb, θb], 
                    "shoulder" => [-θb, θb], 
                    "elbow" => [-θb, θb], 
                    "wrist" => [-θc, θc],
                    "jaw" => [-.003, .003])
