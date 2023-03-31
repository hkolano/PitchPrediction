# Initialize dictionaries for buoyancy forces
cob_vec_dict = Dict{String, SVector{3,Float64}}()
cob_vec_dict["vehicle"] = SVector{3, Float64}([0.0, 0.0, 0.02])
buoyancy_mag_dict = Dict{String, Float64}()
buoyancy_mag_dict["vehicle"] = 10.23*9.81 #(volume * gravity)
buoyancy_force_dict = Dict{String, FreeVector3D}()

# Initialize dictionary for gravitational forces
com_vec_dict = Dict{String, SVector{3, Float64}}()
com_vec_dict["vehicle"] = SVector{3, Float64}([0.0, 0.0, 0.0])
grav_mag_dict = Dict{String, Float64}()
grav_mag_dict["vehicle"] = 10.0*9.81 #(weight * gravity)
gravity_force_dict = Dict{String, FreeVector3D}()

# Drag coefficients for the vehicle
d_lin_coeffs = [4.03, 6.22, 5.18, .07, .07, .07]
d_nonlin_coeffs = [18.18, 21.66, 36.99, 1.55, 1.55, 1.55]
