using CSV, Tables, Plots

sample_freq = 100. # Hz
time_span = 2
t_seq = [n for n=0:1/sample_freq:2]

wave(a, f, t) = a.*sin.(t.*f)

function gen_simple_wave(t::Vector{Float64})
    wave_freq = rand(1)*10*2π
    amp = rand(1)
    return wave(amp, wave_freq, t)
end

function generate_wave_file(i, t=t_seq, time=time_span)
    # Initialize data array
    num_rows = length(t)
    data = Array{Float64}(undef, num_rows, 2)

    # First column (time): 
    data[:,1] = t 
    # Second column (wave):
    data[:,2] = gen_simple_wave(t)
    labels = ["time", "wave"]

    tab = Tables.table(data)
    CSV.write("data/waves/simplewave$(i).csv", tab, header=labels)
end

for i = 1:10
    generate_wave_file(i)
end

# print(wave)

# tab = Tables.table(mat)
# labels = ["Var A", "Var B", "Var C"]
# CSV.write("data/waves/testdata.csv", tab, header=labels)