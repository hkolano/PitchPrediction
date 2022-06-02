using CSV, Tables

sample_freq = 20. # Hz
wave_freq = 1. # Hz
time_span = 2
amp = rand(1)

t_seq = [0:1/sample_freq:2]
wave = amp.*sin.(t_seq*wave_freq)

# print(wave)

# tab = Tables.table(mat)
# labels = ["Var A", "Var B", "Var C"]
# CSV.write("data/waves/testdata.csv", tab, header=labels)