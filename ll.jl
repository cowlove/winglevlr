#!/usr/bin/env -S /home/jim/julia-1.6.3/bin/julia --sysimage  ./sys_ll.so 
#create_sysimage(["Plots","PyPlot"], sysimage_path="sys_ll.so", precompile_execution_file="ll.jl")

# 19.8 seconds even with PackageCompiler
# 11.2 seconds with pyplot()


using PackageCompiler 
using Plots
using DelimitedFiles

pyplot()
@time f=readdlm("./logs/AHRSD044.plog", comments=true, comment_char='#')

#default(size=(600,400))

@time plot(f[:,1], f[:,37])
@time plot!(f[:,1],f[:, 40], show=true)

print("WHAT")
sleep(5)



