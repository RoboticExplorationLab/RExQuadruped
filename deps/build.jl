using CMake
using CxxWrap 

## Installing CxxWrap https://github.com/JuliaInterop/CxxWrap.jl

build_dir = joinpath(@__DIR__, "build")

if !isdir(build_dir)
    Base.Filesystem.mkdir(build_dir)
end 
unitreecxxlib_dir = joinpath(@__DIR__, "unitree_sdk_cxx")

# Sys.ARCH