# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct Vector3Msg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function Vector3Msg(; kwargs...)
        obj = new(meta(Vector3Msg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct Vector3Msg
const __meta_Vector3Msg = Ref{ProtoMeta}()
function meta(::Type{Vector3Msg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_Vector3Msg)
            __meta_Vector3Msg[] = target = ProtoMeta(Vector3Msg)
            allflds = Pair{Symbol,Union{Type,String}}[:x => Float64, :y => Float64, :z => Float64]
            meta(target, Vector3Msg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_Vector3Msg[]
    end
end
function Base.getproperty(obj::Vector3Msg, name::Symbol)
    if name === :x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct QuaternionMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function QuaternionMsg(; kwargs...)
        obj = new(meta(QuaternionMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct QuaternionMsg
const __meta_QuaternionMsg = Ref{ProtoMeta}()
function meta(::Type{QuaternionMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_QuaternionMsg)
            __meta_QuaternionMsg[] = target = ProtoMeta(QuaternionMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:w => Float64, :x => Float64, :y => Float64, :z => Float64]
            meta(target, QuaternionMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_QuaternionMsg[]
    end
end
function Base.getproperty(obj::QuaternionMsg, name::Symbol)
    if name === :w
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct EKFMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function EKFMsg(; kwargs...)
        obj = new(meta(EKFMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct EKFMsg
const __meta_EKFMsg = Ref{ProtoMeta}()
function meta(::Type{EKFMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_EKFMsg)
            __meta_EKFMsg[] = target = ProtoMeta(EKFMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:pos => Vector3Msg, :quat => QuaternionMsg, :v => Vector3Msg, :v_ang => Vector3Msg, :acc_bias => Vector3Msg, :v_ang_bias => Vector3Msg, :time => Float64]
            meta(target, EKFMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_EKFMsg[]
    end
end
function Base.getproperty(obj::EKFMsg, name::Symbol)
    if name === :pos
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :quat
        return (obj.__protobuf_jl_internal_values[name])::QuaternionMsg
    elseif name === :v
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :v_ang
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :acc_bias
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :v_ang_bias
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct LeggedEKFMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function LeggedEKFMsg(; kwargs...)
        obj = new(meta(LeggedEKFMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct LeggedEKFMsg
const __meta_LeggedEKFMsg = Ref{ProtoMeta}()
function meta(::Type{LeggedEKFMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_LeggedEKFMsg)
            __meta_LeggedEKFMsg[] = target = ProtoMeta(LeggedEKFMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:pos => Vector3Msg, :quat => QuaternionMsg, :v => Vector3Msg, :v_ang => Vector3Msg, :acc_bias => Vector3Msg, :v_ang_bias => Vector3Msg, :p1 => Vector3Msg, :p2 => Vector3Msg, :p3 => Vector3Msg, :p4 => Vector3Msg, :time => Float64]
            meta(target, LeggedEKFMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_LeggedEKFMsg[]
    end
end
function Base.getproperty(obj::LeggedEKFMsg, name::Symbol)
    if name === :pos
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :quat
        return (obj.__protobuf_jl_internal_values[name])::QuaternionMsg
    elseif name === :v
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :v_ang
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :acc_bias
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :v_ang_bias
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :p1
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :p2
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :p3
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :p4
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct ImuMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function ImuMsg(; kwargs...)
        obj = new(meta(ImuMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct ImuMsg
const __meta_ImuMsg = Ref{ProtoMeta}()
function meta(::Type{ImuMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_ImuMsg)
            __meta_ImuMsg[] = target = ProtoMeta(ImuMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:acc => Vector3Msg, :gyro => Vector3Msg, :time => Float64]
            meta(target, ImuMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_ImuMsg[]
    end
end
function Base.getproperty(obj::ImuMsg, name::Symbol)
    if name === :acc
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :gyro
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct CmdMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function CmdMsg(; kwargs...)
        obj = new(meta(CmdMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct CmdMsg
const __meta_CmdMsg = Ref{ProtoMeta}()
function meta(::Type{CmdMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_CmdMsg)
            __meta_CmdMsg[] = target = ProtoMeta(CmdMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:Kp => Float64, :Kd => Float64, :pos => Float64, :vel => Float64, :tau => Float64]
            meta(target, CmdMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_CmdMsg[]
    end
end
function Base.getproperty(obj::CmdMsg, name::Symbol)
    if name === :Kp
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :Kd
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :pos
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :vel
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :tau
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct MotorCmdMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function MotorCmdMsg(; kwargs...)
        obj = new(meta(MotorCmdMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct MotorCmdMsg
const __meta_MotorCmdMsg = Ref{ProtoMeta}()
function meta(::Type{MotorCmdMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_MotorCmdMsg)
            __meta_MotorCmdMsg[] = target = ProtoMeta(MotorCmdMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:FR_Hip => CmdMsg, :FR_Thigh => CmdMsg, :FR_Calf => CmdMsg, :FL_Hip => CmdMsg, :FL_Thigh => CmdMsg, :FL_Calf => CmdMsg, :RR_Hip => CmdMsg, :RR_Thigh => CmdMsg, :RR_Calf => CmdMsg, :RL_Hip => CmdMsg, :RL_Thigh => CmdMsg, :RL_Calf => CmdMsg, :time => Float64]
            meta(target, MotorCmdMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_MotorCmdMsg[]
    end
end
function Base.getproperty(obj::MotorCmdMsg, name::Symbol)
    if name === :FR_Hip
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :FR_Thigh
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :FR_Calf
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :FL_Hip
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :FL_Thigh
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :FL_Calf
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :RR_Hip
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :RR_Thigh
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :RR_Calf
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :RL_Hip
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :RL_Thigh
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :RL_Calf
        return (obj.__protobuf_jl_internal_values[name])::CmdMsg
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct JointMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function JointMsg(; kwargs...)
        obj = new(meta(JointMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct JointMsg
const __meta_JointMsg = Ref{ProtoMeta}()
function meta(::Type{JointMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_JointMsg)
            __meta_JointMsg[] = target = ProtoMeta(JointMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:FR_Hip => Float64, :FR_Thigh => Float64, :FR_Calf => Float64, :FL_Hip => Float64, :FL_Thigh => Float64, :FL_Calf => Float64, :RR_Hip => Float64, :RR_Thigh => Float64, :RR_Calf => Float64, :RL_Hip => Float64, :RL_Thigh => Float64, :RL_Calf => Float64]
            meta(target, JointMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_JointMsg[]
    end
end
function Base.getproperty(obj::JointMsg, name::Symbol)
    if name === :FR_Hip
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :FR_Thigh
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :FR_Calf
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :FL_Hip
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :FL_Thigh
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :FL_Calf
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RR_Hip
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RR_Thigh
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RR_Calf
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RL_Hip
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RL_Thigh
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RL_Calf
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct JointSensorsMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function JointSensorsMsg(; kwargs...)
        obj = new(meta(JointSensorsMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct JointSensorsMsg
const __meta_JointSensorsMsg = Ref{ProtoMeta}()
function meta(::Type{JointSensorsMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_JointSensorsMsg)
            __meta_JointSensorsMsg[] = target = ProtoMeta(JointSensorsMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:torques => JointMsg, :positions => JointMsg, :velocities => JointMsg, :FR_foot => Float64, :FL_foot => Float64, :RR_foot => Float64, :RL_foot => Float64, :time => Float64]
            meta(target, JointSensorsMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_JointSensorsMsg[]
    end
end
function Base.getproperty(obj::JointSensorsMsg, name::Symbol)
    if name === :torques
        return (obj.__protobuf_jl_internal_values[name])::JointMsg
    elseif name === :positions
        return (obj.__protobuf_jl_internal_values[name])::JointMsg
    elseif name === :velocities
        return (obj.__protobuf_jl_internal_values[name])::JointMsg
    elseif name === :FR_foot
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :FL_foot
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RR_foot
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :RL_foot
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

mutable struct ViconMsg <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function ViconMsg(; kwargs...)
        obj = new(meta(ViconMsg), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct ViconMsg
const __meta_ViconMsg = Ref{ProtoMeta}()
function meta(::Type{ViconMsg})
    ProtoBuf.metalock() do
        if !isassigned(__meta_ViconMsg)
            __meta_ViconMsg[] = target = ProtoMeta(ViconMsg)
            allflds = Pair{Symbol,Union{Type,String}}[:pos => Vector3Msg, :quat => QuaternionMsg, :time => Float64]
            meta(target, ViconMsg, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_ViconMsg[]
    end
end
function Base.getproperty(obj::ViconMsg, name::Symbol)
    if name === :pos
        return (obj.__protobuf_jl_internal_values[name])::Vector3Msg
    elseif name === :quat
        return (obj.__protobuf_jl_internal_values[name])::QuaternionMsg
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

export Vector3Msg, QuaternionMsg, EKFMsg, LeggedEKFMsg, ImuMsg, MotorCmdMsg, CmdMsg, JointSensorsMsg, JointMsg, ViconMsg
