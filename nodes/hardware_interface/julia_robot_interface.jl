"""
caution! when including this module in julia,
and robot = RobotInterface() is called,
the robot will receive a initialize command and fall down in place

"""

module A1Robot
  using CxxWrap
  using StaticArrays
  using LinearAlgebra

  @wrapmodule(joinpath(@__DIR__, "../../deps/unitree_sdk_cxx/build/lib/libjulia_a1_interface.so"))
  function __init__()
    @initcxx
  end 
  
  """Set the motor position command in the unitree cxx interface. The 
  motor commands here should be in C indices"""
  # The pos here should be in C index 
  # send command with SendCommand(itf)
  function setPositionCommands(itf::RobotInterface, pos::AbstractVector{Float64}, Kp, Kd) 
    #setMotorCmd(int motor_id, double q, double dq, double Kp, double Kd, double tau)
    for i in 0:11
      setMotorCmd(itf,i, pos[i+1], 0.0, Kp, Kd, 0.0)
    end 
  end 

  """Set the motor torque command in the unitree cxx interface. The 
  motor commands here should be in C indices."""
  function setTorqueCommands(itf::RobotInterface, τ::AbstractVector{Float64})
    posStopF = 2.146e9 
    velStopF = 16000.0e0
    for i in 0:11
      setMotorCmd(itf, i, posStopF, velStopF, 0.0, 0.0, τ[i+1])
    end 
  end 


  """"Return four vectors that each contains the motor's 12 position, velocities,
  acceleration, and torque. All are arranged in the C indices"""
  function getMotorReadings(itf::RobotInterface)
    qs = @MVector zeros(12)
    dqs = @MVector zeros(12)
    ddqs = @MVector zeros(12)
    τs = @MVector zeros(12)
    for i in 0:11
      qs[i+1] = getMotorStateQ(itf,i)
      dqs[i+1] = getMotorStateDQ(itf, i)
      ddqs[i+1] = getMotorStateDDQ(itf, i)
      τs[i+1] = getMotorStateTau(itf,i)
    end 
    qs = SVector(qs)
    dqs = SVector(dqs)
    τs = SVector(τs)
    ddqs = SVector(ddqs)
    return (qs, dqs, ddqs, τs)
  end 

  """Return a Vector that contains the motor's 12 position. The position 
  readings here are in order of the C indices"""
  function getMotorPositions(itf::RobotInterface)
    qs = @MVector zeros(12)
    for i in 0:11
      qs[i+1] = getMotorStateQ(itf, i)
    end 
    qs = SVector(qs)
    return qs 
  end 

  """Return a vector that contains the motor's 12 velocities. Velocities 
  readings here are in order of the C indices"""
  function getMotorVelocities(itf::RobotInterface)
    vs = @MVector zeros(12)
    for i in 0:11
      vs[i+1] = getMotorStateDQ(itf, i)
    end 
    vs = SVector(vs)
    return vs
  end 

  """Return a vector that contains the 4 foot forces"""
  function getFootForces(itf::RobotInterface)
    fs = @MVector zeros(4)
    for i in 0:3
      fs[i+1] = getFootForce(itf, i)
    end 
    fs = SVector(fs)
    return fs 
  end 

end # end of the module


