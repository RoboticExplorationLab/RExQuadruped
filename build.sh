JL_PLUGIN="$HOME/.julia/packages/ProtoBuf/h2AAj/plugin/protoc-gen-julia"

SRC_DIR="proto"
JL_BUILD_DIR="msgs"

MSG_NAME="imu_msg.proto"
# IMU MSG 
protoc -I=. --plugin=$JL_PLUGIN --proto_path=$SRC_DIR --julia_out=$JL_BUILD_DIR $MSG_NAME  # this only works with system level Julia installation 