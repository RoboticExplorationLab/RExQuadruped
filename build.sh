JL_PLUGIN="$HOME/.julia/packages/ProtoBuf/h2AAj/plugin/protoc-gen-julia"

SRC_DIR="proto"
MSG_BUILD_DIR="msgs"

MSG_NAME="messages.proto"
# Build all messages  
protoc -I=. --plugin=$JL_PLUGIN --proto_path=$SRC_DIR --julia_out=$MSG_BUILD_DIR $MSG_NAME
protoc -I=. --python_out=$MSG_BUILD_DIR --proto_path=$SRC_DIR $MSG_NAME
