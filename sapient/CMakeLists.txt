#
# SAPIENT 服务模块
#
include_directories(../../../usr/include)
include_directories(../../common)
include_directories(../../common/zlog)
include_directories(../protocol)
include_directories(../sapientpb)

# 收集源文件，排除 .pb.cc 文件（它们已经在 sapientpb 库中）
file(GLOB SAPIENT_SERVICE_SRCS "*.cpp")
# 如果还有其他非 .pb.cc 的源文件，可以在这里添加

add_library(sapient_service ${SAPIENT_SERVICE_SRCS})
target_link_libraries(sapient_service 
    sapientpb
    ${SRC_LIBS}
)
set(SAPIENT_SERVICE_LIBS sapient_service PARENT_SCOPE)
