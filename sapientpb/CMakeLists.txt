#
# SAPIENT Protocol Buffers 封装模块
#
include_directories(../../../usr/include)
include_directories(../sapient)
include_directories(../../common)
include_directories(../../common/zlog)
include_directories(../../common/cJSON)
include_directories(../../inc)
include_directories(../../service/platfrom/attitude)
include_directories(adapter)

# 添加 protobuf 库目录
link_directories(../../../usr/lib)

# 收集当前目录的源文件
aux_source_directory(. DIR_LIBS_SRCS)

# 收集 adapter 子目录的源文件
aux_source_directory(adapter ADAPTER_SRCS)

# 收集 protobuf 生成的 .pb.cc 文件（从 ../sapient 目录）
file(GLOB PROTOBUF_SRCS "../sapient/*.pb.cc")

# 合并所有源文件
set(ALL_SRCS ${DIR_LIBS_SRCS} ${ADAPTER_SRCS} ${PROTOBUF_SRCS})

add_library(sapientpb ${ALL_SRCS})
target_link_libraries(sapientpb 
    ${ROOT_DIR}/usr/lib/libprotobuf.a
    pthread
    config       # ConfigManager
    utils        # 工具库
    monitor      # CStatusMonit (如果是独立库)
)
set(SAPIENTPB_LIBS sapientpb PARENT_SCOPE)

