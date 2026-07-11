cmake_minimum_required(VERSION 3.5.0)

# 交叉编译工具链
include(cross.cmake)

# 基础路径与项目名称配置
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)  
get_filename_component(PROJ_NAME ${PARENT_DIR} NAME)           # 取上级目录名作为项目名
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# 编译参数（龙芯LoongArch适配）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 通用编译优化与龙芯架构参数
set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# OpenCV依赖配置
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE: ${OpenCV_INCLUDE_DIRS}, OpenCV_LIBS: ${OpenCV_LIBS}")

# NCNN路径
# -------------------------------------------------
set(NCNN_ROOT /home/zhy/ncnn/build-loongarch/install)
include_directories(${NCNN_ROOT}/include)
link_directories(${NCNN_ROOT}/lib)

# 头文件路径汇总（拆分TFLM路径）
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    # 业务代码头文件
    ../user 
    ../code 
    ../model
    ../../libraries/zf_common 
    ../../libraries/zf_device 
    ../../libraries/zf_driver
    ../../libraries/zf_components/seekfree_assistant
    
    # TFLM核心头文件（完全拆分）
    ../../libraries/zf_components/tflm
    ../../libraries/zf_components/tflm/third_party/flatbuffers/include
    ../../libraries/zf_components/tflm/third_party/gemmlowp
    ../../libraries/zf_components/tflm/third_party/gemmlowp/fixedpoint
    ../../libraries/zf_components/tflm/third_party/ruy
    ../../libraries/zf_components/tflm/third_party/ruy/ruy
    ../../libraries/zf_components/tflm/tensorflow/lite/micro
    ../../libraries/zf_components/tflm/tensorflow/lite/micro/kernels
    ../../libraries/zf_components/tflm/tensorflow/lite/kernels/internal
    ../../libraries/zf_components/tflm/tensorflow/lite/core/api
    ../../libraries/zf_components/tflm/tensorflow/lite/core/c
    ../../libraries/zf_components/tflm/tensorflow/lite/schema
    ../../libraries/zf_components/ncnn/include/ncnn
)

# 静态库路径
link_directories(
    ../../libraries/zf_components/tflm/lib
    ../../libraries/zf_components/ncnn/lib
    ../../libraries/lq
)

# Explicit ownership list.  Application sources are intentionally not globbed:
# adding a layer must be an architectural decision rather than an accidental
# side effect of placing a .cpp file under project/code.
set(PRIMER_APP_SRCS
    ../user/main.cpp

    ../code/runtime/application.cpp
    ../code/runtime/hardware_composition.cpp
    ../code/runtime/lifecycle.cpp
    ../code/runtime/control_adapter.cpp
    ../code/runtime/estimation_adapter.cpp
    ../code/runtime/vision_stage_ports.cpp
    ../code/runtime/presentation_ports.cpp

    ../code/platform/device_platform.cpp
    ../code/control/pid_controller.cpp
    ../code/estimation/imu_estimator.cpp
    ../code/parameters/parameter_store.cpp

    ../code/vision/facts.cpp
    ../code/vision/preprocess.cpp
    ../code/vision/track/image_top.cpp
    ../code/vision/track/sidelines.cpp
    ../code/vision/track/corners.cpp
    ../code/vision/track/midline.cpp
    ../code/vision/track/straightness.cpp
    ../code/vision/track/geometry.cpp
    ../code/vision/scenes/roundabout.cpp
    ../code/vision/line_repair.cpp
    ../code/vision/steering.cpp
    ../code/vision/scenes/picture_scene.cpp
    ../code/vision/scenes/element_scenes.cpp
    ../code/vision/red_target.cpp
    ../code/runtime/service_composition.cpp
    ../code/vision/pipeline_init.cpp
    ../code/vision/pipeline.cpp

    ../code/presentation/show.cpp
    ../code/presentation/state.cpp
    ../code/presentation/input/key_input.cpp
    ../code/presentation/pages/oled_pages.cpp
    ../code/presentation/pages/red_debug_page.cpp
    ../code/presentation/pages/page3.cpp
    ../code/presentation/pages/page2.cpp
    ../code/presentation/pages/page1.cpp
    ../code/presentation/pages/page0.cpp
    ../code/inference/lq_ncnn.cpp
    ../code/transport/ww_transmission.cpp
)

set(PRIMER_VENDOR_SRCS
    ../../libraries/zf_common/zf_common_fifo.cpp
    ../../libraries/zf_common/zf_common_font.cpp
    ../../libraries/zf_common/zf_common_function.cpp

    ../../libraries/zf_device/zf_device_dl1x.cpp
    ../../libraries/zf_device/zf_device_imu.cpp
    ../../libraries/zf_device/zf_device_ips200_fb.cpp
    ../../libraries/zf_device/zf_device_tft180_fb.cpp
    ../../libraries/zf_device/zf_device_uvc.cpp

    ../../libraries/zf_driver/zf_driver_adc.cpp
    ../../libraries/zf_driver/zf_driver_delay.cpp
    ../../libraries/zf_driver/zf_driver_encoder.cpp
    ../../libraries/zf_driver/zf_driver_file_buffer.cpp
    ../../libraries/zf_driver/zf_driver_file_string.cpp
    ../../libraries/zf_driver/zf_driver_gpio.cpp
    ../../libraries/zf_driver/zf_driver_pit.cpp
    ../../libraries/zf_driver/zf_driver_pit_fd.cpp
    ../../libraries/zf_driver/zf_driver_pwm.cpp
    ../../libraries/zf_driver/zf_driver_tcp_client.cpp
    ../../libraries/zf_driver/zf_driver_udp.cpp

    ../../libraries/zf_components/seekfree_assistant/seekfree_assistant.cpp
    ../../libraries/zf_components/seekfree_assistant/seekfree_assistant_interface.cpp
)

# 生成可执行文件 & 配置属性
add_executable(${PROJ_NAME} ${PRIMER_APP_SRCS} ${PRIMER_VENDOR_SRCS})
set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

# 链接依赖库
target_link_libraries(
${PROJ_NAME} 
${OpenCV_LIBS}
tflm          # 链接TFLM静态库（必须）
pthread       # 线程库（TFLM内部使用）
m             # 数学库（算子计算依赖）
ncnn  gomp
liblq_camera_ex.a
)
