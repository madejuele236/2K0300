cmake_minimum_required(VERSION 3.5.0)

include(cross.cmake)

set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(NEW_DIR ${CURRENT_DIR}/.. ABSOLUTE)
get_filename_component(REPO_ROOT ${NEW_DIR}/.. ABSOLUTE)
set(TRUE_VENDOR_ROOT ${REPO_ROOT}/true_LS2K0300_Library/Seekfree_LS2K0300_Opensource_Library)
set(LS2K_LIB_ROOT ${TRUE_VENDOR_ROOT}/libraries)
get_filename_component(PROJ_NAME ${NEW_DIR} NAME)
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

option(LS2K_PERF_ENABLED "Enable fixed-stage runtime performance counters" ON)
option(LS2K_PERF_USE_CYCLE_COUNTER "Use LoongArch cycle counter for performance counters when available" ON)

set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build")
find_package(OpenCV REQUIRED)

add_library(ls2k_project_headers INTERFACE)
target_include_directories(ls2k_project_headers INTERFACE
    ${OpenCV_INCLUDE_DIRS}
    ../user
    ../code
    ../code/port
    ../code/platform
    ../code/platform/true_ls2k0300
    ../code/runtime
)

add_library(vendor_tflm STATIC IMPORTED GLOBAL)
set_target_properties(vendor_tflm PROPERTIES
    IMPORTED_LOCATION ${LS2K_LIB_ROOT}/zf_components/tflm/lib/libtflm.a
    INTERFACE_INCLUDE_DIRECTORIES "${LS2K_LIB_ROOT}/zf_components/tflm;${LS2K_LIB_ROOT}/zf_components/tflm/third_party/flatbuffers/include;${LS2K_LIB_ROOT}/zf_components/tflm/third_party/gemmlowp;${LS2K_LIB_ROOT}/zf_components/tflm/third_party/gemmlowp/fixedpoint;${LS2K_LIB_ROOT}/zf_components/tflm/third_party/ruy;${LS2K_LIB_ROOT}/zf_components/tflm/third_party/ruy/ruy;${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/micro;${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/micro/kernels;${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/kernels/internal;${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/core/api;${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/core/c;${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/schema"
)

add_library(vendor_ncnn STATIC IMPORTED GLOBAL)
set_target_properties(vendor_ncnn PROPERTIES
    IMPORTED_LOCATION ${LS2K_LIB_ROOT}/zf_components/ncnn/lib/libncnn.a
    INTERFACE_INCLUDE_DIRECTORIES ${LS2K_LIB_ROOT}/zf_components/ncnn/include
)

add_library(ls2k_linux_io STATIC
    ../code/platform/linux/linux_io.cpp
)
target_link_libraries(ls2k_linux_io PUBLIC ls2k_project_headers)

add_library(ls2k_platform_timer STATIC
    ../code/platform/true_ls2k0300/timer_bridge.cpp
)
target_link_libraries(ls2k_platform_timer PUBLIC ls2k_project_headers ls2k_linux_io pthread)

add_library(ls2k_platform_adc STATIC
    ../code/platform/true_ls2k0300/adc_bridge.cpp
    ../code/platform/power_adapter.cpp
)
target_link_libraries(ls2k_platform_adc PUBLIC ls2k_project_headers ls2k_linux_io)

add_library(ls2k_platform_encoder STATIC
    ../code/platform/true_ls2k0300/encoder_bridge.cpp
    ../code/platform/encoder_adapter.cpp
)
target_link_libraries(ls2k_platform_encoder PUBLIC ls2k_project_headers ls2k_linux_io)

add_library(ls2k_platform_imu STATIC
    ../code/platform/true_ls2k0300/imu_bridge.cpp
    ../code/platform/imu_adapter.cpp
)
target_link_libraries(ls2k_platform_imu PUBLIC ls2k_project_headers ls2k_linux_io pthread)

add_library(ls2k_platform_camera STATIC
    ../code/platform/true_ls2k0300/camera_bridge.cpp
    ../code/platform/camera_adapter.cpp
    ../code/platform/camera_frame_source.cpp
)
target_link_libraries(ls2k_platform_camera PUBLIC ls2k_project_headers ls2k_linux_io)

add_library(ls2k_platform_motor STATIC
    ../code/platform/true_ls2k0300/motor_bridge.cpp
    ../code/platform/actuator_adapter.cpp
)
target_link_libraries(ls2k_platform_motor PUBLIC ls2k_project_headers ls2k_linux_io)

add_library(ls2k_assistant_protocol STATIC
    ../code/transport/assistant_protocol.cpp
)
target_link_libraries(ls2k_assistant_protocol PUBLIC ls2k_project_headers)

add_library(ls2k_assistant_transport STATIC
    ../code/transport/assistant_link.cpp
    ../code/platform/true_ls2k0300/assistant_bridge.cpp
)
target_link_libraries(ls2k_assistant_transport PUBLIC ls2k_project_headers ls2k_assistant_protocol pthread)

find_program(PYTHON3_EXECUTABLE NAMES python3)
if(NOT PYTHON3_EXECUTABLE)
    message(FATAL_ERROR "python3 is required to generate ML classifier artifacts")
endif()

set(LS2K_ML_CLASSIFIER "V9" CACHE STRING "Compile exactly one ML classifier: V9 or TFLITE_INT8")
set_property(CACHE LS2K_ML_CLASSIFIER PROPERTY STRINGS V9 TFLITE_INT8)
set(ML_CLASSIFIER_COMMON_SOURCES
    ../code/vision/ml/ml_class_mapping.cpp
    ../code/vision/ml/selected_ml_classifier.cpp)
set(ML_CLASSIFIER_LINK_LIBS)

if(LS2K_ML_CLASSIFIER STREQUAL "V9")
    add_definitions(-DLS2K_ML_CLASSIFIER_BACKEND=1)
    set(V9_BUNDLE_DIR
        ${REPO_ROOT}/model_training/experiments/v9_binary_template_replay_20260522_0001/current/pruned_firmware_bundles/dct8_sign_absmedian__duplicate_code_merge)
    set(ML_CLASSIFIER_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/v9)
    set(V9_GENERATED_HEADER ${ML_CLASSIFIER_GENERATED_DIR}/generated_v9_artifact.hpp)
    set(V9_GENERATED_SOURCE ${ML_CLASSIFIER_GENERATED_DIR}/generated_v9_artifact.cpp)
    add_custom_command(
        OUTPUT ${V9_GENERATED_HEADER} ${V9_GENERATED_SOURCE}
        COMMAND ${PYTHON3_EXECUTABLE}
                ${REPO_ROOT}/model_training/generate_v9_firmware_artifact.py
                ${V9_BUNDLE_DIR}
                ${V9_GENERATED_HEADER}
                ${V9_GENERATED_SOURCE}
        DEPENDS
            ${REPO_ROOT}/model_training/generate_v9_firmware_artifact.py
            ${V9_BUNDLE_DIR}/manifest.json
            ${V9_BUNDLE_DIR}/template_codes_packed_u8.bin
            ${V9_BUNDLE_DIR}/template_parent_u8.bin
        VERBATIM)
    set(ML_CLASSIFIER_SOURCES
        ${ML_CLASSIFIER_COMMON_SOURCES}
        ../code/vision/ml/selected_ml_classifier_v9.cpp
        ../code/vision/ml/v9_descriptor.cpp
        ../code/vision/ml/v9_replay.cpp
        ${V9_GENERATED_SOURCE})
elseif(LS2K_ML_CLASSIFIER STREQUAL "TFLITE_INT8")
    add_definitions(-DLS2K_ML_CLASSIFIER_BACKEND=2)
    set(TFLITE_CLASSIFIER_MODEL
        ${REPO_ROOT}/model_training/experiments/v8_parent_c2612_d4_multiteacher_20260520_0001/s2d_c2_6_12_d4_shadow_init/parent_int8.tflite)
    set(TFLITE_CLASSIFIER_SHA256 0362178a0f665a3bf9dde16d4f6d9883d06370f2559824ef2d723b32002600cf)
    set(TFLITE_IDENTITY_PARAMS
        ${REPO_ROOT}/model_training/experiments/v8_input_canonicalization_20260522_0001/d4best_recompile_probe/identity_best_params.npz)
    set(TFLITE_IDENTITY_PARAMS_SHA256 5ac2827bfc8aa37f928802bc93f1274879e7ab73c97f8da534424094d8a701c5)
    set(ML_CLASSIFIER_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/tflite_classifier)
    set(TFLITE_GENERATED_HEADER ${ML_CLASSIFIER_GENERATED_DIR}/generated_tflite_classifier_artifact.hpp)
    set(TFLITE_GENERATED_SOURCE ${ML_CLASSIFIER_GENERATED_DIR}/generated_tflite_classifier_artifact.cpp)
    add_custom_command(
        OUTPUT ${TFLITE_GENERATED_HEADER} ${TFLITE_GENERATED_SOURCE}
        COMMAND ${PYTHON3_EXECUTABLE}
                ${REPO_ROOT}/model_training/generate_tflite_classifier_artifact.py
                ${TFLITE_CLASSIFIER_MODEL}
                ${TFLITE_IDENTITY_PARAMS}
                ${TFLITE_GENERATED_HEADER}
                ${TFLITE_GENERATED_SOURCE}
                ${TFLITE_CLASSIFIER_SHA256}
                ${TFLITE_IDENTITY_PARAMS_SHA256}
        DEPENDS
            ${REPO_ROOT}/model_training/generate_tflite_classifier_artifact.py
            ${TFLITE_CLASSIFIER_MODEL}
            ${TFLITE_IDENTITY_PARAMS}
        VERBATIM)
    set(ML_CLASSIFIER_SOURCES
        ${ML_CLASSIFIER_COMMON_SOURCES}
        ../code/vision/ml/selected_ml_classifier_tflite.cpp
        ${TFLITE_GENERATED_SOURCE}
        ${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/micro/kernels/unused/reduce_common.cc
        ${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/micro/kernels/unused/reduce.cc
        ${LS2K_LIB_ROOT}/zf_components/tflm/tensorflow/lite/micro/kernels/unused/space_to_depth.cc)
    set(ML_CLASSIFIER_LINK_LIBS vendor_tflm)
else()
    message(FATAL_ERROR "LS2K_ML_CLASSIFIER must be V9 or TFLITE_INT8")
endif()

set(NEW_SRCS
    ../user/main.cpp
    ../code/platform/bootstrap.cpp
    ../code/platform/param_store.cpp
    ../code/platform/hardware_profile.cpp
    ../code/transport/steering_media_link.cpp
    ../code/transport/steering_media_protocol.cpp
    ../code/port/perf_counter.cpp
    ../code/port/thread_scheduling.cpp
    ../code/platform/true_ls2k0300/steering_media_bridge.cpp
    ../code/control/steering_yaw_controller.cpp
    ../code/control/actuator_command_builder.cpp
    ../code/control/wheel_target_mixer.cpp
    ../code/control/wheel_pid.cpp
    ../code/vision/image/luma_sampler.cpp
    ../code/vision/image/color_sampler.cpp
    ../code/vision/ml/red_rectangle_detector.cpp
    ../code/vision/ml/roi_sampler.cpp
    ${ML_CLASSIFIER_SOURCES}
    ../code/vision/ml/ml_path_generator.cpp
    ../code/vision/ml/ml_reference_adapter.cpp
    ../code/vision/ml/ml_inertial_path_tracker.cpp
    ../code/vision/ml/ml_scene.cpp
    ../code/vision/bev/bev_projector.cpp
    ../code/vision/bev/bev_image_segment_connectivity.cpp
    ../code/vision/bev/single_boundary_offset.cpp
    ../code/vision/bev/bev_boundary_row.cpp
    ../code/vision/bev/bev_sample_projection_lut.cpp
    ../code/vision/bev/bev_sparse_row_scanner.cpp
    ../code/vision/bev/bev_reference_path_builder.cpp
    ../code/vision/bev/bev_simple_perception.cpp
    ../code/vision/elements/cross_exit_element_evidence.cpp
    ../code/vision/elements/visual_element_evidence.cpp
    ../code/vision/elements/visual_element_pipeline.cpp
    ../code/reference/visual_reference_orchestration.cpp
    ../code/reference/reference_usability.cpp
    ../code/reference/reference_continuity.cpp
    ../code/reference/reference_lateral_error.cpp
    ../code/reference/reference_tracking_geometry.cpp
    ../code/reference/reference_control_readiness.cpp
    ../code/runtime/lifecycle/startup.cpp
    ../code/runtime/capture/camera_frame_store.cpp
    ../code/runtime/capture/camera_capture_worker.cpp
    ../code/vision/elements/circle_v2/circle_v2_scene.cpp
    ../code/vision/elements/circle_v2/circle_v2_reference_adapter.cpp
    ../code/vision/elements/circle_v2/detail/circle_v2_event_observer.cpp
    ../code/vision/elements/circle_v2/detail/circle_v2_expansion_observer.cpp
    ../code/vision/elements/circle_v2/detail/circle_v2_reducer.cpp
    ../code/vision/elements/circle_v2/detail/circle_v2_geometry_observer.cpp
    ../code/vision/elements/circle_v2/detail/circle_v2_composer.cpp
    ../code/runtime/pipelines/steering_frame_pipeline.cpp
    ../code/runtime/perception_frontend.cpp
    ../code/safety/low_voltage_sampler.cpp
    ../code/control/motion_supervisor.cpp
    ../code/safety/control_gate.cpp
    ../code/safety/control_apply_observation.cpp
    ../code/estimation/vehicle_pose_delta_estimator.cpp
    ../code/reference/reference_time_alignment.cpp
    ../code/runtime/loops/control_loop.cpp
    ../code/observability/control_debug_reporter.cpp
    ../code/runtime/services/assistant_service.cpp
    ../code/runtime/services/steering_media_service.cpp
    ../code/control/tuning_state.cpp
    ../code/runtime/lifecycle/shutdown.cpp
)

add_executable(
    ${PROJ_NAME}
    ${NEW_SRCS}
)
target_include_directories(${PROJ_NAME} PRIVATE ${ML_CLASSIFIER_GENERATED_DIR})
target_compile_definitions(${PROJ_NAME} PRIVATE
    LS2K_PERF_ENABLED=$<BOOL:${LS2K_PERF_ENABLED}>
    LS2K_PERF_USE_CYCLE_COUNTER=$<BOOL:${LS2K_PERF_USE_CYCLE_COUNTER}>
)
set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

target_link_libraries(
    ${PROJ_NAME}
    ls2k_project_headers
    ls2k_platform_timer
    ls2k_platform_adc
    ls2k_platform_encoder
    ls2k_platform_imu
    ls2k_platform_camera
    ls2k_platform_motor
    ls2k_assistant_transport
    ${ML_CLASSIFIER_LINK_LIBS}
    ${OpenCV_LIBS}
    pthread
    m
)

add_executable(
    ml_yuyv_capture
    ../user/ml_yuyv_capture.cpp
)
target_link_libraries(
    ml_yuyv_capture
    ls2k_project_headers
    ls2k_platform_camera
    pthread
    m
)

if(LS2K_ML_CLASSIFIER STREQUAL "V9")
    add_executable(
        ml_v9_board_probe
        ../user/ml_v9_board_probe.cpp
        ../code/vision/image/color_sampler.cpp
        ../code/vision/ml/red_rectangle_detector.cpp
        ../code/vision/ml/roi_sampler.cpp
        ../code/vision/ml/v9_descriptor.cpp
        ../code/vision/ml/v9_replay.cpp
        ../code/vision/bev/bev_projector.cpp
        ${V9_GENERATED_SOURCE})
    target_include_directories(ml_v9_board_probe PRIVATE ${ML_CLASSIFIER_GENERATED_DIR})
    target_link_libraries(
        ml_v9_board_probe
        ls2k_project_headers
        ls2k_platform_camera
        ${OpenCV_LIBS}
        pthread
        m)
endif()

add_executable(
    ml_classifier_board_probe
    ../user/ml_classifier_board_probe.cpp
    ${ML_CLASSIFIER_SOURCES})
target_include_directories(ml_classifier_board_probe PRIVATE ${ML_CLASSIFIER_GENERATED_DIR})
target_link_libraries(
    ml_classifier_board_probe
    ls2k_project_headers
    ${ML_CLASSIFIER_LINK_LIBS}
    pthread
    m)

option(LS2K_BUILD_VERIFICATION_HELPERS "Build verification helper binaries" OFF)
option(LS2K_TFLM_OP_BENCH "Build isolated TFLM operator benchmark binary" OFF)
set(LS2K_TFLM_LIB_PATH "${LS2K_LIB_ROOT}/zf_components/tflm/lib/libtflm.a" CACHE FILEPATH "TFLM static library for benchmark builds")
set(LS2K_TFLM_OP_BENCH_MODELS_HEADER "benchmark_models_empty.h" CACHE STRING "Header containing benchmark model assets")

if(LS2K_BUILD_VERIFICATION_HELPERS)
    add_executable(
        bev_simple_perception_test
        ../verification/tests/bev_simple_perception_test.cpp
        ../code/vision/bev/bev_projector.cpp
        ../code/vision/bev/bev_image_segment_connectivity.cpp
        ../code/vision/bev/single_boundary_offset.cpp
        ../code/vision/bev/bev_boundary_row.cpp
        ../code/vision/bev/bev_sample_projection_lut.cpp
        ../code/vision/bev/bev_sparse_row_scanner.cpp
        ../code/vision/bev/bev_reference_path_builder.cpp
        ../code/vision/bev/bev_simple_perception.cpp
        ../code/vision/image/luma_sampler.cpp
        ../code/vision/elements/visual_element_evidence.cpp
        ../code/vision/elements/cross_exit_element_evidence.cpp
        ../code/vision/elements/visual_element_pipeline.cpp
        ../code/reference/visual_reference_orchestration.cpp
        ../code/reference/reference_usability.cpp
        ../code/reference/reference_continuity.cpp
        ../code/port/perf_counter.cpp
    )
    target_link_libraries(
        bev_simple_perception_test
        pthread
        m
    )

    add_executable(
        reference_usability_lateral_error_test
        ../verification/tests/reference_usability_lateral_error_test.cpp
        ../code/reference/reference_usability.cpp
        ../code/reference/reference_lateral_error.cpp
        ../code/reference/reference_tracking_geometry.cpp
        ../code/reference/reference_control_readiness.cpp
        ../code/port/perf_counter.cpp
        ../code/control/steering_yaw_controller.cpp
        ../code/safety/control_gate.cpp
        ../code/safety/control_apply_observation.cpp
    )
    target_link_libraries(
        reference_usability_lateral_error_test
        pthread
        m
    )

    add_executable(
        reference_tracking_geometry_test
        ../verification/tests/reference_tracking_geometry_test.cpp
        ../code/reference/reference_usability.cpp
        ../code/reference/reference_tracking_geometry.cpp
    )
    target_link_libraries(
        reference_tracking_geometry_test
        pthread
        m
    )

    add_executable(
        visual_reference_orchestration_test
        ../verification/tests/visual_reference_orchestration_test.cpp
        ../code/reference/visual_reference_orchestration.cpp
    )
    target_link_libraries(
        visual_reference_orchestration_test
        pthread
        m
    )

    add_executable(
        cross_exit_element_evidence_test
        ../verification/tests/cross_exit_element_evidence_test.cpp
        ../code/vision/elements/cross_exit_element_evidence.cpp
    )
    target_link_libraries(
        cross_exit_element_evidence_test
        pthread
        m
    )

    add_executable(
        steering_circle_v2_scene_test
        ../verification/tests/steering_circle_v2_scene_test.cpp
        ../code/vision/elements/circle_v2/circle_v2_scene.cpp
        ../code/vision/elements/circle_v2/circle_v2_reference_adapter.cpp
        ../code/vision/elements/circle_v2/detail/circle_v2_event_observer.cpp
        ../code/vision/elements/circle_v2/detail/circle_v2_expansion_observer.cpp
        ../code/vision/elements/circle_v2/detail/circle_v2_reducer.cpp
        ../code/vision/elements/circle_v2/detail/circle_v2_geometry_observer.cpp
        ../code/vision/elements/circle_v2/detail/circle_v2_composer.cpp
        ../code/vision/bev/single_boundary_offset.cpp
    )
    target_link_libraries(
        steering_circle_v2_scene_test
        pthread
        m
    )

    add_executable(
        assistant_telemetry_selftest
        ../verification/tests/assistant_telemetry_selftest.cpp
        ../code/transport/assistant_protocol.cpp
        ../code/safety/control_apply_observation.cpp
        ../code/control/motion_supervisor.cpp
    )
    target_link_libraries(
        assistant_telemetry_selftest
        ${OpenCV_LIBS}
        pthread
        m
    )

    add_executable(
        param_store_load_runtime_parameters_test
        ../verification/tests/param_store_load_runtime_parameters_test.cpp
        ../code/platform/param_store.cpp
    )
    target_link_libraries(
        param_store_load_runtime_parameters_test
        ${OpenCV_LIBS}
        pthread
        m
    )

    add_executable(
        power_adapter_threshold_test
        ../verification/tests/power_adapter_threshold_test.cpp
        ../code/platform/power_adapter.cpp
        ../code/platform/true_ls2k0300/adc_bridge.cpp
    )
    target_link_libraries(
        power_adapter_threshold_test
        pthread
        m
    )

    add_executable(
        startup_low_voltage_order_test
        ../verification/tests/startup_low_voltage_order_test.cpp
        ../code/runtime/lifecycle/startup.cpp
        ../code/platform/hardware_profile.cpp
    )
    target_link_libraries(
        startup_low_voltage_order_test
        pthread
        m
    )

    add_executable(
        steering_media_selftest
        ../user/steering_media_selftest.cpp
        ../code/transport/steering_media_link.cpp
        ../code/transport/steering_media_protocol.cpp
        ../code/platform/true_ls2k0300/steering_media_bridge.cpp
        ../code/observability/control_debug_reporter.cpp
        ../code/safety/control_gate.cpp
        ../code/safety/control_apply_observation.cpp
        ../code/control/motion_supervisor.cpp
        ../code/runtime/capture/camera_frame_store.cpp
        ../code/runtime/services/steering_media_service.cpp
        ../code/vision/image/luma_sampler.cpp
        ../code/port/perf_counter.cpp
    )
    target_link_libraries(
        steering_media_selftest
        pthread
        m
    )

    add_executable(
        camera_device_board_test
        ../verification/tests/camera_device_board_test.cpp
    )
    target_link_libraries(
        camera_device_board_test
        ls2k_project_headers
        ls2k_platform_camera
        pthread
        m
    )

    add_executable(
        timer_device_board_test
        ../verification/tests/timer_device_board_test.cpp
        ../code/port/thread_scheduling.cpp
    )
    target_link_libraries(
        timer_device_board_test
        ls2k_project_headers
        ls2k_platform_timer
        pthread
        m
    )

    add_executable(
        motor_device_board_test
        ../verification/tests/motor_device_board_test.cpp
    )
    target_link_libraries(
        motor_device_board_test
        ls2k_project_headers
        ls2k_platform_adc
        ls2k_platform_encoder
        ls2k_platform_motor
        pthread
        m
    )

    add_executable(
        encoder_pair_board_test
        ../verification/tests/encoder_pair_board_test.cpp
    )
    target_link_libraries(
        encoder_pair_board_test
        ls2k_project_headers
        ls2k_platform_encoder
        pthread
        m
    )

    add_executable(
        assistant_bridge_board_test
        ../verification/tests/assistant_bridge_contract_test.cpp
    )
    target_link_libraries(
        assistant_bridge_board_test
        ls2k_project_headers
        ls2k_assistant_transport
        pthread
        m
    )
    set_target_properties(assistant_bridge_board_test PROPERTIES
        LINK_FLAGS "-Wl,--wrap=send"
    )

    foreach(verification_target
            bev_simple_perception_test
            reference_usability_lateral_error_test
            reference_tracking_geometry_test
            visual_reference_orchestration_test
            cross_exit_element_evidence_test
            steering_circle_v2_scene_test
            assistant_telemetry_selftest
            param_store_load_runtime_parameters_test
            power_adapter_threshold_test
            startup_low_voltage_order_test
            steering_media_selftest)
        target_link_libraries(${verification_target} ls2k_project_headers)
    endforeach()

    target_link_libraries(power_adapter_threshold_test ls2k_linux_io)

endif()

if(LS2K_TFLM_OP_BENCH)
    add_executable(
        tflm_op_benchmark
        ../verification/tflm_op_benchmark/board_tflm_op_benchmark.cpp
    )
    target_include_directories(
        tflm_op_benchmark
        PRIVATE
        ../verification/tflm_op_benchmark
        ../verification/tflm_op_benchmark/generated
    )
    target_compile_definitions(
        tflm_op_benchmark
        PRIVATE
        TFLM_OP_BENCH_MODELS_HEADER="${LS2K_TFLM_OP_BENCH_MODELS_HEADER}"
        TFLM_OP_BENCH_ARENA_BYTES=1048576
    )
    target_link_libraries(
        tflm_op_benchmark
        ${LS2K_TFLM_LIB_PATH}
        pthread
        m
    )
    set_target_properties(tflm_op_benchmark PROPERTIES
        INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
        BUILD_WITH_INSTALL_RPATH ON
    )
endif()
