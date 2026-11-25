cmake_minimum_required(VERSION 3.16)

# Dual-mode CMakeLists.txt:
#   1. Native ESP-IDF component when ESP_PLATFORM is defined (idf.py add-dependency)
#   2. Standard portable C11 static library (OpenNear::core) for Linux/macOS/Windows/STM32/Zephyr

set(OPENNEAR_CORE_SOURCES
    core/src/on_frame.c
    core/src/on_polar.c
    core/src/on_sync.c
    core/src/on_superframe.c
    core/src/on_sec.c
    core/src/on_ecdh.c
    core/src/on_afh.c
    core/src/on_amc.c
    core/src/on_sar.c
    core/src/on_ranging.c
    core/src/on_ota.c
    core/src/on_gateway.c
    core/src/on_mesh.c
    core/src/on_mqtt.c
    core/src/on_locate.c
    core/src/on_node.c
)

if(DEFINED ESP_PLATFORM)
    set(OPENNEAR_IDF_SRCS ${OPENNEAR_CORE_SOURCES})
    set(OPENNEAR_IDF_INCS include core/include hal ports/esp32_wroom32/include)
    set(OPENNEAR_IDF_REQS esp_wifi esp_timer driver nvs_flash app_update)

    list(APPEND OPENNEAR_IDF_SRCS
        ports/esp32_wroom32/src/hal_esp32_raw80211.c
        ports/esp32_wroom32/src/esp32_ota_flash.cpp
    )
    if(CONFIG_ARDUINO_RUNNING_CORE OR DEFINED ARDUINO_ARCH_ESP32)
        list(APPEND OPENNEAR_IDF_SRCS
            ports/esp32_wroom32/src/esp32_demo_app.cpp
            ports/esp32_wroom32/src/esp32_tls_transport.cpp
            ports/esp32_wroom32/src/esp32_net_gateway.cpp
            ports/esp32_wroom32/src/opennear_esp32_sdk.cpp
        )
        list(APPEND OPENNEAR_IDF_REQS arduino)
    endif()

    idf_component_register(
        SRCS ${OPENNEAR_IDF_SRCS}
        INCLUDE_DIRS ${OPENNEAR_IDF_INCS}
        REQUIRES ${OPENNEAR_IDF_REQS}
    )
else()
    project(OpenNear VERSION 7.3.0 LANGUAGES C)

    option(OPENNEAR_BUILD_TESTS "Build OpenNear C11 42-suite automated tests" ON)
    option(OPENNEAR_BUILD_SIM   "Build OpenNear 10-node multi-hop simulator" ON)

    add_library(opennear_core STATIC ${OPENNEAR_CORE_SOURCES})
    add_library(OpenNear::core ALIAS opennear_core)

    target_compile_features(opennear_core PUBLIC c_std_11)
    target_include_directories(opennear_core
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hal>
            $<INSTALL_INTERFACE:include/opennear>
    )
    if(UNIX)
        target_link_libraries(opennear_core PUBLIC m)
    endif()

    if(OPENNEAR_BUILD_TESTS)
        enable_testing()
        add_executable(test_opennear sim/sim_world.c tests/test_opennear.c)
        target_include_directories(test_opennear PRIVATE sim)
        target_link_libraries(test_opennear PRIVATE opennear_core)
        add_test(NAME opennear_core_suite COMMAND test_opennear)
    endif()

    if(OPENNEAR_BUILD_SIM)
        add_executable(opennear_sim sim/sim_world.c sim/main_sim.c)
        target_include_directories(opennear_sim PRIVATE sim)
        target_link_libraries(opennear_sim PRIVATE opennear_core)
    endif()
endif()
