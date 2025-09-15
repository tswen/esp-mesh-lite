set(srcs "src/esp_mesh_lite_espnow.c")
set(require_components "json" "esp_wifi" "app_update" "wifi_provisioning" "qrcode" "nvs_flash" "protobuf-c" "console")

if (CONFIG_MESH_LITE_ENABLE)
    list(APPEND srcs "src/esp_mesh_lite.c" "src/esp_mesh_lite_port.c" "src/esp_mesh_lite_log.c" "src/mesh_lite.pb-c.c")
endif()

if (CONFIG_MESH_LITE_PROV_ENABLE)
    list(APPEND srcs "src/wifi_prov/wifi_prov_mgr.c" "src/wifi_prov/zero_provisioning.c")
endif()

if (CONFIG_MESH_LITE_WIRELESS_DEBUG)
    list(APPEND srcs "src/esp_mesh_lite_wireless_debug.c")
endif()

# Apply ESP-IDF Patch
if(IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR EQUAL 5)
    execute_process(COMMAND
                    git apply ${CMAKE_CURRENT_LIST_DIR}/patches/lwip_kconfig_v5.5.patch
                    WORKING_DIRECTORY $ENV{IDF_PATH})
    message(STATUS "Apply ESP-IDF Patch: lwip_kconfig_v5.5.patch")
else()
    execute_process(COMMAND
                    git apply ${CMAKE_CURRENT_LIST_DIR}/patches/lwip_kconfig_below_v5.5.patch
                    WORKING_DIRECTORY $ENV{IDF_PATH})
    message(STATUS "Apply ESP-IDF Patch: lwip_kconfig_below_v5.5.patch")
endif()

set(package_version "")
set(yml_file "${CMAKE_CURRENT_LIST_DIR}/idf_component.yml")
if(EXISTS ${yml_file})
    file(READ ${yml_file} ver)
    string(REGEX MATCH "version:[ ]*\"([^\"]+)\"" _ ${ver})
    set(package_version ${CMAKE_MATCH_1})
    message(STATUS "Found package version: ${package_version}")
else()
    message(WARNING " ${yml_file} not exist")
endif()

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS include
                       REQUIRES ${require_components})

target_compile_definitions(${COMPONENT_LIB} PRIVATE PACKAGE_VERSION=\"${package_version}\")

execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_LIST_DIR}/include/esp_mesh_lite_lwip_hook.h ${CMAKE_BINARY_DIR}/config/)
get_property(hook_file GLOBAL PROPERTY CUSTOMER_LWIP_HOOK_FILE_LISTS)

if(NOT "${hook_file}" STREQUAL "")
    file(APPEND ${hook_file} "#include \"esp_mesh_lite_lwip_hook.h\"\n")
endif()

idf_component_get_property(lwip lwip COMPONENT_LIB)
if(IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR LESS 4)
    target_link_libraries(${lwip} INTERFACE "-Wl,--wrap=dhcp_parse_extra_opts_internal")
endif()

if (NOT CONFIG_MESH_LITE_CORE_ENABLE)
    string(CONCAT LIBS "esp_mesh_lite_" "${IDF_TARGET}")

    add_library(${LIBS} STATIC IMPORTED)

    if (CONFIG_MESH_LITE_PROTO_VERSION_V1X)
        set(REMOVE_FILE esp_mesh_lite_proto_v0_x.c.obj
                        esp_mesh_lite_lan_ota_proto_v0_x.c.obj)
    else()
        set(REMOVE_FILE esp_mesh_lite_proto_v1_x.c.obj
                        esp_mesh_lite_lan_ota_proto_v1_x.c.obj)
    endif()

    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${LIBS}.a ${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a
        OUTPUT_VARIABLE AR_OUTPUT
        ERROR_VARIABLE AR_ERROR)

    foreach(file ${REMOVE_FILE})
        execute_process(
            COMMAND ${CMAKE_AR} d "${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a" "${file}"
            OUTPUT_VARIABLE AR_OUTPUT
            ERROR_VARIABLE AR_ERROR)

        execute_process(
            COMMAND ${CMAKE_NM} "${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a"
            OUTPUT_VARIABLE AR_OUTPUT
            ERROR_VARIABLE AR_ERROR)

        string(FIND "${AR_OUTPUT}" "${file}" FOUND_INDEX)
        if(FOUND_INDEX GREATER -1)
            message(FATAL_ERROR
            "- MeshLite protocol version mismatch, terminating build.
            - Please raise an issue at https://github.com/espressif/esp-mesh-lite/issues")
        endif()
    endforeach()

    set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a)

    target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
    set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})
endif()

include(package_manager)
if (CONFIG_MESH_LITE_ENABLE)
    cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
endif()
