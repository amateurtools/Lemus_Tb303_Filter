cmake_minimum_required(VERSION 3.22)
project(TB303VCF VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DJUCE_WEB_BROWSER=0)
add_subdirectory(JUCE) # JUCE source in ./JUCE

if (MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# No editor, no GUI code of our own: the host (REAPER) draws its own generic parameter UI.
juce_add_plugin(TB303VCF
    COMPANY_NAME "AmateurTools"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD FALSE
    PLUGIN_MANUFACTURER_CODE Amat
    PLUGIN_CODE T3vf
    FORMATS VST3 AU
    PRODUCT_NAME "TB303 VCF"
)

target_sources(TB303VCF
    PRIVATE
        Source/PluginProcessor.cpp

    PUBLIC
        Source/PluginProcessor.h
        Source/tb303_vcf.h
)

target_include_directories(TB303VCF
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
)

target_compile_definitions(TB303VCF
    PUBLIC
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
        JUCE_VST3_CAN_REPLACE_VST2=0
)

juce_generate_juce_header(TB303VCF)

# audio_processors pulls in gui_basics as a dependency, but we link and use no GUI of our own.
target_link_libraries(TB303VCF PRIVATE
    juce::juce_audio_basics
    juce::juce_audio_plugin_client
    juce::juce_audio_processors
    juce::juce_core
    juce::juce_data_structures
    juce::juce_events

    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags
)

# =============================================================================
# Compiler optimizations
# =============================================================================
# Deliberately NO -ffast-math / -ffinite-math-only here: the filter solves a stiff implicit
# system with Newton iterations and relies on std::isfinite() to catch a blow-up and reset.
# Those flags let the compiler assume NaN/Inf never happen, which silently removes that guard.
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    if(MSVC)
        target_compile_options(TB303VCF PRIVATE
            /O2
            /Oi
            /Ot
            /GL
            /arch:AVX2
        )
        target_link_options(TB303VCF PRIVATE /LTCG)
    else()
        target_compile_options(TB303VCF PRIVATE
            -O3
            -funroll-loops
            -fno-math-errno
        )
        if(NOT APPLE)
            target_compile_options(TB303VCF PRIVATE -march=native)
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
            target_compile_options(TB303VCF PRIVATE -mcpu=apple-m1)
        endif()
    endif()

    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
    if(ipo_supported)
        set_target_properties(TB303VCF PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
        message(STATUS "IPO/LTO enabled for TB303VCF")
    else()
        message(STATUS "IPO/LTO not supported: ${ipo_error}")
    endif()
else()
    message(STATUS "Building in Debug mode - optimizations disabled")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(TB303VCF PRIVATE
        JUCE_DISABLE_ASSERTIONS=1
        NDEBUG=1
    )
endif()

if(APPLE)
    set_target_properties(TB303VCF PROPERTIES OSX_ARCHITECTURES "arm64;x86_64")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
if(NOT CMAKE_BUILD_TYPE)
    message(WARNING "CMAKE_BUILD_TYPE not set! Use: cmake -B build -DCMAKE_BUILD_TYPE=Release")
endif()