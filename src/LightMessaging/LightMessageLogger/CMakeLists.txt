set(FILTER_NAME lightmessage_logger)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

adtf_add_filter(${FILTER_NAME}
    stdafx.h

    LightMessageLogger.h
    LightMessageLogger.cpp
)


adtf_set_folder(${FILTER_NAME} aadcuser/lightmessagelogger) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
