set(FILTER_NAME PIDcontroller)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp     
                stdafx.h
                PIDController.h
                PIDController.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/pidcontroller) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
