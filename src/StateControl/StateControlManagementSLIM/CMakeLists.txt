set(FILTER_NAME aadc_StateControlManagementSLIM)

   
adtf_add_filter(${FILTER_NAME}
            stdafx.cpp
            stdafx.h
            StateControlManagementSlim.cpp            
            StateControlManagementSlim.h
            StateControl.h
            ${HEADER_MOC}
)


# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/StateControlManagementSLIM) 

adtf_install_plugin(${FILTER_NAME} ${AADC_INSTALL_DIR})
    




