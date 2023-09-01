if(WIN32)
    add_library(usb_bridge STATIC usb_bridge.c)
    target_link_libraries(usb_bridge PUBLIC mpsse)
else()
    add_library(usb_bridge STATIC usb_bridge_libftdi.c)
    target_link_libraries(usb_bridge PUBLIC libftdi)
endif()

