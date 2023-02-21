add_library(mpsse STATIC 
	ftdi_common.c
	ftdi_infra.c
	ftdi_mid.c
	ftdi_spi.c
)
target_link_libraries(mpsse PUBLIC d2xx)