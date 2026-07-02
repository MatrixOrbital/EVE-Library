add_library(mpsse STATIC 
	ftdi_infra.c
	ftdi_mid.c
	ftdi_spi.c
)
target_link_libraries(mpsse PUBLIC d2xx)
target_compile_definitions(mpsse PUBLIC FTD2XX_STATIC FTDIMPSSE_STATIC FTDIMPSSE_EXPORTS)
