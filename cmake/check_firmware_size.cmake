# 按100,000字节验收，比100 KiB更严格；统计烧录镜像，不统计ELF调试符号。
file(SIZE "${FIRMWARE_FILE}" firmware_bytes)
if(firmware_bytes GREATER_EQUAL 100000)
    message(FATAL_ERROR "Firmware ${firmware_bytes} bytes exceeds the <100000-byte budget")
endif()
message(STATUS "Firmware image: ${firmware_bytes} / 100000 bytes")
