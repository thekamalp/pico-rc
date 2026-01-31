add_executable(pico_rc
        "pico-rc.c"
        "btclient.c")

# pull in common dependencies
target_link_libraries(pico_rc
    pico_stdlib
    hardware_pwm
    hardware_adc
    pico_btstack_classic
    pico_btstack_cyw43
    pico_cyw43_arch_none)

# include current directory for btstack_config.h
target_include_directories(pico_rc PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    )

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_rc)

# add url via pico_set_program_url
example_auto_set_url(pico_rc)
