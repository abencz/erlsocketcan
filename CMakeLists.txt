cmake_minimum_required(VERSION 2.8)

project(erlang_socketcan)

add_library(socketcan_driver SHARED
	socketcan_driver.c
	)

target_link_libraries(socketcan_driver erl_interface ei)

set_target_properties(socketcan_driver PROPERTIES PREFIX "")
