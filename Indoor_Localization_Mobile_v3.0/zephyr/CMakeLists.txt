cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(Indoor_Localization_Mobile_v3.0)

FILE(GLOB app_sources ../src/*.c*)
target_sources(app PRIVATE ${app_sources})
