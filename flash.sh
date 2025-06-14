#!/bin/bash
# 只是烧录，并不能复位
probe-rs download --chip  STM32F407ZG build/Weak_Signal.elf
# 复位
probe-rs reset --chip  STM32F407ZG
# 这个脚本是我偷懒写的，本质就是调用probe-rs，直接命令行输入也行