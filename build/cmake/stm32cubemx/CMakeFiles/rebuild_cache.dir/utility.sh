set -e

cd /e/PostGraduateProgram/WeakSignal/WeakSignal_VoltCur/SoftWare/debug/Weak_Signal/build/cmake/stm32cubemx
/usr/bin/cmake.exe --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
