# #!/bin/bash

# # 创建并进入 build 目录，每次都会删除之前的文件
# mkdir -p build 
# cd build
# rm -rf *

# # 运行 CMake 和 Ninja 构建
# cmake -G Ninja ..
# cmake --build . --target all --config Release -- -j 16

# # 获取当前工程所在的盘符
# drive_letter=$(pwd | cut -d'/' -f2 | tr '[:lower:]' '[:upper:]')

# # 修改 compile_commands.json 文件中的路径
# sed -i "s|/c/|C:/|g; s|/d/|D:/|g; s|/e/|E:/|g; s|/f/|F:/|g; s|/g/|G:/|g; s|/h/|H:/|g; s|/i/|I:/|g; s|/j/|J:/|g; s|/k/|K:/|g; s|/l/|L:/|g; s|/m/|M:/|g; s|/n/|N:/|g; s|/o/|O:/|g; s|/p/|P:/|g; s|/q/|Q:/|g; s|/r/|R:/|g; s|/s/|S:/|g; s|/t/|T:/|g; s|/u/|U:/|g; s|/v/|V:/|g; s|/w/|W:/|g; s|/x/|X:/|g; s|/y/|Y:/|g; s|/z/|Z:/|g" compile_commands.json

# # 获取 .ioc 文件的前缀
# ioc_file=$(basename ../*.ioc .ioc)
# elf_file="$ioc_file.elf"
# bin_file="$ioc_file.bin"
# hex_file="$ioc_file.hex"

# # 检查 ELF 文件是否存在
# if [ ! -f "$elf_file" ]; then
#     echo "Error: ELF file $elf_file not found!"
#     exit 1
# fi

# # 生成二进制文件和 HEX 文件
# arm-none-eabi-objcopy -O binary "$elf_file" "$bin_file"
# arm-none-eabi-objcopy -O ihex "$elf_file" "$hex_file"

# # 检查二进制文件是否生成成功
# if [ ! -f "$bin_file" ]; then
#     echo "Error: Binary file $bin_file not generated!"
#     exit 1
# fi

# # 检查 HEX 文件是否生成成功
# if [ ! -f "$hex_file" ]; then
#     echo "Error: HEX file $hex_file not generated!"
#     exit 1
# fi

# # 打印文件大小信息
# echo "ELF file size:"
# arm-none-eabi-size "$elf_file"

# echo "Binary file information:"
# file "$bin_file"

# echo "HEX file size:"
# arm-none-eabi-size "$hex_file"

# echo "清理临时文件..."


# # 烧录
# #openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program $hex_file verify reset exit"

#!/bin/bash

# 创建 build 目录（如果不存在）
mkdir -p build 
cd build

# 检查是否需要重新生成 CMake 文件
need_cmake=false
if [ ! -f "build.ninja" ] || [ "../CMakeLists.txt" -nt "build.ninja" ]; then
    need_cmake=true
fi

# 只在需要时运行 CMake
if [ "$need_cmake" = true ]; then
    echo "正在配置 CMake..."
    cmake -G Ninja ..
fi

# 增量编译（只编译修改过的文件）
echo "正在编译..."
cmake --build . --target all --config Release -- -j $(nproc)

# 检查编译是否成功
if [ $? -ne 0 ]; then
    echo "编译失败!"
    exit 1
fi

# 只在第一次或 compile_commands.json 更新时修改路径
if [ ! -f "compile_commands_fixed.json" ] || [ "compile_commands.json" -nt "compile_commands_fixed.json" ]; then
    echo "修正 compile_commands.json 路径..."
    sed "s|/c/|C:/|g; s|/d/|D:/|g; s|/e/|E:/|g; s|/f/|F:/|g; s|/g/|G:/|g; s|/h/|H:/|g; s|/i/|I:/|g; s|/j/|J:/|g; s|/k/|K:/|g; s|/l/|L:/|g; s|/m/|M:/|g; s|/n/|N:/|g; s|/o/|O:/|g; s|/p/|P:/|g; s|/q/|Q:/|g; s|/r/|R:/|g; s|/s/|S:/|g; s|/t/|T:/|g; s|/u/|U:/|g; s|/v/|V:/|g; s|/w/|W:/|g; s|/x/|X:/|g; s|/y/|Y:/|g; s|/z/|Z:/|g" compile_commands.json > compile_commands_fixed.json
    cp compile_commands_fixed.json compile_commands.json
fi

# 获取 .ioc 文件的前缀
ioc_file=$(basename ../*.ioc .ioc)
elf_file="$ioc_file.elf"
bin_file="$ioc_file.bin"
hex_file="$ioc_file.hex"

# 检查 ELF 文件是否存在
if [ ! -f "$elf_file" ]; then
    echo "错误: ELF 文件 $elf_file 未找到!"
    exit 1
fi

# 只在 ELF 文件更新时重新生成二进制文件
if [ "$elf_file" -nt "$bin_file" ] || [ ! -f "$bin_file" ]; then
    echo "生成二进制文件..."
    arm-none-eabi-objcopy -O binary "$elf_file" "$bin_file"
fi

if [ "$elf_file" -nt "$hex_file" ] || [ ! -f "$hex_file" ]; then
    echo "生成 HEX 文件..."
    arm-none-eabi-objcopy -O ihex "$elf_file" "$hex_file"
fi

# 检查文件是否生成成功
if [ ! -f "$bin_file" ]; then
    echo "错误: 二进制文件 $bin_file 生成失败!"
    exit 1
fi

if [ ! -f "$hex_file" ]; then
    echo "错误: HEX 文件 $hex_file 生成失败!"
    exit 1
fi

# 打印文件大小信息
echo "文件信息:"
arm-none-eabi-size "$elf_file"

echo "编译完成! 文件位置:"
echo "  ELF: $(pwd)/$elf_file"
echo "  BIN: $(pwd)/$bin_file"  
echo "  HEX: $(pwd)/$hex_file"

# 可选：自动烧录（取消注释启用）
# echo "正在烧录..."
# openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg -c "program $hex_file verify reset exit"