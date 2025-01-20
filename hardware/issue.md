方法 2: 从源代码手动安装 CMake 3.16
如果您希望手动安装 CMake 3.16，可以通过从源代码构建和安装。以下是步骤：

下载 CMake 3.16 源代码

您可以从 CMake 官网下载源代码包：

bash
复制
编辑
wget https://cmake.org/files/v3.16/cmake-3.16.8.tar.gz
解压并进入目录

bash
复制
编辑
tar -zxvf cmake-3.16.8.tar.gz
cd cmake-3.16.8
构建并安装 CMake

执行以下命令构建并安装 CMake：

bash
复制
编辑
./bootstrap
make
sudo make install
验证安装

完成安装后，使用以下命令检查 CMake 版本：

bash
复制
编辑
cmake --version
这样，您就可以手动安装 CMake 3.16 了。
