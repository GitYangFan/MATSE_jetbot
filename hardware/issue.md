方法 1: 使用官方的 Kitware 仓库
Kitware 提供了一个官方的 CMake 软件包仓库，您可以通过添加这个仓库来安装 CMake 的特定版本（如 3.16）。

添加 Kitware 仓库

运行以下命令来将 Kitware 仓库添加到您的系统中：

bash
复制
编辑
sudo apt update
sudo apt install -y software-properties-common
sudo apt-get install -y lsb-release
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ $(lsb_release -c | awk '{print $2}') main'
添加 Kitware 的公钥

使用以下命令来下载并添加 Kitware 的 GPG 密钥：

bash
复制
编辑
wget -qO - https://apt.kitware.com/keys/kitware-archive.sh | sudo bash
安装 CMake 3.16

运行以下命令来安装特定版本的 CMake（3.16）：

bash
复制
编辑
sudo apt update
sudo apt install cmake=3.16.*
验证安装

安装完成后，可以通过以下命令检查 CMake 版本：

bash
复制
编辑
cmake --version
如果显示的是 CMake 3.16.x，那么您的安装已经成功。
