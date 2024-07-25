'''
Author: Hongkun Luo
Date: 2024-07-14 16:14:50
LastEditors: Hongkun Luo
Description: 

Hongkun Luo
'''
"""
Author: Hongkun Luo
Date: 2024-07-13 17:26:56
LastEditors: Hongkun Luo
Description: 

Hongkun Luo
"""

import subprocess
import sys
import time
import subprocess
import platform
import os
import dbus
# import wnck


# 关闭终端的函数
def close_all_terminals():
    os.system("pkill gnome-terminal")

close_all_terminals()

# 读取txt文件中的命令
with open("command.txt", "r") as f:
    temp_commands = [line.strip() for line in f]


# 打开终端并输入指令的函数
def open_terminal_with_command(command):
    system = platform.system()
    if system == "Windows":
        subprocess.Popen(f"start cmd /k {command}", shell=True)
    elif system == "Darwin":  # macOS
        subprocess.Popen(
            ["osascript", "-e", f'tell app "Terminal" to do script "{command}"']
        )
    elif system == "Linux":
        subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", f"{command}; exec bash"]
        )
        subprocess.call(["xdotool", "search", "--name", "Terminal", "windowminimize"])
    else:
        print(f"Unsupported operating system: {system}")


# 使用示例
commands = [
    [temp_commands[0]],
    [temp_commands[1]],
    [temp_commands[2]],
    [temp_commands[3]],
]

# 打开终端并且输入指令
for command in commands:
    for cmd in command:
        # open_terminal_with_command(cmd)
        # 等待1秒钟
        proc = subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"]
        )
        time.sleep(1)

def minimize_all_gnome_terminals():
    """最小化所有 GNOME 终端窗口"""
    try:
        # 使用 DBUS 连接到 GNOME Shell
        bus = dbus.SessionBus()
        obj = bus.get_object('org.gnome.Shell', '/org/gnome/Shell')
        iface = dbus.Interface(obj, 'org.gnome.Shell')

        # 获取所有当前打开的 GNOME 终端窗口
        windows = iface.get_windows()

        # 循环最小化每个终端窗口
        for window in windows:
            if 'Terminal' in window[2]:
                iface.Minimize(window[0])
                time.sleep(0.1)  # 添加一些延迟以避免过快

    except dbus.exceptions.DBusException as e:
        print(f"Error: {e}")

# 调用函数来最小化所有 GNOME 终端窗口
minimize_all_gnome_terminals()


# 如果某一个终端中断就关闭所有终端

# close_all_terminals()
# 检查所有终端是否停止输出内容
