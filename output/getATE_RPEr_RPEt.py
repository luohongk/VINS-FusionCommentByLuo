"""
Author: Hongkun Luo
Date: 2024-07-25 10:06:03
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
import threading
import re


data_sequence1 = "V201"
data_sequence2 = "MH01"


# 读取txt文件中的命令
with open("run.txt", "r") as f:
    temp_commands = [line.strip() for line in f]

# 将 MH01 替换为 MH02
change_commands = []
for cmd in temp_commands:
    if data_sequence1 in cmd:
        cmd = cmd.replace(data_sequence1, data_sequence2)
    change_commands.append([cmd])

# 使用示例
commands = [
    [change_commands[0]],
    [change_commands[1]],
    [change_commands[2]],
    [change_commands[3]],
    [change_commands[4]],
    [change_commands[5]],
    [change_commands[6]],
    [change_commands[7]],
    [change_commands[8]],
]


# 定义一个函数来执行命令并捕获输出
def run_command(cmd):
    output = []
    proc = subprocess.Popen(
        cmd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    )
    while True:
        realtime_output = proc.stdout.readline()
        if realtime_output == "" and proc.poll() is not None:
            break
        if realtime_output:
            output.append(realtime_output.strip())
    rc = proc.poll()
    return output


# 执行命令并保存输出
all_output = []
for cmd in commands:
    print("执行命令:", cmd)
    output = run_command(cmd)
    all_output.extend(output)

# 遍历 all_output，捕获包含 "rmse" 字符串的行中的数字
rmse_values = []
for line in all_output:
    if "rmse" in line.lower():
        # 使用正则表达式捕获数字
        match = re.search(r"\b\d+(\.\d+)?\b", line)
        if match:
            rmse_values.append(float(match.group()))

# 将 RMSE 值保存到 TXT 文件
with open("rmse_values.txt", "w") as f:
    f.write(" ".join(map(str, rmse_values)))

# 保存输出到文件
with open("terminal_output.txt", "w", encoding="utf-8") as f:
    for line in all_output:
        f.write(line + "\n")

print("捕获的 RMSE 值:", rmse_values)
print("所有命令已完成执行,输出已保存到 terminal_output.txt 文件.")
