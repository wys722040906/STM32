# Minicom

- 安装

```
sudo apt update
sudo apt install minicom
```

- 配置

```
sudo minicom -s
Save setup as dfl：
 Exit
sudo usermod -a -G dialout $USER //用户组
```

- 使用

```
sudo minicom -D /dev/ttyUSB0
Ctrl+A Z：显示帮助菜单，列出所有可用命令。
Ctrl+A X：退出 minicom。
```

# cutecom

- 安装

```
sudo apt install cutecom

```

- 使用

```
sudo usermod -a -G dialout $USER //用户组
sudo cutecom
```
