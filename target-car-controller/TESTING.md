# 串口与蓝牙通信测试步骤

本阶段先验证左右ES3144霍尔端点，再验证遥控和四种工作模式。自动平移由两端磁铁触发换向，测试时仍须有人看护。

## 1. 烧录固件

使用以下任一文件：

```text
C:\Users\users\Desktop\HNU_RailTargetVehicle\cmake-build-debug\HNU_RailTargetVehicle.hex
C:\Users\users\Desktop\HNU_RailTargetVehicle\cmake-build-debug\HNU_RailTargetVehicle.bin
```

烧录后先架空车轮或断开传动机构验证方向；确认方向正确后，再把靶车放到轨道中间进行低风险实车测试。

## 2. 接线

```text
STM32 PA9  / USART1_TX -> USB-TTL或HC-05 RX
STM32 PB7  / USART1_RX <- USB-TTL或HC-05 TX
STM32 GND              -- USB-TTL或HC-05 GND
STM32 PB12 / HALL_LEFT  <- 左侧霍尔模块DO（建议串联1 kΩ）
STM32 PB14 / HALL_RIGHT <- 右侧霍尔模块DO（建议串联1 kΩ）
STM32 +5V               -> 两个霍尔模块VCC
STM32 GND               -- 两个霍尔模块GND（必须共地）
```

串口参数固定为 `115200 / 8-N-1`。HC-05 RX是否需要分压取决于模块底板是否已包含电平转换。

## 3. 启动上位机

双击 `start.bat`，或者执行：

```powershell
npm start
```

点击“刷新”，选择USB-TTL或蓝牙虚拟COM口，然后点击“连接”。

## 4. 预期测试现象

### 连接

- 页面显示“设备在线”。
- 日志出现 `PONG：STM32在线`。
- STM32 PH10指示灯常亮；未连接时该灯闪烁。
- 回传模式变量显示 `1 遥控`。
- 正确帧计数持续增加，因为上位机每200 ms发送心跳。

### 摇杆

- 左右拖动时，发送和回传的平移变量在 `-1000～1000` 变化。
- 上下拖动时，发送和回传的旋转变量在 `-1000～1000` 变化。
- 松开摇杆后，两项回到0。
- 遥控模式下，M3508目标转速为 `±1500 rpm`，GM6020目标转速为 `±200 rpm`。
- 松开摇杆时目标转速应立即变为0，PID输出会短暂呈现制动方向，实际转速接近0后PID输出再回到接近0。

### 模式按钮

| 按钮 | STM32 `mode` | 预期行为 |
|---|---:|---|
| 遥控模式 | 1 | 摇杆左右控制M3508，上下控制GM6020 |
| 自动旋转靶 | 2 | M3508停止，GM6020以 `+200 rpm` 持续旋转 |
| 自动平移靶 | 3 | GM6020停止，M3508以 `±1500 rpm` 运行，到左右端点后换向 |
| 自动旋转平移靶 | 4 | 上述自动旋转与自动平移同时执行 |

切换模式时，STM32中的 `move_command` 和 `rotate_command` 应归零，两个电机先停约200 ms后再执行新模式。M3508反向时会先将目标转速降到0，再反向加速，不会直接从正转跳成反转。

### 推荐逐项测试

1. 先点“遥控模式”，确认原有左右平移和上下旋转都正常。
2. 把靶车放在轨道中间，点“自动旋转靶”：M3508目标应为0，GM6020目标应逐渐达到200 rpm。
3. 电机动力断开时，用磁铁正确的一极分别靠近两个霍尔模块；连续检测约15 ms后，对应端点应显示“已触发”，移开约15 ms后应解除。
4. 确认左右霍尔都能分别触发和释放，再接通电机动力并点“自动平移靶”：M3508到端点时目标立即归零，停约200 ms后向相反方向加速；GM6020目标应为0。
5. 再测试“自动旋转平移靶”：两项动作同时存在。
6. 在任一自动模式中点击“立即急停”，两个目标和PID输出都应归零。

如果页面左右霍尔与实际安装方向相反，先交换PB12/PB14两个信号插头或修正软件映射，再进行自动平移测试。两端同时触发时M3508应停止。

### 急停

- 点击“立即急停”：`emergency_stop=1`、`mode=0`、两个控制变量为0。
- 急停状态下点击其他模式，下位机返回错误状态，模式不改变。
- 点击“解除急停”：`emergency_stop=0`，模式仍保持0；之后需要重新选择模式。

### 断联

- 断开串口或关闭蓝牙后约600 ms，PH10指示灯恢复闪烁。
- 调试器中 `g_bt_test_state.link_online=0`、`mode=0`、两个控制变量为0。
- 重新连接时必须重新发送模式和控制量。

## 5. 调试器观察变量

在 Watch 窗口添加：

```c
g_bt_test_state
g_target_motor_test_state
```

重点观察：

```text
mode
emergency_stop
link_online
last_command
move_command
rotate_command
valid_frames
crc_errors
framing_errors
track_target_rpm
track_speed_rpm
track_control_output
armor_target_rpm
armor_speed_rpm
armor_control_output
```

正常测试时 `crc_errors` 和 `framing_errors` 应保持0。

## 6. 常见故障

- 能连接COM口但没有PONG：检查TX/RX是否交叉、是否共地、波特率是否115200。
- STM32能接收但页面没有回传：重点检查STM32 TX到蓝牙RX这一路。
- CRC错误持续增加：检查是否选择了错误串口、串口参数是否一致，或线路受到干扰。
- 蓝牙COM口无法打开：关闭其他占用该COM口的串口助手。
- 蓝色LED亮但摇杆变量不变：确认当前模式为遥控模式且急停已解除。
- 自动模式不动作：确认蓝牙仍连接、急停已解除，并确认对应电机的 `track_online` 或 `armor_online` 为1。
- 到端点后不反向：检查对应霍尔原始输入和端点确认是否变为“检测到磁铁/已触发”，并确认磁铁极性和安装距离。
- 到端点后继续朝障碍运动：立即急停，并检查M3508正转方向是否确实对应右侧；如果相反，需要修改方向映射。
