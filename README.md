# flight_with_t265

> Yunfan  REN
>
> renyunfan@outlook.com

# History

* 12/26/20：更新了初始化状态和自定义代码框架

# Warnning

## 1.1 ROSBAG

每次飞行前习惯性录制`rosbag`，可以再`.zshrc`中添加录制命令

```bash
alias lgb="rosbag record -a -O"
```

随后在使用时在终端中输入

```bash
lgb NAME
```

其中`[NAME]`是你希望保存的包名字。

## 1.2 主循环

主循环状态机中，千万不要把主循环写在`if`中了。

自定义代码请写在`src/mav_fsm.cpp`中

```cpp
void MavFsmNode::att_mode() {
    if(cnt++ > 50)// make the information frequency reduce to 1Hz.
    {
        cnt=0;
        ROS_WARN("[WTR MAV MAIN FSM][ATT]:--------------ATT HAS NOT BEEEN DEFINED-------------------");
    }
    /*  Write your own code here!!! */


}
```

