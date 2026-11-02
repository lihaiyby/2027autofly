"""2027autofly：搜索-投递-走廊-降落 任务栈。

分层：
    config   纯数据（YAML -> dataclass），无 ROS 依赖
    search   搜索航线与目标簿记，纯逻辑，可单测
    flight   MAVROS 飞行原语（解锁/OFFBOARD/设定点/守护）
    vision   /uav_vision 话题封装
    mission  编排：起飞 -> 搜索 -> 投递 -> 走廊 -> 降落
    cli      命令行入口
"""

__version__ = "1.0.0"
