# 改进与修正避障算法库的ROS2迁移实现

task36将一个ros1的避障算法部分移植到了ros2. 检查发现有些不一致或bug, 需要改进的

## 具体要求

1. 主要参考文件是：37ref-dyn_small_obs_avoidance_ros2_review_zh.md. 此文件来自gpt-6-pro的审查，可作为较为可靠的参考。如果执行中没有遇到问题，则不需要再完整检查复现此审查结果
2. 执行任务前，将当前版本的dyn_small_obs_avoidance-ros2进行一次全量备份
3. 不破坏当前作为一个ROS2包的独立性
4. 在飞机`new`上进行验证。当前飞机前方2-3米有一处障碍物，可以进行实机测试；不要解锁或起飞飞机
5. 对于难度不高的独立任务，并行发布gpt-6.1-sol high subagent执行具体修改，你只负责指挥
6. 对于可复现且迟迟定位不了的分歧，交给一个 gpt-6-astra high subagent 进行排查