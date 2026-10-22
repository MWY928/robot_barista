DOBOT Nova 2 冰美式 V1.1

基于 Americano 版本，使用项目根目录的 TCP_IP_Python_V3 SDK。
控制端口 29999，运动端口 30003；MovL 逐段运动，Sync 等待完成。

流程：
HOME -> SAFE_CUP -> CUP_PRE -> CUP_RECEIVE
-> 确认落杯 -> CUP_PRE -> SAFE_CUP
-> SAFE_ICE -> ICE_PRE -> ICE_POS
-> 确认冰和水均已出完、出口停止 -> ICE_PRE -> SAFE_ICE
-> SAFE_COFFEE -> COFFEE_PRE -> COFFEE_POS
-> 确认 espresso 出完，再等 2 秒 -> COFFEE_PRE -> SAFE_COFFEE
-> SAFE_WINDOW -> WINDOW_PRE -> WINDOW_PLACE
-> WINDOW_RELEASE -> WINDOW_EXIT -> WINDOW_CLEAR -> SAFE_WINDOW -> HOME

公共配置：根目录 nova2_points.json，与 Americano 共用点位及机器人设置。
公共点位只需示教一次，两套脚本读取同一份数据；须复核夹具、杯型、Tool/User 和 TCP 是否适用。
新增 SAFE_ICE（走廊点）、ICE_PRE（接近点）、ICE_POS（接冰水点）均待示教。
假设冰和水在同一 ICE_POS 接取；若出口分开，需要另加点位和动作。
所有点位需要末端 pose，HOME 还需要 joint；示教会同时保存两者。

在项目根目录执行：
  python IceAmericano/nova2_ice_coffee_v1_1.py --dry-run
  python IceAmericano/nova2_ice_coffee_v1_1.py --teach SAFE_ICE
  python IceAmericano/nova2_ice_coffee_v1_1.py --teach ICE_PRE
  python IceAmericano/nova2_ice_coffee_v1_1.py --teach ICE_POS
其他缺失点位同样示教，输入 SAVE 保存，不发送运动指令。

运行前核对 robot_ip、Tool/User、TCP 和负载。
持杯时保持杯子直立的工具姿态，固定 J6 不能保证杯子直立。
脚本不自动校验姿态一致性，不提供全臂避障。
低速验证所有路线，尤其 SAFE_CUP -> SAFE_ICE、SAFE_ICE -> SAFE_COFFEE。
完成验证后手动将 safe_routes_physically_verified.ice_americano 设为 true；任一脚本重新示教会重置所有流程标记。
先在 Studio 手动放到 HOME，再运行：
  python IceAmericano/nova2_ice_coffee_v1_1.py --run
默认每段输入 Y；--run --auto 跳过运动确认，但仍需人工确认落杯、冰水和 espresso。

外设接口 wait_cup_drop_finished / wait_ice_water_finished / wait_coffee_finished
均为人工确认占位，不会自动启动设备；需要人工选择冰水出料和 espresso 配方。
咖啡机只出 espresso，不加额外热水。人工确认目前没有自动超时。
接入真实设备时须实现启动、完成反馈和超时处理。
关闭网络连接不保证机器人停止。
