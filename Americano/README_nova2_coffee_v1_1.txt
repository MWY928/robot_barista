DOBOT Nova 2 单臂美式咖啡搬运 V1.1
========================================================
机型：Nova 2 DT-NOVA020-1；SDK：https://github.com/Dobot-Arm/TCP-IP-Python-V3
需确认固件与 SDK 兼容。

为什么修改：
  V1 使用 JointMovJ 直接从落杯机前跨到咖啡机前，
  关节运动末端/关节连杆扫过的空间不可从起终点推断，会有撞机风险。示教的安全走廊点位 SAFE_CUP / SAFE_COFFEE / SAFE_WINDOW。
  所有工位接近/撤离及走廊段暂用 MovL；每段必须确认可达、无奇异、无碰撞。
  MovL 只约束 TCP 直线，不能保证整台机械臂无碰撞。

工位序列：
  先用 Studio 手动让机械臂处于 HOME 并验证关节角
  HOME -> SAFE_CUP -> CUP_PRE -> CUP_RECEIVE
  -> 确认落杯 -> CUP_PRE -> SAFE_CUP
  -> SAFE_COFFEE -> COFFEE_PRE -> COFFEE_POS
  -> 确认制作完成再等待 2 秒 -> COFFEE_PRE -> SAFE_COFFEE
  -> SAFE_WINDOW -> WINDOW_PRE -> WINDOW_PLACE
  -> WINDOW_RELEASE -> WINDOW_EXIT -> WINDOW_CLEAR
  -> SAFE_WINDOW -> HOME

使用：
1. Windows 安装 Python3 和 numpy，下载官方 TCP-IP-Python-V3；
   保持仓库目录结构；SDK 位于根目录 TCP_IP_Python_V3，公共配置位于根目录 nova2_points.json。
2. DobotStudio Pro 配置 TCP、Tool、User、负载以及机器 IP，切到兼容的 TCP/IP 模式。
3. 一次性记录点位，如：
       python Americano/nova2_coffee_v1_1.py --teach SAFE_CUP
   程序只读取当前位置，输入 SAVE 后存入 JSON，不发送运动指令。
   其他点位名称见根目录 nova2_points.json，未记录的点位需示教；不要按猜测填写坐标。
4. 流程干运行（无需 SDK 或机器人）：
       python Americano/nova2_coffee_v1_1.py --dry-run
5. 所有动作在 Studio 中低速、分段验证：机器/夹具/纸杯及整臂都应有间隙，
   包括走廊内从 HOME 到 SAFE_CUP、从 SAFE_CUP 到 SAFE_COFFEE 等线性段。
6. 物理验证完成后，才手动将 JSON 中 safe_routes_physically_verified.americano 设为 true。
7. 手动将机器人置于 HOME，启动机器人及安全回路，之后运行：
       python Americano/nova2_coffee_v1_1.py --run
   程序先检查 HOME 关节角，而不会从未知起始姿态盲目 JointMovJ 回 HOME。
   默认每段发送前输入 Y；发生错误后不自动清故障或运动。

- 尽量让 SAFE_CUP、SAFE_COFFEE、SAFE_WINDOW 在无遮挡的一侧形成通道，
  运输杯子时尽量使用一致的工具姿态和高度；核对肘部、底座、夹具的扫掠空间。
- 若一次长 MovL 超出工作范围/遇到奇异位置，应拆分更多安全段，不要强行执行。
- 首次先不拿杯，在人工监护、可触达急停前提下单段低速验证；
  再使用空杯、冷水，最终验证热咖啡。

外设：
  wait_cup_drop_finished() 和 wait_coffee_finished() 为人工输入占位接口，
  后续替换为自动落杯机和咖啡机真正的信号反馈，且需加入超时保护。

重要限制：
- 代码没有计算碰撞体积或做全臂避障，不代替经过风险评估的工作站安全措施。
- 调试必须设置夹具 TCP 和负载，记录数据时的用户/工具坐标系不能变化。
- 路径验证开关仅是人工流程记录，不是安全证明，也没有经过真机安全认证。
- 不能同时由 Studio 和脚本争夺运动控制权。


点位和机器人设置与冰美式共用；运行仅检查本流程所需点位。
任一脚本重新示教后，会重置所有流程的路线验证标记。
