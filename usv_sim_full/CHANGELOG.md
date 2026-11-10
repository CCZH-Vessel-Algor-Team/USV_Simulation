# Changelog

后续修改本包功能、配置或文档时，应在 `Unreleased` 中记录面向使用者的变更。

## Unreleased

### Added

- 增加 `usv_bringup_system_m5.launch.py` 与仓库内 M5 场景/功率控制配置；普通 bringup 默认入口不变。
- 新增功能包架构和数据流文档。

### Changed

- M5 仿真采用快响应 PI，巡航目标3m/s；ALOS与速度平滑器按实测推力余量设置速度/加速度限制。未改模型系数、到站停稳或Spin策略。
- M5 传感器安装跟随船体网格对齐变换，保持原相对安装关系；M5 可显式排除不匹配的 RViz 工具，保留真实目标船和导航目标入口。
- 更新 README，说明构建、配置和启动入口。
