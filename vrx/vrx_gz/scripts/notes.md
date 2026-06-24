# vrx_gz/scripts 目录说明

包含用于批量更新海况参数的脚本：

- update_sea_state_params.sh
  - Shell 脚本：修改 `models` 和 `worlds` 文件中 `<wavefield>` 的 `<gain>`、`<period>` 等参数，以及世界文件中的风速参数，便于在本地批量调整海况/风场配置。

该脚本来源于 upstream（保留版权），在清理时可保留或移动到 `tools/` 以便与其它运维脚本集中管理。
