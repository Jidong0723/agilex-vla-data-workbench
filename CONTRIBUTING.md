# Contributing

## 开始前

- 不提交任何真实 Episode、相机图像、机器人遥测、训练视图或导出的标签文件。
- 保持页面可作为纯静态站点在本地运行，不引入不必要的服务端依赖。
- 修改用户可见行为时，同步更新 `README.md` 或 `docs/` 中相关说明。

## 提交前检查

1. 确认 `app.js` 可通过 `node --check app.js`。
2. 确认页面可通过本地静态服务加载。
3. 使用简短、明确的提交信息，例如 `feat: add label export validation`。
4. 不在提交中包含 `training_view/`、`episode_source/` 或 `labels.json`。

## 问题报告

报告问题时请说明浏览器版本、复现步骤和不含隐私数据的报错信息。不要粘贴原始图像、完整 Episode 或设备标识符。
