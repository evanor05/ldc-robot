# 真浏览器验收（零依赖）

用真实 Chromium 打开**真实构建产物**，验证此前只能靠 vm 沙箱推断的交互。
不连车、不连 ESP32、不改生产代码。

## 运行

```bash
cd web_console
npm run build          # 必须先构建：验收打开的是 dist/
npm run test:browser   # = node tests/browser/acceptance.js
```

## 组成

| 文件 | 作用 |
| --- | --- |
| `fakeConsoleServer.js` | 假后端：静态 `dist/` + `/api/*` 固定响应 + 最小 Foxglove WebSocket 桥（握手需回选子协议，否则浏览器直接握手失败） |
| `acceptance.js` | CDP 驱动（Node 内置 WebSocket，不依赖 Playwright）：导航/点击/读 DOM/处理 confirm 对话框 |

## 当前覆盖

1. 面板渲染 + 真实 WS 握手（监听 App 自己的 `foxglove:connected` 事件，不用占位文案判定）
2. 差图不得被默认选中：开始导航禁用且给出 blocker
3. 就绪面板在"开始导航"之前可见
4. 开始建图后同时出现「保存并停止」与「停止但不保存」
5. 点「停止但不保存」→ 只发 `/api/abort_mapping`（带 sessionId），不发 `/api/save_map`
6. 相机 `running` 但无画面 → 显示"摄像头离线"，不显示"实时画面正常"

## 环境要求

- Chromium：默认在 `~/.cache/ms-playwright/chromium-*/chrome-linux64/chrome`；
  自定义用 `CHROME_PATH=/path/to/chrome`
- 端口：API 3000、WS 8765（构建产物把这些端口烘死了），可用
  `FAKE_API_PORT` / `FAKE_WS_PORT` **但必须与构建时一致**，否则页面连不上
- 视口固定 1440×900：小窗口会同时渲染移动端布局，造成与桌面不同的占位文案

## 已知边界

- 假后端只提供固定 JSON，不模拟真实 ROS 数据流；不做图像/地图渲染断言
- 尚未覆盖：多航点提交、初始位姿点击地图、遥控使能链路（需要更完整的假数据流）
