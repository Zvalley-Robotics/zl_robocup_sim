// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_SIMULATE_NULL_UI_ADAPTER_H_
#define MUJOCO_SIMULATE_NULL_UI_ADAPTER_H_

#include <utility>

#include <mujoco/mujoco.h>

#include "platform_ui_adapter.h"

namespace mujoco {

// 无窗口（headless）模式下的 UI 适配器：所有接口都是空实现，全程不接触
// OpenGL/GLFW。用途：
//   - 没有可用 GL 环境（如 WSL2 的 WSLg d3d12/swrast 栈在渲染负载下崩溃）；
//   - 纯批处理/联调仿真：只要物理、ROS 话题和 DDS，不需要窗口。
//
// 挂到 Simulate 上后：没有窗口、没有渲染线程、没有深度相机像素，但
// Simulate::mtx / exitrequest / 深度相机槽位配置等无 GL 能力全部保留。
//
// 注意：RefreshMjrContext 必须保持返回 false——基类默认实现会调用
// mjr_makeContext 创建 OpenGL 上下文，headless 下绝不能执行。
class NullUIAdapter : public PlatformUIAdapter {
 public:
  NullUIAdapter();
  ~NullUIAdapter() override = default;

  std::pair<double, double> GetCursorPosition() const override;
  double GetDisplayPixelsPerInch() const override;
  std::pair<int, int> GetFramebufferSize() const override;
  std::pair<int, int> GetWindowSize() const override;
  bool IsGPUAccelerated() const override;
  void PollEvents() override;
  void SetClipboardString(const char* text) override;
  void SetVSync(bool enabled) override;
  void SetWindowTitle(const char* title) override;
  bool ShouldCloseWindow() const override;
  void SwapBuffers() override;
  void ToggleFullscreen() override;

  bool IsLeftMouseButtonPressed() const override;
  bool IsMiddleMouseButtonPressed() const override;
  bool IsRightMouseButtonPressed() const override;

  bool IsAltKeyPressed() const override;
  bool IsCtrlKeyPressed() const override;
  bool IsShiftKeyPressed() const override;

  bool IsMouseButtonDownEvent(int act) const override;
  bool IsKeyDownEvent(int act) const override;

  int TranslateKeyCode(int key) const override;
  mjtButton TranslateMouseButton(int button) const override;

  // 覆盖基类默认实现：不创建 GL 上下文（mjr_makeContext）。
  bool RefreshMjrContext(const mjModel* m, int fontscale) override;
};

}  // namespace mujoco

#endif  // MUJOCO_SIMULATE_NULL_UI_ADAPTER_H_
