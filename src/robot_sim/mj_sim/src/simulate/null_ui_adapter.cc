// null_ui_adapter.cc
//
// 无窗口（headless）UI 适配器的实现：所有接口空操作，绝不接触 OpenGL/GLFW。

#include "null_ui_adapter.h"

#include <cstring>

namespace mujoco {

NullUIAdapter::NullUIAdapter() {
  // 基类构造不会清零 state_（GLFW 路径是在 RenderLoop 里 memset 的），
  // 这里显式清零，保证任何读取 UI 状态的代码拿到确定值。
  std::memset(&state_, 0, sizeof(state_));
}

std::pair<double, double> NullUIAdapter::GetCursorPosition() const { return {0.0, 0.0}; }

double NullUIAdapter::GetDisplayPixelsPerInch() const { return 96.0; }

std::pair<int, int> NullUIAdapter::GetFramebufferSize() const { return {0, 0}; }

std::pair<int, int> NullUIAdapter::GetWindowSize() const { return {0, 0}; }

bool NullUIAdapter::IsGPUAccelerated() const { return false; }

void NullUIAdapter::PollEvents() {}

void NullUIAdapter::SetClipboardString(const char* text) { (void)text; }

void NullUIAdapter::SetVSync(bool enabled) { (void)enabled; }

void NullUIAdapter::SetWindowTitle(const char* title) { (void)title; }

// 窗口永远不"应该关闭"：headless 的退出由 Simulate::exitrequest 驱动。
bool NullUIAdapter::ShouldCloseWindow() const { return false; }

void NullUIAdapter::SwapBuffers() {}

void NullUIAdapter::ToggleFullscreen() {}

bool NullUIAdapter::IsLeftMouseButtonPressed() const { return false; }

bool NullUIAdapter::IsMiddleMouseButtonPressed() const { return false; }

bool NullUIAdapter::IsRightMouseButtonPressed() const { return false; }

bool NullUIAdapter::IsAltKeyPressed() const { return false; }

bool NullUIAdapter::IsCtrlKeyPressed() const { return false; }

bool NullUIAdapter::IsShiftKeyPressed() const { return false; }

bool NullUIAdapter::IsMouseButtonDownEvent(int act) const {
  (void)act;
  return false;
}

bool NullUIAdapter::IsKeyDownEvent(int act) const {
  (void)act;
  return false;
}

int NullUIAdapter::TranslateKeyCode(int key) const {
  (void)key;
  return 0;
}

mjtButton NullUIAdapter::TranslateMouseButton(int button) const {
  (void)button;
  return mjBUTTON_LEFT;
}

bool NullUIAdapter::RefreshMjrContext(const mjModel* m, int fontscale) {
  // 绝不调用 mjr_makeContext：headless 下不允许创建 OpenGL 上下文。
  (void)m;
  (void)fontscale;
  return false;
}

}  // namespace mujoco
