// MujocoSimBase.cpp
#include "mj_sim/MujocoSimBase.h"

#include <pthread.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#if defined(__linux__)
#include <dirent.h>
#include <glob.h>
#endif
#include <unistd.h>

#include "simulate/glfw_adapter.h"
#include "simulate/null_ui_adapter.h"

namespace {
const int kErrorLength = 1024;

// UI 字体档位上限：Simulate 的 "Font" 选择项提供 50%~300%，对应档位 0..5。
constexpr int kMaxFontIndex = 5;

// 把目标实时倍率（百分比）吸附到 Simulate 的档位表上。与
// Simulate::LoadOnRenderThread 里"依据 XML vis/global realtime 选档"用同一套
// 对数尺度度量，保证 YAML 给 50 时落到的就是 50% 那一档。
int NearestRealtimeIndex(double percent) {
  constexpr int kNumClicks =
      static_cast<int>(sizeof(mj::Simulate::percentRealTime) / sizeof(mj::Simulate::percentRealTime[0]));
  const double desired = std::log(percent);
  int best = 0;
  double min_error = std::numeric_limits<double>::max();
  for (int i = 0; i < kNumClicks; ++i) {
    const double error = std::abs(std::log(static_cast<double>(mj::Simulate::percentRealTime[i])) - desired);
    if (error < min_error) {
      min_error = error;
      best = i;
    }
  }
  return best;
}

// ---- GPU 渲染后端（NVIDIA PRIME render offload）----
//
// 窗口默认走 Intel iGPU 时，iGPU/Xorg 渲染会和单线程物理线程争 CPU，
// 表现为窗口掉帧、200 Hz 状态流掉节拍。这里在创建 GLFW/GLX 之前设置
// __NV_PRIME_RENDER_OFFLOAD / __GLX_VENDOR_LIBRARY_NAME，把窗口切到独显。
constexpr int kGpuOffloadDisabled = 0;
constexpr int kGpuOffloadAuto = 1;
constexpr int kGpuOffloadForced = 2;
constexpr const char* kPrimeOffloadEnv = "__NV_PRIME_RENDER_OFFLOAD";
constexpr const char* kGlxVendorEnv = "__GLX_VENDOR_LIBRARY_NAME";

const char* EnvNonEmpty(const char* name) {
  const char* value = std::getenv(name);
  return (value != nullptr && value[0] != '\0') ? value : nullptr;
}

bool StrCaseEqual(const char* value, const char* expected) {
  if (value == nullptr || expected == nullptr) {
    return false;
  }
  while (*value != '\0' && *expected != '\0') {
    if (std::tolower(static_cast<unsigned char>(*value)) !=
        std::tolower(static_cast<unsigned char>(*expected))) {
      return false;
    }
    ++value;
    ++expected;
  }
  return *value == '\0' && *expected == '\0';
}

bool EnvIsFalsy(const char* value) {
  return value != nullptr &&
         (std::strcmp(value, "0") == 0 || StrCaseEqual(value, "false") ||
          StrCaseEqual(value, "off") || StrCaseEqual(value, "no"));
}

// 只接受本地 Unix socket 形式：":N" 或 "unix:N"。远程/SSH 转发显示
// （host:N、localhost:N）视为不可用，避免在 ssh -X 下误启用 PRIME。
bool ParseLocalDisplay(const char* display, long* number) {
  if (display == nullptr || number == nullptr) {
    return false;
  }
  const char* colon = std::strchr(display, ':');
  if (colon == nullptr) {
    return false;
  }
  const std::size_t prefix_len = static_cast<std::size_t>(colon - display);
  const bool local = (prefix_len == 0) ||
                     (prefix_len == 4 && std::strncmp(display, "unix", 4) == 0);
  if (!local) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(colon + 1, &end, 10);
  if (end == colon + 1 || errno != 0 || parsed < 0 || parsed > 65535 ||
      (*end != '\0' && *end != '.')) {
    return false;
  }
  *number = parsed;
  return true;
}

#if defined(__linux__)
bool GlobHasMatch(const char* pattern) {
  glob_t matches{};
  const int rc = ::glob(pattern, 0, nullptr, &matches);
  const bool found = (rc == 0 && matches.gl_pathc > 0);
  ::globfree(&matches);
  return found;
}

// 扫描 /proc，确认该显示号是否由 Xvfb 提供。覆盖 xvfb-run，以及
// scripts/run.sh 在没有 DISPLAY 时直接启动的 "Xvfb :99"。
bool XvfbOwnsDisplay(long display_number) {
  DIR* proc = ::opendir("/proc");
  if (proc == nullptr) {
    return false;
  }
  bool found = false;
  while (const dirent* entry = ::readdir(proc)) {
    if (entry->d_name[0] < '0' || entry->d_name[0] > '9') {
      continue;
    }
    char path[512];
    std::snprintf(path, sizeof(path), "/proc/%s/comm", entry->d_name);
    FILE* comm = std::fopen(path, "r");
    if (comm == nullptr) {
      continue;
    }
    char name[32] = {};
    const bool is_xvfb = std::fgets(name, sizeof(name), comm) != nullptr &&
                         std::strncmp(name, "Xvfb", 4) == 0;
    std::fclose(comm);
    if (!is_xvfb) {
      continue;
    }

    std::snprintf(path, sizeof(path), "/proc/%s/cmdline", entry->d_name);
    FILE* cmdline = std::fopen(path, "r");
    if (cmdline == nullptr) {
      continue;
    }
    char args[4096] = {};
    const std::size_t size = std::fread(args, 1, sizeof(args) - 1, cmdline);
    std::fclose(cmdline);
    for (std::size_t i = 0; i < size;) {
      const char* arg = args + i;
      const std::size_t len = std::strlen(arg);
      long parsed = -1;
      if (ParseLocalDisplay(arg, &parsed) && parsed == display_number) {
        found = true;
        break;
      }
      i += len + 1;
    }
    if (found) {
      break;
    }
  }
  ::closedir(proc);
  return found;
}
#endif

bool IsXvfbDisplay(long display_number) {
  const char* xauthority = EnvNonEmpty("XAUTHORITY");
  if (xauthority != nullptr && std::strstr(xauthority, "xvfb-run") != nullptr) {
    return true;
  }
#if defined(__linux__)
  return XvfbOwnsDisplay(display_number);
#else
  (void)display_number;
  return false;
#endif
}

bool IsWaylandSession() {
  const char* session = EnvNonEmpty("XDG_SESSION_TYPE");
  if (session != nullptr) {
    return std::strcmp(session, "x11") != 0;
  }
  return EnvNonEmpty("WAYLAND_DISPLAY") != nullptr;
}

// 返回 nullptr 表示可以启用；否则返回跳过原因。
const char* AutoSkipReason() {
  const char* display = EnvNonEmpty("DISPLAY");
  if (display == nullptr) {
    return "DISPLAY not set";
  }
  long number = 0;
  if (!ParseLocalDisplay(display, &number)) {
    return "remote/non-local DISPLAY";
  }
  if (IsXvfbDisplay(number)) {
    return "virtual X display (Xvfb)";
  }
  if (IsWaylandSession()) {
    return "Wayland/non-X11 session";
  }
  if (::access("/dev/nvidia0", R_OK | W_OK) != 0) {
    return "/dev/nvidia0 missing or inaccessible";
  }
#if defined(__linux__)
  if (!GlobHasMatch("/usr/lib/*/libGLX_nvidia.so*") &&
      !GlobHasMatch("/usr/lib64/libGLX_nvidia.so*") &&
      !GlobHasMatch("/usr/lib/libGLX_nvidia.so*")) {
    return "libGLX_nvidia not found";
  }
#endif
  return nullptr;
}
}  // namespace

void ApplyGpuOffloadEnvironment(const SimViewerOptions& options) {
  // 进程内只应用一次：构造阶段调用后，Run() 里的兜底调用直接返回。
  static std::atomic<bool> applied{false};
  if (applied.exchange(true)) {
    return;
  }

  int mode = options.gpu_offload;
  if (mode < kGpuOffloadDisabled || mode > kGpuOffloadForced) {
    std::cout << "[mj_sim] viewer.gpu_offload=" << mode
              << " invalid (expected 0..2), falling back to auto" << std::endl;
    mode = kGpuOffloadAuto;
  }

  const char* preset_offload = EnvNonEmpty(kPrimeOffloadEnv);
  const char* preset_vendor = EnvNonEmpty(kGlxVendorEnv);

  if (mode == kGpuOffloadDisabled) {
    const bool had_env = preset_offload != nullptr || preset_vendor != nullptr;
    ::unsetenv(kPrimeOffloadEnv);
    ::unsetenv(kGlxVendorEnv);
    std::cout << "[mj_sim] viewer.gpu_offload=0: PRIME render offload disabled"
              << (had_env ? " (external env cleared)" : "") << std::endl;
    return;
  }

  // 外部已显式设置的环境变量优先，只补齐缺失的一半。
  if (preset_offload != nullptr && EnvIsFalsy(preset_offload)) {
    std::cout << "[mj_sim] viewer.gpu_offload=" << mode
              << ": external env override kept (" << kPrimeOffloadEnv << "="
              << preset_offload << ")" << std::endl;
    return;
  }
  if (preset_vendor != nullptr) {
    if (preset_offload == nullptr && StrCaseEqual(preset_vendor, "nvidia")) {
      ::setenv(kPrimeOffloadEnv, "1", 1);
    }
    std::cout << "[mj_sim] viewer.gpu_offload=" << mode
              << ": external env override kept (" << kGlxVendorEnv << "="
              << preset_vendor << ")" << std::endl;
    return;
  }
  if (preset_offload != nullptr) {
    ::setenv(kGlxVendorEnv, "nvidia", 1);
    std::cout << "[mj_sim] viewer.gpu_offload=" << mode
              << ": external env override kept (" << kPrimeOffloadEnv << "="
              << preset_offload << ")" << std::endl;
    return;
  }

  if (mode == kGpuOffloadAuto) {
    if (const char* reason = AutoSkipReason(); reason != nullptr) {
      std::cout << "[mj_sim] viewer.gpu_offload=1: PRIME render offload skipped ("
                << reason << ")" << std::endl;
      return;
    }
  } else {
    // 强制模式仍然设置，但对明显不可用的环境给出警告。
    const char* display = EnvNonEmpty("DISPLAY");
    if (display == nullptr) {
      std::cout << "[mj_sim] viewer.gpu_offload=2: DISPLAY is unset; GLFW may fail"
                << std::endl;
    } else {
      long number = 0;
      if (ParseLocalDisplay(display, &number) && IsXvfbDisplay(number)) {
        std::cout << "[mj_sim] viewer.gpu_offload=2: forcing PRIME offload on Xvfb; "
                     "GLFW may fail"
                  << std::endl;
      }
    }
  }

  const int offload_rc = ::setenv(kPrimeOffloadEnv, "1", 1);
  const int vendor_rc = ::setenv(kGlxVendorEnv, "nvidia", 1);
  if (offload_rc != 0 || vendor_rc != 0) {
    std::cout << "[mj_sim] viewer.gpu_offload=" << mode
              << ": failed to set PRIME offload environment variables" << std::endl;
    return;
  }
  std::cout << "[mj_sim] viewer.gpu_offload=" << mode
            << ": PRIME render offload enabled (" << kPrimeOffloadEnv
            << "=1, " << kGlxVendorEnv << "=nvidia)" << std::endl;
}

MujocoSimBase* MujocoSimBase::controller_instance_ = nullptr;

MujocoSimBase::MujocoSimBase() : m_(nullptr), d_(nullptr), is_record_(false), running_(false) {
  // MuJoCo 控制回调无法携带用户数据，这里保存当前实例，供 StaticComputeControl 转发。
  controller_instance_ = this;
}

MujocoSimBase::~MujocoSimBase() {
  std::cout << "sim t4 end." << std::endl;
  running_ = false;
  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  if (controller_instance_ == this) {
    controller_instance_ = nullptr;
  }

  if (d_)
    mj_deleteData(d_);
  if (m_)
    mj_deleteModel(m_);
}

void MujocoSimBase::Run() {
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    // 头文件和动态库版本不一致会导致结构体布局不匹配，必须直接终止。
    mju_error("Headers and library have different versions");
  }

  // 扫描与可执行文件同目录下的 MuJoCo 插件，支持后续扩展自定义 actuator/sensor。
  ScanPluginLibraries();

  // 无窗口（headless）模式：不创建窗口、不接触 OpenGL，物理循环直接跑在
  // 当前（主）线程。WSL2 下 WSLg 的 d3d12/swrast 栈在渲染负载下会崩
  // （D3D12: Removing Device -> SIGSEGV，甚至拖死宿主机），默认自动启用。
  if (DetectHeadlessMode()) {
    headless_ = true;
    std::printf("[mj_sim] headless mode: no window, no OpenGL; physics/ROS keep running.\n");
    std::printf("[mj_sim] set ZL_ROBOT_HEADLESS=0 to force the GLFW window instead.\n");
    RunHeadless();
    return;
  }

  // 窗口/GPU 路径：PRIME offload 环境变量必须在创建 GLFW 窗口之前设置。
  // 正常路径已在派生类构造阶段应用；此处对未来的直接派生类再调用一次，
  // 函数内的一次性 guard 保证不会重复 setenv/打印。
  ::ApplyGpuOffloadEnvironment(viewer_options_);

  mjvCamera cam;
  mjv_defaultCamera(&cam);
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  sim_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam, &opt, &pert, false);
  // 必须在 RenderLoop() 之前应用：UI 主题/字体/垂直同步由 RenderLoop 启动时读入。
  ApplyStartupOptions();

  const char* filename = xml_path.c_str();
  std::cout << "It will load xml file at path: " << filename << std::endl;
  running_ = true;
  // 物理线程负责加载模型和推进仿真；当前线程进入 GUI 渲染循环。
  physics_thread_ = std::thread(&MujocoSimBase::RunPhysicsThread, this, filename);
  // 线程名便于用 /proc 或调试器分辨物理/渲染/ROS 线程（同名进程下只有一个主线程名）。
  pthread_setname_np(physics_thread_.native_handle(), "mj_physics");

  sim_->RenderLoop();
  running_ = false;

  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }

  Cleanup();
}

bool MujocoSimBase::DetectHeadlessMode() {
  // 1) 环境变量显式开关优先：ZL_ROBOT_HEADLESS=0/false/off/no 强制窗口模式，
  //    其它非空值（1/true/on/yes/...）强制 headless。
  const char* env = std::getenv("ZL_ROBOT_HEADLESS");
  if (env != nullptr && env[0] != '\0') {
    std::string value(env);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (value == "0" || value == "false" || value == "off" || value == "no") {
      return false;
    }
    return true;
  }

  // 2) 未显式设置时按环境自动判定：WSL2 的 WSLg 走 Mesa d3d12/swrast 翻译
  //    层，在 MuJoCo 窗口渲染负载下会丢 D3D12 设备并段错误，因此 WSL2
  //    默认 headless。需要窗口画面请到原生 Ubuntu 运行。
  std::ifstream version("/proc/version");
  std::string line;
  if (std::getline(version, line)) {
    std::transform(line.begin(), line.end(), line.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (line.find("microsoft") != std::string::npos) {
      return true;
    }
  }
  return false;
}

void MujocoSimBase::RunHeadless() {
  const char* filename = xml_path.c_str();
  std::cout << "It will load xml file at path: " << filename << std::endl;

  running_ = true;

  // 先加载模型再创建 Simulate：headless 下没有窗口可以显示 "loading" 状态，
  // 也避免模型加载失败时留下半初始化的对象。
  m_ = LoadModel(filename);
  if (m_ != nullptr) {
    d_ = mj_makeData(m_);
  }

  mjvCamera cam;
  mjv_defaultCamera(&cam);
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjvPerturb pert;
  mjv_defaultPerturb(&pert);
  // 派生类仍然依赖 sim_ 的 mtx / exitrequest / 深度相机槽位等无 GL 能力，
  // 因此 headless 也要创建 Simulate，只是把 UI 适配器换成 NullUIAdapter。
  sim_ = std::make_unique<mj::Simulate>(std::make_unique<mj::NullUIAdapter>(), &cam, &opt, &pert,
                                        false);
  // headless 仍需应用 dynamics.start_paused/busywait/refresh_rate；viewer.* 不影响物理。
  ApplyStartupOptions();

  if (m_ != nullptr && d_ != nullptr) {
    // 初始姿态必须在 LoadHeadless 之前写入：加载时会拷贝一份 qpos 缓存和
    // 历史缓冲的初始状态。
    InitRobotState(d_);
    // 无窗口加载：只重建数据缓存与历史缓冲，不建场景/GL 上下文/UI。
    sim_->LoadHeadless(m_, d_, filename);
    // 加载完成后才能覆盖 LoadOnRenderThread 依据 XML 选出的实时倍率档位。
    ApplyPostLoadOptions();
    // 必须等模型加载成功后才能解析关节、执行器和传感器索引。
    InitInterface();
    InitCamera();
    // 根据初始状态做一次正向动力学，确保传感器和派生量有效。
    mj_forward(m_, d_);
  } else {
    std::printf("Model loading failed, simulation idles: %s\n", filename);
  }

  // 物理循环直接跑在主线程：没有渲染线程，也不会有人调用 RenderLoop。
  RunPhysicsLoop();

  running_ = false;
  Cleanup();
}

void MujocoSimBase::ApplyStartupOptions() {
  if (sim_ == nullptr) {
    return;
  }

  // ---- viewer：只影响窗口与界面显示 ----
  const SimViewerOptions& v = viewer_options_;
  sim_->ui0_enable = v.ui0_enable ? 1 : 0;
  sim_->ui1_enable = v.ui1_enable ? 1 : 0;
  sim_->spacing = v.spacing;
  sim_->color = v.color;
  sim_->help = v.help ? 1 : 0;
  sim_->info = v.info ? 1 : 0;
  sim_->profiler = v.profiler ? 1 : 0;
  sim_->sensor = v.sensor ? 1 : 0;
  sim_->vsync = v.vsync ? 1 : 0;
  sim_->pause_update = v.pause_update ? 1 : 0;
  if (v.font >= 0) {
    // 显式配置字体档位；-1（默认）时保持 RenderLoop 的 DPI 自动推断。
    if (v.font > kMaxFontIndex) {
      std::printf("viewer.font=%d 超出 0..%d，按 %d（300%%）处理\n", v.font, kMaxFontIndex, kMaxFontIndex);
    }
    sim_->font = std::min(v.font, kMaxFontIndex);
  }

  // ---- dynamics：只影响物理线程的推进节奏与运行状态 ----
  const SimDynamicsOptions& d = dynamics_options_;
  sim_->run = d.start_paused ? 0 : 1;
  sim_->busywait = d.busywait ? 1 : 0;
  sim_->refresh_rate = d.refresh_rate;
}

void MujocoSimBase::ApplyPostLoadOptions() {
  if (sim_ == nullptr) {
    return;
  }
  const double percent = dynamics_options_.realtime_percent;
  if (percent <= 0.0) {
    return;  // 未配置：沿用 XML 里 vis/global realtime 决定的档位。
  }

  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  sim_->real_time_index = NearestRealtimeIndex(percent);
  // 通知物理线程重新对齐同步基准，避免换档瞬间按新的目标倍率追赶历史步数。
  sim_->speed_changed = true;
  std::printf("dynamics.realtime_percent=%.1f%% -> Simulate 档位 %g%%\n", percent,
              static_cast<double>(mj::Simulate::percentRealTime[sim_->real_time_index]));
}

void MujocoSimBase::RunPhysicsThread(const char* filename) {
  if (filename != nullptr) {
    sim_->LoadMessage(filename);
    m_ = LoadModel(filename);
    if (m_) {
      // mjData 保存当前仿真状态，包括 qpos/qvel/ctrl/sensordata。
      d_ = mj_makeData(m_);
    }
    if (d_) {
      InitRobotState(d_);
      sim_->Load(m_, d_, filename);
      // 加载完成后才能覆盖 LoadOnRenderThread 依据 XML 选出的实时倍率档位。
      ApplyPostLoadOptions();
      // 必须等模型加载成功后才能解析关节、执行器和传感器索引。
      InitInterface();
      InitCamera();
      // 根据初始状态做一次正向动力学，确保传感器和派生量有效。
      mj_forward(m_, d_);
    } else {
      sim_->LoadMessageClear();
    }
  }

  RunPhysicsLoop();
}

void MujocoSimBase::StaticComputeControl(const mjModel* m, mjData* d) {
  // MuJoCo 调用的是全局函数指针，这里再转发到当前实例的虚函数。
  if (controller_instance_) {
    controller_instance_->ComputeControl(m, d);
    // 紧跟在写 ctrl 之后，赶在 MuJoCo 读取 d->ctrl 之前做后处理（执行延迟等）。
    controller_instance_->PostComputeControl(m, d);
  }
}

void MujocoSimBase::RunPhysicsLoop() {
  using Seconds = std::chrono::duration<double>;
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  double sync_sim = 0;

  // 注册 MuJoCo 控制回调。每次 mj_step 内部都会调用 StaticComputeControl。
  mjcb_control = StaticComputeControl;

  while (!sim_->exitrequest.load()) {
    if (sim_->uiloadrequest.load()) {
      // GUI 中重新加载模型时，重新创建 mjModel/mjData，并重建派生类接口索引。
      sim_->uiloadrequest.fetch_sub(1);
      sim_->LoadMessage(sim_->filename);
      mjModel* mnew = LoadModel(sim_->filename);
      mjData* dnew = nullptr;
      if (mnew)
        dnew = mj_makeData(mnew);
      if (dnew) {
        sim_->Load(mnew, dnew, sim_->filename);
        // GUI 重载同样会重置实时倍率档位，这里再应用一次。
        ApplyPostLoadOptions();
        mj_deleteData(d_);
        mj_deleteModel(m_);
        m_ = mnew;
        d_ = dnew;
        InitRobotState(d_);
        // 新模型的关节/执行器地址可能变化，必须重新初始化接口映射。
        InitInterface();
        InitCamera();
        mj_forward(m_, d_);
      } else {
        sim_->LoadMessageClear();
      }
    }

    // 根据 GUI busywait 设置选择让出 CPU 或短暂睡眠。
    if (sim_->run && sim_->busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      if (m_) {
        // 外部复位（/robot/sim/reset）：在物理线程内重置，避免与 mj_step 竞争。
        if (external_reset_requested_.exchange(false)) {
          mj_resetData(m_, d_);
          InitRobotState(d_);
          mj_forward(m_, d_);
          sync_sim = d_->time;
          syncCPU = mj::Simulate::Clock::now();
          sim_->speed_changed = false;
        }
        if (sim_->run) {
          // 在步进前给派生类机会发布状态、读取 ROS 命令或同步外部数据。
          ExchangeMessages(sync_sim);

          bool stepped = false;
          const auto startCPU = mj::Simulate::Clock::now();
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d_->time - sync_sim;
          double slowdown = 100 / sim_->percentRealTime[sim_->real_time_index];

          // 当仿真时间和 CPU 时间明显错位时，重置同步基准，避免追赶过多历史步。
          bool misaligned = std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim_->speed_changed) {
            syncCPU = startCPU;
            sync_sim = d_->time;
            sim_->speed_changed = false;
            stepped = StepAndRecord();
          } else {
            bool measured = false;
            double prevSim = d_->time;
            double refreshTime = simRefreshFraction / sim_->refresh_rate;

            // 在单个显示刷新周期内尽量推进足够多的物理步，使仿真接近目标实时倍率。
            while (Seconds((d_->time - sync_sim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              if (!measured && elapsedSim) {
                sim_->measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }
              stepped = StepAndRecord();
              if (d_->time < prevSim)
                break;
            }
          }

          if (stepped) {
            // 保存历史轨迹，供 MuJoCo GUI 中的可视化/曲线工具使用。
            sim_->AddToHistory();
          }
        } else {
          // 暂停时仍做 forward，保证拖拽、扰动或 UI 修改后传感器显示是最新的。
          mj_forward(m_, d_);
          sim_->speed_changed = true;
        }
      }
    }
  }
}

mjModel* MujocoSimBase::LoadModel(const char* file) {
  char load_err[kErrorLength] = "";
  mjModel* mnew = nullptr;

  // 支持加载二进制 MJB 或 XML。当前 robot_sim 默认使用 XML，便于调试模型配置。
  if (std::strlen(file) > 4 && !std::strncmp(file + std::strlen(file) - 4, ".mjb", 4)) {
    mnew = mj_loadModel(file, nullptr);
    if (!mnew) {
      std::strcpy(load_err, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(file, nullptr, load_err, kErrorLength);
  }

  if (load_err[0]) {
    std::printf("Model loading warning: %s\n", load_err);
  }

  return mnew;
}

void MujocoSimBase::ScanPluginLibraries() {
  // 先打印已经静态链接到可执行文件中的插件，便于确认插件注册状态。
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }
  const std::string sep = "/";

  // 再尝试扫描 ${EXECDIR}/mujoco_plugin。${EXECDIR} 是当前可执行文件所在目录。
  const std::string executable_dir = GetExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = GetExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

std::string MujocoSimBase::GetExecutableDir() {
  constexpr char kPathSep = '/';
  const char* path = "/proc/self/exe";

  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      ssize_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path 已经不是符号链接，直接使用。
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // 缓冲区不够，扩容后重试。
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }
  // 找不到可执行文件目录时返回空，避免从文件系统根目录开始扫描插件。
  return "";
}
