#!/usr/bin/env python3
"""MuJoCo 相机视场（FOV）查看工具。

在 MuJoCo 查看器中打开模型并启用 `mjVIS_CAMERA`，即可从外部看到相机视锥；
加 `--through` 可直接切换到该相机的第一人称视角（相当于看相机实际画面）。
终端会同时打印相机的世界位姿、前向/上向、水平/垂直 FOV 以及 1 m 处视锥角点，
方便核对装配位置是否正确。

用法：
    python3 utils/view_camera_fov.py <模型.xml>
    python3 utils/view_camera_fov.py <模型.xml> --cam depth_cam --through
    python3 utils/view_camera_fov.py <模型.xml> --info-only     # 只打印参数，不开窗口

说明：
- 模型默认位姿为 XML 中的初始状态（qpos0）；如需其它位姿，可在查看器中暂停后查看。
- 视锥由 MuJoCo 查看器的 mjVIS_CAMERA 标志绘制，不会写回 XML。
"""
import argparse
import math
import time

import mujoco
import mujoco.viewer
import numpy as np


def cam_fovs(model, i):
    """返回 (垂直 FOV, 水平 FOV)，单位度。"""
    fovy = float(model.cam_fovy[i])
    w, h = model.cam_resolution[i]
    if w <= 0 or h <= 0:          # 未声明分辨率时按 4:3 估算水平视场
        w, h = 4, 3
    fovx = 2.0 * math.degrees(math.atan(math.tan(math.radians(fovy / 2.0)) * w / h))
    return fovy, fovx


def main():
    ap = argparse.ArgumentParser(description="查看 MuJoCo 模型的相机视场")
    ap.add_argument("xml", help="模型 XML 路径")
    ap.add_argument("--cam", default=None, help="相机名（默认第一个）")
    ap.add_argument("--through", action="store_true", help="切换到该相机的第一人称视角")
    ap.add_argument("--distance", type=float, default=1.0, help="打印视锥角点所用的距离（m，默认 1.0）")
    ap.add_argument("--info-only", action="store_true", help="只打印相机参数，不启动查看器")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if model.ncam == 0:
        raise SystemExit("模型中没有任何相机")

    cam_id = 0
    if args.cam:
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, args.cam)
        if cam_id < 0:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(model.ncam)]
            raise SystemExit(f"找不到相机 '{args.cam}'，可用相机：{names}")

    print(f"模型：{args.xml}")
    for i in range(model.ncam):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.cam_bodyid[i])
        pos = np.array(data.cam_xpos[i])
        R = data.cam_xmat[i].reshape(3, 3)      # 列向量 = 相机 x/y/z 轴
        right, up, back = R[:, 0], R[:, 1], R[:, 2]
        fwd = -back                              # MuJoCo 相机沿 -z 观察
        fovy, fovx = cam_fovs(model, i)
        w, h = model.cam_resolution[i]

        mark = "  <-- 选中" if i == cam_id else ""
        print(f"\n[{i}] {name}  父 body={body}{mark}")
        print(f"    世界位置 = {np.round(pos, 6)}")
        print(f"    前向     = {np.round(fwd, 6)}")
        print(f"    上向     = {np.round(up, 6)}   右向 = {np.round(right, 6)}")
        print(f"    FOV      = 垂直 {fovy:.2f}° / 水平 {fovx:.2f}°   分辨率 {w}x{h}")

        d = args.distance
        hw = d * math.tan(math.radians(fovx / 2.0))
        hh = d * math.tan(math.radians(fovy / 2.0))
        center = pos + d * fwd
        corners = [center + sx * hw * right + sy * hh * up
                   for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        print(f"    {d:g} m 处视锥角点：")
        for c in corners:
            print(f"      {np.round(c, 6)}")

    if args.info_only:
        return

    print("\n启动查看器：相机视锥由 mjVIS_CAMERA 绘制；关闭窗口即退出。")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CAMERA] = True
        if args.through:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = cam_id
        viewer.sync()
        while viewer.is_running():
            viewer.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()
