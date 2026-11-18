#!/usr/bin/env python3
"""把 FIFA Trionda 足球的 glTF 转成 MuJoCo 3.1.1 可加载的带 UV OBJ。

为什么需要转换
--------------
MuJoCo 3.1.1 的 <mesh> 只支持 OBJ/STL，不支持 glTF；且它的 <material> 只有
一张 2D 贴图（baseColor），没有法线/金属度贴图通道。因此这里把 scene.gltf 的
网格、UV 和节点变换烘焙成 OBJ，只保留 baseColor 贴图。

坐标/尺度
---------
glTF 网格顶点是局部坐标，场景里通过 Sketchfab_model / GLTF_SceneRootNode /
Trionda Ball 2026_0 三层节点缩放旋转。这里按节点链烘焙到世界坐标，再把顶点
平均半径归一化到 --radius（默认 0.11 m = 场地里碰撞球的半径）。网格是离散
球面（顶点半径 0.108~0.111），用平均半径可让视觉球与碰撞球整体重合。

UV 方向
-------
glTF 的 UV 原点在图像左上角（v 向下），OBJ/MuJoCo 的 v=0 对应图像底部，
所以写 OBJ 时按 v' = 1 - v 翻转；否则贴图会上下颠倒（FIFA/TRIONDA 字样镜像）。
U 方向两者一致，不需要翻转。

用法
----
    python3 robot_assets/utils/gen_trionda_ball_obj.py [--radius 0.11]

默认读/写：
    src/robot_assets/Z01_std_robocup/xml/fifa_trionda_ball_world_cup_2026/
        scene.gltf + scene.bin  ->  ball.obj
"""

import argparse
import json
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.dirname(SCRIPT_DIR)
BALL_DIR = os.path.join(
    ASSETS_DIR, "Z01_std_robocup", "xml", "fifa_trionda_ball_world_cup_2026")
DEFAULT_GLTF = os.path.join(BALL_DIR, "scene.gltf")
DEFAULT_OUT = os.path.join(BALL_DIR, "ball.obj")

COMPONENT_DTYPE = {
    5120: np.int8, 5121: np.uint8, 5122: np.int16,
    5123: np.uint16, 5125: np.uint32, 5126: np.float32,
}
TYPE_WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def read_accessor(gltf, binbuf, index):
  """读取一个 accessor，支持 bufferView 的交错存储（byteStride）。"""
  acc = gltf["accessors"][index]
  view = gltf["bufferViews"][acc["bufferView"]]
  width = TYPE_WIDTH[acc["type"]]
  dtype = COMPONENT_DTYPE[acc["componentType"]]
  offset = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
  stride = view.get("byteStride", 0)
  item_size = np.dtype(dtype).itemsize * width
  if stride and stride != item_size:
    rows = [np.frombuffer(binbuf, dtype=dtype, count=width,
                          offset=offset + i * stride)
            for i in range(acc["count"])]
    data = np.stack(rows)
  else:
    data = np.frombuffer(binbuf, dtype=dtype,
                         count=acc["count"] * width, offset=offset)
  return data.reshape(acc["count"], width)


def node_matrix(node):
  """节点局部矩阵（glTF matrix 为列主序；缺省为单位阵）。"""
  if "matrix" in node:
    return np.array(node["matrix"], dtype=np.float64).reshape(4, 4, order="F")
  mat = np.eye(4)
  if "translation" in node:
    mat[:3, 3] = node["translation"]
  if "rotation" in node:  # glTF 四元数顺序为 x y z w
    x, y, z, w = node["rotation"]
    mat[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
  if "scale" in node:
    mat[:3, :3] = mat[:3, :3] @ np.diag(node["scale"])
  return mat


def world_matrix(gltf, mesh_node_index):
  """从场景根沿父子链累乘到含 mesh 的节点。"""
  nodes = gltf["nodes"]
  parents = {}
  for i, node in enumerate(nodes):
    for child in node.get("children", []):
      parents[child] = i

  chain = []
  cursor = mesh_node_index
  while cursor is not None:
    chain.append(cursor)
    cursor = parents.get(cursor)
  chain.reverse()

  mat = np.eye(4)
  for i in chain:
    mat = mat @ node_matrix(nodes[i])
  return mat


def convert(gltf_path, out_path, radius):
  with open(gltf_path, encoding="utf-8") as handle:
    gltf = json.load(handle)
  bin_uri = gltf["buffers"][0]["uri"]
  with open(os.path.join(os.path.dirname(gltf_path), bin_uri), "rb") as handle:
    binbuf = handle.read()

  # 取第一个带 mesh 的节点及其 primitive。
  mesh_node = next(i for i, n in enumerate(gltf["nodes"]) if "mesh" in n)
  primitive = gltf["meshes"][gltf["nodes"][mesh_node]["mesh"]]["primitives"][0]
  attrs = primitive["attributes"]

  positions = read_accessor(gltf, binbuf, attrs["POSITION"]).astype(np.float64)
  normals = read_accessor(gltf, binbuf, attrs["NORMAL"]).astype(np.float64)
  texcoords = read_accessor(gltf, binbuf, attrs["TEXCOORD_0"]).astype(np.float64)
  indices = read_accessor(gltf, binbuf, primitive["indices"]).reshape(-1)

  matrix = world_matrix(gltf, mesh_node)
  rot, trans = matrix[:3, :3], matrix[:3, 3]
  positions = positions @ rot.T + trans
  normals = normals @ rot.T
  lengths = np.linalg.norm(normals, axis=1, keepdims=True)
  lengths[lengths == 0] = 1.0
  normals /= lengths

  # 归一化到碰撞球半径（按平均半径，离散网格的半径有 ±3mm 波动）。
  scale = radius / np.linalg.norm(positions, axis=1).mean()
  positions *= scale

  with open(out_path, "w", encoding="utf-8") as handle:
    handle.write("# FIFA TRIONDA Ball World Cup 2026 "
                 "(converted from scene.gltf, radius %.3f m)\n" % radius)
    handle.write("# source: https://sketchfab.com/3d-models/"
                 "fifa-trionda-ball-world-cup-2026-4c577717c59f44a882c48c5d8b5e41f8\n")
    handle.write("# license: CC-BY-4.0\n")
    for value in positions:
      handle.write("v %.6f %.6f %.6f\n" % tuple(value))
    for value in texcoords:
      # glTF 的 v 原点在图像顶部，OBJ/MuJoCo 在底部：写 OBJ 时翻转 V。
      handle.write("vt %.6f %.6f\n" % (value[0], 1.0 - value[1]))
    for value in normals:
      handle.write("vn %.6f %.6f %.6f\n" % tuple(value))
    for i in range(0, len(indices), 3):
      a, b, c = (int(indices[i + k]) + 1 for k in range(3))
      handle.write("f %d/%d/%d %d/%d/%d %d/%d/%d\n"
                   % (a, a, a, b, b, b, c, c, c))

  print("wrote %s: %d verts, %d tris, radius %.3f m"
        % (out_path, len(positions), len(indices) // 3, radius))


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--gltf", default=DEFAULT_GLTF, help="输入 scene.gltf")
  parser.add_argument("--out", default=DEFAULT_OUT, help="输出 OBJ")
  parser.add_argument("--radius", type=float, default=0.11, help="归一化半径 (m)")
  args = parser.parse_args()
  convert(args.gltf, args.out, args.radius)
  return 0


if __name__ == "__main__":
  sys.exit(main())
