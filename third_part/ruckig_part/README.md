# ruckig_part

从 [Ruckig](https://github.com/pantor/ruckig) 提取的**离线位置轨迹**子集，对应上游 `examples/02_position_offline`。

## 功能

- `Ruckig::calculate()`：一次性计算点到点轨迹
- `Trajectory::get_duration()` / `at_time()` / `get_position_extrema()`
- 支持 `min_velocity`、`min_acceleration` 等非对称约束
- 固定自由度模板 `Ruckig<N>` 与运行时自由度 `Ruckig<DynamicDOFs>`

不包含：在线 `update()` 循环、中间路径点、Cloud API、Trackig、速度控制接口示例等。

## 构建

```bash
cd ruckig_part
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
./example_position_offline
```

在其他 CMake 工程中：

```cmake
add_subdirectory(path/to/ruckig_part)
target_link_libraries(your_target PRIVATE ruckig_part::ruckig_part)
```

## 头文件

```cpp
#include <ruckig/ruckig.hpp>
```

## 许可

MIT，与上游 Ruckig 相同（见 `LICENSE`）。
