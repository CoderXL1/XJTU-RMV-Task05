# RMV Task05 大作业
**使用 MVS 取流 + ROS2 管理节点 + OpenCV 识别装甲板 + LeNet 识别装甲板编号/图案**

-----------
## 📁 项目结构
```
XJTU-RMV-Task05/ 
├── README.md 
├── XJTU-RMV-Task05.code-workspace
└── src
    ├── detect_and_solve
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   ├── classes.json
    │   │   └── config.json
    │   ├── include/detect_and_solve/
    │   ├── launch/
    │   ├── package.xml
    │   └── src
    │       ├── model_runtime.cpp
    │       ├── node.cpp
    │       ├── recognition.cpp
    │       └── util.cpp
    ├── hik_camera
    │   ├── CMakeLists.txt
    │   ├── include/hik_camera/
    │   ├── launch/
    │   ├── package.xml
    │   └── src
    │       ├── hik_camera_driver.cpp
    │       └── hik_camera_node.cpp
    └── training
        ├── dataset/
        ├── dataset_split/
		├── classes.json
        ├── export_model.py
        ├── sort_dataset.py
        ├── train.py
		├── best_model.pth
        ├── tiny_lenet18.onnx.data
        ├── tiny_lenet18.onnx
        └── tiny_lenet.onnx
```

---------
## 🛠️ 特性
#### 功能
✅ 从视频或相机取流，稳定识别装甲板

✅ 通过相机标定和 PnP 解算，从识别结果得到装甲板位姿

✅ 通过 LeNet 识别装甲板编号/图案

✅ 使用 ROS2 整合相机驱动和识别部分，并在 ROS2 中发布识别结果
#### 使用库
`OpenCV`
`MVS`
`ROS2-humble`
`PyTorch`
`ONNX`
`nlohmann-json`

-----------
## 🧩 开发过程
**耗时：1 × 通宵+ 3 × 白天**
1. 利用 Task04 的 HikRobot Camera Driver，稍作适配；
2. 利用极少量 Task02 的代码作为装甲板识别基础代码；
3. 在 `node.cpp` 中实现了 订阅相机 topic 和 视频取流 两种入口，以及相机标定参数（内参矩阵、畸变系数）的读入和处理；
4. 在 `recognition.cpp` 中实现了 对帧图像中灯条和装甲板数字的分别识别，以及灯条的方向的精确测算；
5. 在 `node.cpp` 中实现了根据灯条和装甲板识别结果，将数字部分使用 `model_runtime.cpp` 进行识别并发布识别结果，发布框选的图像，并发布通过 pnp 解算得到的装甲板位姿；
6. 使用 `train.py` 训练模型，并通过 `export_model.py` 转换为 onnx 格式以适配 OpenCV DNN Runtime；
7. 在 `model_runtime.cpp` 中实现了帧实时级别的预处理和推理
------------
## 🔥 主要优化
1. 使用 scale 系数将帧图像分辨率降低，提高处理速度，再将识别出的坐标重新放大得到正确结果
2. 使用 C++ OpenCV DNN Runtime 加快推理速度与集成度，而非新 Package + Python 方案
3. 将 装甲板数字/图案识别候选结果 和 灯条识别候选结果 交叉匹配，提升识别准确度
4. 识别模型的输入头分辨率降低到 32x48，加快训练和推理速度
5. 推理时基于全局亮度动态选择阈值，以二值化图案，提升模型识别准度
