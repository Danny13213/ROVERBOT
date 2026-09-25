# YOLO models

Place the TensorRT engine here locally:

`~/ROVERBOT/models/yolo11n.engine`

Model binaries (`*.engine`, `*.pt`, `*.onnx`) are intentionally ignored by Git.

Export the TensorRT engine on the Jetson so it matches the JetPack/TensorRT installation. The C++ node expects a raw Ultralytics detection output rather than an engine with NMS embedded.
