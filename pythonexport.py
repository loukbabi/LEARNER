import onnx
from onnxconverter_common import (
    float16,
)  # Ensure you have onnxconverter-common installed: pip install onnxconverter-common

# Load your FP32 ONNX model
fp32_model_path = "yolov8n.onnx"
model = onnx.load(fp32_model_path)

# Convert the model to FP16
model_fp16 = float16.convert_float_to_float16(model)

# Save the FP16 model
fp16_model_path = "yolov8n_fp16.onnx"
onnx.save(model_fp16, fp16_model_path)
print(f"Model converted and saved to {fp16_model_path}")
