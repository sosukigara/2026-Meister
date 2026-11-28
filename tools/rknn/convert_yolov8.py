"""Convert Rockchip-optimised YOLOv8 / YOLO11 / Open-Vocab ONNX to RKNN for RK3588.

Examples:
    # Standard INT8 YOLOv8s/m or YOLO11s/m (Rockchip model-zoo export)
    python3 tools/rknn/convert_yolov8.py \
      --onnx /tmp/yolov8m.onnx --dataset /tmp/coco_eval/calib.txt \
      --out ros/src/umeonigiri_vision/models/yolov8m-zoo.rknn

    # FP16 conversion (higher small-object recall, still 25+ FPS on 3-core NPU pool)
    python3 tools/rknn/convert_yolov8.py \
      --onnx /tmp/yolo11s.onnx --quant fp16 \
      --out ros/src/umeonigiri_vision/models/yolo11s-fp16.rknn
"""
import argparse

from rknn.api import RKNN


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--dataset", default="", help="Calibration image list (required for i8)")
    parser.add_argument("--out", required=True)
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--quant", choices=["i8", "fp16"], default="i8")
    parser.add_argument("--platform", default="rk3588")
    parser.add_argument("--opt-level", type=int, default=3)
    args = parser.parse_args()

    do_quant = args.quant == "i8"
    if do_quant and not args.dataset:
        parser.error("--dataset is required when --quant i8")

    rknn = RKNN(verbose=False)
    config_kwargs = dict(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        target_platform=args.platform,
        optimization_level=args.opt_level,
    )
    if do_quant:
        config_kwargs["quantized_dtype"] = "w8a8"
        config_kwargs["quantized_algorithm"] = "normal"
    rknn.config(**config_kwargs)

    assert rknn.load_onnx(
        model=args.onnx,
        input_size_list=[[1, args.input_size, args.input_size, 3]],
    ) == 0
    assert rknn.build(
        do_quantization=do_quant,
        dataset=args.dataset if do_quant else None,
    ) == 0
    assert rknn.export_rknn(args.out) == 0
    rknn.release()
    print(f"saved {args.out} ({args.platform}, {args.quant}, {args.input_size}x{args.input_size})")


if __name__ == "__main__":
    main()
