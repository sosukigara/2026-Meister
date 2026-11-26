"""Convert a Rockchip-optimised YOLOv8 ONNX to RKNN INT8 for RK3588.

    python3 tools/rknn/convert_yolov8.py \
      --onnx /tmp/yolov8s.onnx --dataset /tmp/coco_eval/calib.txt \
      --out /tmp/yolov8s-zoo.rknn

The ONNX must be the Rockchip model-zoo export (9 outputs).  The upstream
Ultralytics export fails graph fusion in rknn-toolkit2 (see README).
"""
import argparse

from rknn.api import RKNN


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rknn = RKNN(verbose=False)
    rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
                quantized_dtype="w8a8", target_platform="rk3588")
    assert rknn.load_onnx(model=args.onnx,
                          input_size_list=[[1, 640, 640, 3]]) == 0
    assert rknn.build(do_quantization=True, dataset=args.dataset) == 0
    assert rknn.export_rknn(args.out) == 0
    rknn.release()
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
