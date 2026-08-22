#!/usr/bin/env python3
"""Validate a Walker ONNX policy against the deployment contract."""

from __future__ import annotations

import argparse
from pathlib import Path


EXPECTED_INPUT = 1477
EXPECTED_OUTPUT = 30


def validate_policy(path: Path) -> None:
    import onnxruntime as ort

    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError("Policy must expose exactly one input and one output")

    input_shape = inputs[0].shape
    output_shape = outputs[0].shape
    if len(input_shape) != 2 or input_shape[-1] != EXPECTED_INPUT:
        raise ValueError(f"Expected input shape [batch, {EXPECTED_INPUT}], got {input_shape}")
    if len(output_shape) != 2 or output_shape[-1] != EXPECTED_OUTPUT:
        raise ValueError(f"Expected output shape [batch, {EXPECTED_OUTPUT}], got {output_shape}")
    print(f"valid Walker policy: input={input_shape}, output={output_shape}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("policy", type=Path)
    args = parser.parse_args()
    validate_policy(args.policy)


if __name__ == "__main__":
    main()