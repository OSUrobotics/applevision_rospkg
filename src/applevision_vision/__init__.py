from pathlib import Path as _Path

_self_path = _Path(__file__)
MODEL_PATH = _self_path.parent.joinpath('best.onnx')
