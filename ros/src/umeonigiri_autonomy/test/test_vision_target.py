"""vision_target フラグメント差し込みの契約テスト。"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "simulation.launch.py"
WORLDS = PKG / "worlds"


def load_simulation_module():
    spec = importlib.util.spec_from_file_location("simulation_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_fragment_is_a_valid_model_with_one_texture_placeholder():
    fragment = (WORLDS / "vision_target.sdf").read_text(encoding="utf-8")
    assert fragment.count("__TEXTURE_PATH__") == 1
    root = ET.fromstring(
        "<sdf>" + fragment.replace("__TEXTURE_PATH__", "/tmp/bus.jpg") + "</sdf>")
    names = {child.attrib.get("name") for child in root}
    assert "vision_target_bus" in names
    assert "vision_target_light" in names


def test_injection_leaves_default_world_untouched():
    module = load_simulation_module()
    world = (WORLDS / "maze.sdf").read_text(encoding="utf-8")
    fragment = (WORLDS / "vision_target.sdf").read_text(encoding="utf-8")
    text, count = module.inject_vision_target(world, fragment, "/tmp/bus.jpg")
    assert count == 1
    assert "vision_target_bus" not in world
    assert "<model name=\"vision_target_bus\">" in text
    assert "/tmp/bus.jpg" in text
    assert "__TEXTURE_PATH__" not in text
    ET.fromstring(text)


def test_unresolved_texture_path_is_rejected():
    module = load_simulation_module()
    try:
        module.inject_vision_target(
            "<world></world>", "<model>__TEXTURE_PATH__</model>",
            "__TEXTURE_PATH__")
    except RuntimeError:
        return
    raise AssertionError("未解決のプレースホルダを検出できなかった")
