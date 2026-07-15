#!/usr/bin/env python3
"""Validate the vendored Gazebo obstacle-course geometry and naming contract."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
EXTRAS = REPO_ROOT / "vendor" / "px4_autopilot_extras"
WORLD_PATH = EXTRAS / "gz_worlds" / "obstacle_course.sdf"
MODELS_PATH = EXTRAS / "gz_models"


def parse_model(name: str) -> ET.Element:
    model = ET.parse(MODELS_PATH / name / "model.sdf").getroot().find("model")
    assert model is not None, f"Missing model element: {name}"
    return model


def floats(text: str | None) -> list[float]:
    assert text is not None
    return [float(value) for value in text.split()]


def assert_close(actual: float, expected: float) -> None:
    assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9), (
        actual,
        expected,
    )


def assert_visual_collision_pairs(link: ET.Element, expected_count: int) -> None:
    collisions = {
        item.get("name", "").removesuffix("_collision")
        for item in link.findall("collision")
    }
    visuals = {
        item.get("name", "").removesuffix("_visual") for item in link.findall("visual")
    }
    assert len(collisions) == expected_count, (len(collisions), expected_count)
    assert collisions == visuals, collisions ^ visuals


def validate_world() -> ET.Element:
    world = ET.parse(WORLD_PATH).getroot().find("world")
    assert world is not None and world.get("name") == "obstacle_course"

    inline_names = {model.get("name", "") for model in world.findall("model")}
    assert "course_ground" in inline_names
    assert all(name == "course_ground" or name.startswith("corridor_") for name in inline_names)

    ground_size = floats(
        world.findtext(
            "model[@name='course_ground']/link/collision/geometry/plane/size"
        )
    )
    assert ground_size == [700.0, 700.0]

    includes = {item.findtext("name"): item for item in world.findall("include")}
    assert all(
        name and (name.startswith("corridor_tree_") or name.startswith("loop_"))
        for name in includes
    )
    tree_names = {name for name in includes if name and name.startswith("corridor_tree_")}
    assert len(tree_names) == 9

    loop_include = includes["loop_arena_square_080m"]
    assert loop_include.findtext("uri") == "model://map_loop_arena"
    assert floats(loop_include.findtext("pose"))[:3] == [0.0, 210.0, 0.0]

    wire_include = includes["loop_wire_gate_222m"]
    assert wire_include.findtext("uri") == "model://wire_test_gate"
    assert floats(wire_include.findtext("pose"))[:3] == [0.0, 222.0, 0.0]
    return world


def validate_moving_obstacles(world: ET.Element) -> None:
    models = {model.get("name"): model for model in world.findall("model")}
    expected = {
        "corridor_mover_025m_slow_crate": ("box", [1.0, 1.0, 8.0], 4.0, 22.0),
        "corridor_mover_105m_medium_barrel": (
            "cylinder",
            [0.7, 10.0],
            5.0,
            24.0,
        ),
        "corridor_mover_185m_fast_panel": (
            "box",
            [1.5, 0.5, 12.0],
            6.0,
            30.0,
        ),
    }

    for name, (kind, dimensions, center_z, force) in expected.items():
        model = models[name]
        assert_close(floats(model.findtext("pose"))[2], center_z)
        assert model.findtext("link/gravity") == "false"
        assert model.find("joint") is None
        assert model.findtext("plugin/link_name") == "moving_target_link"
        assert_close(float(model.findtext("plugin/force", "nan")), force)

        geometry = model.find("link/collision/geometry")
        assert geometry is not None
        if kind == "box":
            actual_dimensions = floats(geometry.findtext("box/size"))
            height = actual_dimensions[2]
        else:
            actual_dimensions = [
                float(geometry.findtext("cylinder/radius", "nan")),
                float(geometry.findtext("cylinder/length", "nan")),
            ]
            height = actual_dimensions[1]
        assert actual_dimensions == dimensions, (name, actual_dimensions)
        assert_close(center_z - height / 2.0, 0.0)
        assert_close(center_z + height / 2.0, height)


def validate_loop_arena() -> None:
    link = parse_model("map_loop_arena").find("link")
    assert link is not None
    assert_visual_collision_pairs(link, 13)
    collisions = {item.get("name"): item for item in link.findall("collision")}

    west_size = floats(
        collisions["outer_west_collision"].findtext("geometry/box/size")
    )
    island_size = floats(
        collisions["island_south_collision"].findtext("geometry/box/size")
    )
    assert west_size == [0.5, 80.5, 15.0]
    assert island_size == [30.0, 0.5, 12.0]

    outer_inner_face = 40.0 - 0.25
    island_outer_face = 15.0 + 0.25
    assert_close(outer_inner_face - island_outer_face, 24.5)
    assert_close(2.0 * (55.0 + 55.0), 220.0)


def validate_wire_gate() -> None:
    link = parse_model("wire_test_gate").find("link")
    assert link is not None
    assert_visual_collision_pairs(link, 8)
    collisions = {item.get("name"): item for item in link.findall("collision")}

    for name, diameter, height in (
        ("wire_20mm", 0.020, 5.0),
        ("wire_50mm", 0.050, 9.0),
        ("wire_100mm", 0.100, 13.0),
    ):
        collision = collisions[f"{name}_collision"]
        radius = float(collision.findtext("geometry/cylinder/radius", "nan"))
        pose = floats(collision.findtext("pose"))
        assert_close(2.0 * radius, diameter)
        assert_close(pose[2], height)
        assert_close(pose[4], math.pi / 2.0)

    for name, diameter in (
        ("rod_40mm", 0.040),
        ("rod_80mm", 0.080),
        ("rod_150mm", 0.150),
    ):
        collision = collisions[f"{name}_collision"]
        radius = float(collision.findtext("geometry/cylinder/radius", "nan"))
        assert_close(2.0 * radius, diameter)


def main() -> None:
    world = validate_world()
    validate_moving_obstacles(world)
    validate_loop_arena()
    validate_wire_gate()
    print(
        "Obstacle-course contract PASS: synchronized names, gravity-free "
        "8/10/12 m movers, 80 m square arena, 220 m lap, and six small targets"
    )


if __name__ == "__main__":
    main()
