"""Pure, diagnostic-only frame-error witness construction."""

from __future__ import annotations

import math
from typing import Any


def _finite_vector(value: Any) -> bool:
    try:
        return len(value) == 3 and all(math.isfinite(float(item)) for item in value)
    except (TypeError, ValueError):
        return False


def _position(sample: Any) -> tuple[float, float, float] | None:
    if isinstance(sample, dict) and _finite_vector(sample.get("position")):
        return tuple(float(item) for item in sample["position"])
    if isinstance(sample, dict):
        value = [sample.get(axis) for axis in ("x", "y", "z")]
        if _finite_vector(value):
            return tuple(float(item) for item in value)
    return None


def _source_stamp(sample: Any) -> int | None:
    if not isinstance(sample, dict):
        return None
    value = sample.get("source_stamp_ns")
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return int(number) if math.isfinite(number) and number.is_integer() and number > 0 else None


def _interpolate_position(
    history: list[dict[str, Any]], target_stamp_ns: int, *, epoch: int | None = None
) -> tuple[float, float, float] | None:
    samples: list[tuple[int, tuple[float, float, float]]] = []
    for sample in history:
        if epoch is not None and sample.get("localization_epoch") != epoch:
            continue
        stamp = _source_stamp(sample)
        position = _position(sample)
        if stamp is not None and position is not None:
            samples.append((stamp, position))
    samples.sort(key=lambda item: item[0])
    for stamp, position in samples:
        if stamp == target_stamp_ns:
            return position
    for (left_stamp, left), (right_stamp, right) in zip(samples, samples[1:]):
        if left_stamp > target_stamp_ns or target_stamp_ns > right_stamp:
            continue
        if right_stamp <= left_stamp:
            return None
        alpha = (target_stamp_ns - left_stamp) / (right_stamp - left_stamp)
        return tuple(left[index] + alpha * (right[index] - left[index]) for index in range(3))
    return None


def _rotation_matrix(value: Any) -> list[list[float]] | None:
    try:
        matrix = [[float(item) for item in row] for row in value]
    except (TypeError, ValueError):
        return None
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        return None
    if not all(math.isfinite(item) for row in matrix for item in row):
        return None
    for column in range(3):
        norm = sum(matrix[row][column] ** 2 for row in range(3))
        if abs(norm - 1.0) > 1.0e-5:
            return None
    for left in range(3):
        for right in range(left):
            dot = sum(matrix[row][left] * matrix[row][right] for row in range(3))
            if abs(dot) > 1.0e-5:
                return None
    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    return matrix if abs(determinant - 1.0) <= 1.0e-5 else None


def build_frame_error_witness(
    *,
    state_source_stamp_ns: int,
    localization_epoch: int,
    truth_frame_witness: dict[str, Any] | None,
    lio_history: list[dict[str, Any]],
    truth_history: list[dict[str, Any]],
) -> dict[str, Any] | None:
    """Build one source-time frame-error witness, or fail closed.

    This function is intentionally independent of ROS and has no navigation
    authority. It requires a transform-backed bracket in both streams and
    never extrapolates or substitutes a zero error.
    """
    if not isinstance(state_source_stamp_ns, int) or state_source_stamp_ns <= 0:
        return None
    if not isinstance(localization_epoch, int) or localization_epoch <= 0:
        return None
    if not isinstance(truth_frame_witness, dict) or truth_frame_witness.get("valid") is not True:
        return None
    witness_epoch = truth_frame_witness.get("lio_localization_epoch")
    if witness_epoch != localization_epoch:
        return None
    scope = truth_frame_witness.get("validity_scope")
    if scope not in {"localization_epoch", "source_interval"}:
        return None
    if not truth_frame_witness.get("provenance") and not truth_frame_witness.get("reference_event"):
        return None
    transform = truth_frame_witness.get("T_L_G")
    if not isinstance(transform, dict):
        return None
    translation = transform.get("translation_lio_from_gazebo", transform.get("translation"))
    if not _finite_vector(translation):
        return None
    matrix = _rotation_matrix(
        transform.get("rotation_matrix_lio_from_gazebo", transform.get("rotation_matrix"))
    )
    if matrix is None:
        return None
    if scope == "source_interval":
        try:
            valid_from = int(truth_frame_witness["valid_from_source_stamp_ns"])
            valid_until = int(truth_frame_witness["valid_until_source_stamp_ns"])
        except (KeyError, TypeError, ValueError):
            return None
        if valid_from <= 0 or valid_until < valid_from:
            return None
        if not valid_from <= state_source_stamp_ns <= valid_until:
            return None
    lio_position = _interpolate_position(
        lio_history, state_source_stamp_ns, epoch=localization_epoch
    )
    truth_position = _interpolate_position(truth_history, state_source_stamp_ns)
    if lio_position is None or truth_position is None:
        return None
    transformed_truth = tuple(
        sum(matrix[row][column] * truth_position[column] for column in range(3))
        + float(translation[row])
        for row in range(3)
    )
    frame_error = [
        transformed_truth[index] - lio_position[index] for index in range(3)
    ]
    if not _finite_vector(frame_error):
        return None
    return {
        "frame_error_vector_m": frame_error,
        "frame_error_source_stamp_ns": state_source_stamp_ns,
        "frame_error_localization_epoch": localization_epoch,
        "frame_error_basis": "truth_in_lio_frame_minus_lio",
        "frame_error_transform_provenance": truth_frame_witness.get(
            "provenance", truth_frame_witness.get("reference_event")
        ),
        "frame_error_validity_scope": scope,
    }
