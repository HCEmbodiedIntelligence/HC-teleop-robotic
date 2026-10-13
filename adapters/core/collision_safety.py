from __future__ import annotations

import math
from itertools import combinations
from typing import Mapping, Sequence


Aabb = tuple[Sequence[float], Sequence[float]]


def aabb_distance(first: Aabb, second: Aabb) -> float:
    """Return the Euclidean separation between two axis-aligned boxes."""
    squared = 0.0
    for axis in range(3):
        gap = max(
            float(second[0][axis]) - float(first[1][axis]),
            float(first[0][axis]) - float(second[1][axis]),
            0.0,
        )
        squared += gap * gap
    return math.sqrt(squared)


def descendants(parents: Mapping[int, int], root: int) -> set[int]:
    result: set[int] = set()
    for link in parents:
        current = link
        while current >= 0:
            if current == root:
                result.add(link)
                break
            current = parents.get(current, -1)
    result.discard(root)
    return result


def tree_distance(parents: Mapping[int, int], first: int, second: int) -> int:
    first_ancestors: dict[int, int] = {}
    current = first
    distance = 0
    while current >= 0:
        first_ancestors[current] = distance
        current = parents.get(current, -1)
        distance += 1

    current = second
    distance = 0
    while current >= 0:
        if current in first_ancestors:
            return first_ancestors[current] + distance
        current = parents.get(current, -1)
        distance += 1
    return 1_000_000


def build_protected_pairs(
    collision_links: set[int],
    parents: Mapping[int, int],
    right_arm_root: int,
    left_arm_root: int,
    ignore_graph_distance: int = 2,
    *,
    check_intra_arm: bool = True,
    check_inter_arm: bool = True,
    check_arm_body: bool = True,
) -> tuple[set[int], set[int], list[tuple[int, int]]]:
    """Build arm/body, inter-arm and non-adjacent intra-arm collision pairs."""
    right_links = descendants(parents, right_arm_root) & collision_links
    left_links = descendants(parents, left_arm_root) & collision_links
    moving_links = right_links | left_links
    pairs: list[tuple[int, int]] = []

    for first, second in combinations(sorted(collision_links), 2):
        if first not in moving_links and second not in moving_links:
            continue
        same_arm = (
            first in right_links and second in right_links
        ) or (
            first in left_links and second in left_links
        )
        inter_arm = (
            first in right_links and second in left_links
        ) or (
            first in left_links and second in right_links
        )
        arm_and_body = (first in moving_links) != (second in moving_links)
        if same_arm and not check_intra_arm:
            continue
        if inter_arm and not check_inter_arm:
            continue
        if arm_and_body and not check_arm_body:
            continue
        if (same_arm or arm_and_body) and tree_distance(
            parents, first, second
        ) <= ignore_graph_distance:
            continue
        pairs.append((first, second))

    return right_links, left_links, pairs


def calibrated_clearance(
    requested: float,
    home_distance: float | None,
    tolerance: float,
) -> float:
    """Preserve a valid close-clearance home pose without ignoring the pair."""
    if home_distance is None or home_distance >= requested:
        return requested
    if home_distance <= 0.0:
        return 0.0
    return max(0.0, min(requested, home_distance - tolerance))


def collision_deficit(collision: tuple[str, str, float, float]) -> float:
    """Return how far a collision result lies inside its required clearance."""
    return float(collision[3]) - float(collision[2])


def improves_collision_clearance(
    current: tuple[str, str, float, float] | None,
    candidate: tuple[str, str, float, float],
    epsilon: float,
) -> bool:
    """Allow an unsafe pose only when it monotonically moves toward safety."""
    if current is None:
        return False
    return collision_deficit(candidate) <= collision_deficit(current) - epsilon
