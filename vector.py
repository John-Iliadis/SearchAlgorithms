from dataclasses import dataclass


@dataclass(unsafe_hash=True, order=True)
class Vector2i:
    x: int
    y: int
