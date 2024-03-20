from dataclasses import dataclass
from typing import List


@dataclass
class Grid:
    data: List[List[int]] = None
    width: int = 0
    height: int = 0
