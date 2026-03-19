from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class DecisionState:
    risk_state: str = "clear"
    should_stop: bool = False


class PerceptionDecision:
    """
    Blank-slate decision scaffold.
    No red-light policy is applied in this baseline.
    """

    def evaluate(
        self,
        detections: Iterable[object],
        frame_size: Optional[Tuple[int, int]] = None,
        in_junction: bool = False,
        turning: bool = False,
    ) -> DecisionState:
        return DecisionState()

    def get_debug_snapshot(self):
        return {
            "risk_state": "clear",
            "should_stop": False,
        }
