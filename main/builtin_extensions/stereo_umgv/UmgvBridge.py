from __future__ import annotations

from reactivex.subject import BehaviorSubject


class UmgvBridge:
    x_subject: BehaviorSubject[float]
    y_subject: BehaviorSubject[float]

    def __init__(self) -> None:
        self.x_subject = BehaviorSubject(0.0)
        self.y_subject = BehaviorSubject(0.0)
