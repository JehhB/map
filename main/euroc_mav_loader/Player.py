from __future__ import annotations

import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError
from typing import Generic, Iterator, Literal, Optional, TypeVar

from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import BehaviorSubject, Subject
from typing_extensions import TypeAlias

from app.EventEmiter import EventEmitter

T = TypeVar("T")

PlayerState: TypeAlias = Literal["playing", "idle", "paused", "completed"]


class Player(EventEmitter, Generic[T]):
    data_iterator: Iterator[T]
    _frame_rate: float
    _frame_interval: float
    subject: Subject[T]
    state_subject: Subject[PlayerState]
    executor: ThreadPoolExecutor
    scheduler: ThreadPoolScheduler

    _running: bool
    _paused: bool
    _stop_requested: bool
    _playback_thread: Optional[Future[None]]
    _lock: threading.Lock
    _frame_rate_changed: threading.Event

    current_frame: Optional[T]
    frame_count: int

    """
    A background task for playing through an iterator with control operations
    (start, stop, pause, step).

    Emits items from the iterator to an RxPy Subject that observers can subscribe to.
    Supports dynamic frame rate adjustment during playback.
    """

    def __init__(
        self,
        data_iterator: Iterator[T],
        subject: Optional[Subject[T]] = None,
        state_subject: Optional[Subject[PlayerState]] = None,
        frame_rate: float = 20.0,
    ):
        """
        Initialize the player with a data iterator and subjects.

        Args:
            data_iterator: Iterator that yields data (e.g., EuRoCDataset iterator)
            subject: RxPy Subject for emitting frames (created if None)
            frame_rate: Playback rate in frames per second (default: 10 fps)
        """
        self.data_iterator = data_iterator
        self._frame_rate = frame_rate
        self._frame_interval = 1.0 / frame_rate

        # Use provided subjects or create new ones
        self.subject = subject if subject is not None else Subject()
        self.state_subject = (
            state_subject if state_subject is not None else BehaviorSubject("idle")
        )

        # Thread pool for background processing
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.scheduler = ThreadPoolScheduler(max_workers=1)

        # Control flags
        self._running = False
        self._paused = False
        self._stop_requested = False
        self._playback_thread = None
        self._lock = threading.Lock()
        self._frame_rate_changed = threading.Event()

        # Current frame info
        self.current_frame = None
        self.frame_count = 0

        _ = self.step()

    @property
    def frame_rate(self):
        """Get the current frame rate"""
        return self._frame_rate

    @frame_rate.setter
    def frame_rate(self, value):
        """Set the frame rate and update the frame interval"""
        if value <= 0:
            raise ValueError("Frame rate must be positive")

        with self._lock:
            old_rate = self._frame_rate
            self._frame_rate = value
            self._frame_interval = 1.0 / value

            # Notify that frame rate has changed
            self._frame_rate_changed.set()

    def start(self):
        """Start playback in the background"""
        with self._lock:
            if self._running:
                return False

            self._running = True
            self._paused = False
            self._stop_requested = False

            # Start the playback thread
            self._playback_thread = self.executor.submit(self._playback_task)

            self.state_subject.on_next("playing")

            return True

    def pause(self):
        """Pause playback"""
        with self._lock:
            if not self._running or self._stop_requested:
                return False

            was_paused = self._paused
            self._paused = not was_paused

            self.state_subject.on_next("paused")

            return True

    def resume(self):
        """Resume playback after pause"""
        with self._lock:
            if not self._running or not self._paused or self._stop_requested:
                return False

            self._paused = False

            self.state_subject.on_next("playing")

            return True

    def stop(self):
        """Stop playback"""
        with self._lock:
            if not self._running:
                return False

            self._stop_requested = True
            self._running = False
            self._paused = False

            # Wait for the thread to finish
            if self._playback_thread is not None:
                try:
                    self._playback_thread.result(timeout=1.0)
                except TimeoutError:
                    # Thread didn't complete in time
                    pass

            self.state_subject.on_next("idle")
            return True

    def step(self):
        """
        Step forward one frame. Works when paused or even when
        playback hasn't been started.
        """
        _ = self.pause()

        try:

            frame = next(self.data_iterator)
            self.current_frame = frame
            self.frame_count += 1
            self.subject.on_next(frame)

            return True
        except StopIteration:
            self.subject.on_completed()

            self.state_subject.on_next("idle")

            return False

    def reset(self, data_iterator: Optional[Iterator[T]] = None):
        """
        Reset the player with a new or reset iterator.
        Stops the current playback if running.

        Args:
            data_iterator: New iterator to use (optional)
        """
        _ = self.stop()

        with self._lock:
            if data_iterator is not None:
                self.data_iterator = data_iterator

            self.frame_count = 0
            self.current_frame = None

    def is_running(self) -> bool:
        """Check if playback is currently running"""
        with self._lock:
            return self._running and not self._stop_requested

    def is_paused(self) -> bool:
        """Check if playback is currently paused"""
        with self._lock:
            return self._running and self._paused

    def set_frame_rate(self, frame_rate: float):
        """
        Change the playback frame rate.
        This is an alias for the frame_rate property setter.

        Args:
            frame_rate: New frame rate in frames per second
        """
        self.frame_rate = frame_rate

    def get_status(self):
        """Get the current status of the player"""
        with self._lock:
            return {
                "running": self._running and not self._stop_requested,
                "paused": self._paused,
                "frame_rate": self._frame_rate,
                "frame_count": self.frame_count,
            }

    def _playback_task(self):
        """Background task that plays through the iterator"""
        try:
            # Reset frame rate change event
            self._frame_rate_changed.clear()

            # Main playback loop
            while not self._stop_requested:
                # If paused, just sleep briefly to avoid busy-waiting
                if self._paused:
                    # Check for frame rate changes while paused
                    self._frame_rate_changed.wait(0.1)
                    self._frame_rate_changed.clear()
                    continue

                # Get the start time for this frame
                start_time = time.time()

                # Try to get the next frame
                try:
                    frame = next(self.data_iterator)
                    self.current_frame = frame
                    self.frame_count += 1

                    # Emit the frame to the subject
                    self.subject.on_next(frame)

                    # Calculate sleep time to maintain frame rate
                    elapsed = time.time() - start_time
                    sleep_time = max(0, self._frame_interval - elapsed)

                    # Use wait with event to allow frame rate changes to interrupt sleep
                    if sleep_time > 0:
                        # Wait until either the sleep time has elapsed or frame_rate_changed is set
                        interrupted = self._frame_rate_changed.wait(sleep_time)
                        if interrupted:
                            # Clear the event for next iteration
                            self._frame_rate_changed.clear()

                except StopIteration:
                    # End of iterator reached
                    self.subject.on_completed()

                    self.state_subject.on_next("idle")

                    break

        except Exception as e:
            self.subject.on_error(e)

        finally:
            with self._lock:
                self._running = False
                self._paused = False
                self._stop_requested = False

    def dispose(self):
        """Gracefully stop the player and release resources."""
        _ = self.stop()
        self.executor.shutdown(wait=True)
        self.subject.on_completed()
        self.state_subject.on_completed()
