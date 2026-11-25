"""Single-slot frame scheduling for the deterministic and process-backed watchdog."""

import math
import multiprocessing as mp
import time
from collections import Counter
from copy import deepcopy
from multiprocessing.connection import wait
from numbers import Real

import numpy as np

from pbs.types import RiskEstimate


def _bind(assessor, scenario):
    try:
        assessor.bind(scenario)
        return True
    except Exception:
        return False


def _assess(assessor, ctx, history):
    try:
        est = assessor.assess(ctx, history)
        if (
            not isinstance(est, RiskEstimate)
            or not isinstance(est.considered, bool | np.bool_)
            or not isinstance(est.ttc, Real)
            or isinstance(est.ttc, bool)
            or not est.ttc >= 0
        ):
            return None, "invalid_output"
        return (float(est.ttc) if est.considered else math.inf), None
    except Exception:
        return None, "exception"


class VirtualChannels:
    """Scalar timings are whole-frame costs; sequences are successive agent costs."""

    def __init__(self, assessors, scenario):
        self.assessors = assessors
        self.bound = {name: _bind(a, scenario) for name, a in assessors.items()}
        self.busy_until = {}

    def collect(self, frame, ctx, histories, start, deadline, timings):
        results, reasons, discarded = {}, Counter(), 0
        elapsed = 0.0
        for name, assessor in self.assessors.items():
            pending = self.busy_until.get(name)
            if pending is not None and pending <= start:
                del self.busy_until[name]
                discarded += 1
            if not histories:
                continue
            if not self.bound[name]:
                reasons[name, "bind_error"] += 1
                continue
            if name in self.busy_until:
                reasons[name, "busy"] += 1
                continue
            costs = timings.get(name + "_s", 0.0)
            if np.isscalar(costs):
                offsets = [float(costs)] * len(histories)
            else:
                if len(costs) != len(histories):
                    raise ValueError("Provide one channel duration per fresh target.")
                if any(not cost >= 0 for cost in costs):
                    raise ValueError("Expected nonnegative channel durations.")
                offsets = np.cumsum(costs)
            if any(not offset >= 0 for offset in offsets):
                raise ValueError("Expected nonnegative channel durations.")
            elapsed = max(elapsed, min(offsets[-1], deadline - start))
            for (agent, history), offset in zip(histories.items(), offsets, strict=True):
                if start + offset > deadline:
                    self.busy_until[name] = start + offsets[-1]
                    reasons[name, "timeout"] += 1
                    break
                value, error = _assess(assessor, ctx, history)
                if error:
                    reasons[name, error] += 1
                else:
                    results[name, agent] = value
        return results, reasons, discarded, elapsed

    def close(self):
        pass


def _worker(assessor, scenario, requests, replies):
    try:
        bound = _bind(assessor, scenario)
        replies.send(bound)
        if not bound:
            return
        while True:
            frame, ctx, histories = requests.get()
            for agent, history in histories.items():
                value, reason = _assess(assessor, ctx, history)
                replies.send((frame, time.monotonic(), agent, value, reason))
            replies.send((frame, time.monotonic(), None, None, None))
    finally:
        replies.close()


class _ProcessChannel:
    """One persistent worker. The parent only submits when the slot is free."""

    def __init__(self, context, assessor, scenario):
        self.requests = context.Queue(maxsize=1)
        self.replies, sender = context.Pipe(duplex=False)
        self.process = context.Process(
            target=_worker, args=(assessor, scenario, self.requests, sender), daemon=True
        )
        self.pending = None
        self.bound = None  # Still starting; False means bind failed or the worker exited.
        try:
            self.process.start()
        except BaseException:
            self.requests.close()
            self.replies.close()
            self.process.close()
            raise
        finally:
            sender.close()

    def check_startup(self, timeout=0.0):
        if self.bound is None and self.replies.poll(timeout):
            try:
                self.bound = self.replies.recv()
            except (EOFError, OSError):
                self.bound = False

    def close(self):
        if self.process.is_alive():
            self.process.kill()
        self.process.join()
        self.replies.close()
        self.requests.cancel_join_thread()
        self.requests.close()
        self.process.close()


class AsyncChannels:
    """Workers send per-agent results as they finish. Only the parent decides."""

    def __init__(self, assessors, scenario):
        self.workers = {}
        try:
            context = mp.get_context("spawn")
            for name, assessor in assessors.items():
                self.workers[name] = _ProcessChannel(context, assessor, scenario)
            # Process startup and scenario binding belong to setup, outside the frame budget.
            for worker in self.workers.values():
                worker.check_startup(timeout=5)
        except BaseException:
            self.close()
            raise

    def collect(self, frame, ctx, histories, start, deadline, timings):
        if timings:
            raise ValueError("Channel timing injection requires the deterministic backend.")
        results, reasons, discarded = {}, Counter(), 0
        waiting = {}
        # The queue feeder serializes later; detach input from the caller for busy workers.
        snapshot = deepcopy((frame, ctx, histories)) if histories else None

        def receive(name, worker):
            nonlocal discarded
            try:
                tag, finished, agent, value, reason = worker.replies.recv()
            except (EOFError, OSError):
                worker.bound = False
                worker.pending = None
                reasons[name, "worker_exit"] += 1
                return
            if agent is None:
                worker.pending = None
                discarded += int(tag != frame)
            elif tag == frame and finished <= deadline:
                if reason:
                    reasons[name, reason] += 1
                else:
                    results[name, agent] = value

        for name, worker in self.workers.items():
            worker.check_startup()
            while worker.bound and worker.pending is not None and worker.replies.poll():
                receive(name, worker)
                if time.monotonic() >= deadline:
                    break
            if not histories:
                continue
            if not worker.bound:
                reasons[name, "unavailable"] += 1
            elif worker.pending is not None:
                reasons[name, "busy"] += 1
            elif time.monotonic() < deadline:
                worker.requests.put_nowait(snapshot)
                worker.pending = frame
                waiting[worker.replies] = (name, worker)
            else:
                reasons[name, "timeout"] += 1

        while waiting:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            for pipe in wait(waiting, timeout=remaining):
                name, worker = waiting[pipe]
                receive(name, worker)
                if worker.pending is None:
                    del waiting[pipe]
        for name, _ in waiting.values():
            reasons[name, "timeout"] += 1
        return results, reasons, discarded, time.monotonic() - start

    def close(self):
        for worker in self.workers.values():
            worker.close()
        self.workers.clear()
