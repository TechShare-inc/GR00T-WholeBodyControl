"""Regression tests for onscreen simulation scheduling and fall recovery."""

from types import SimpleNamespace
import unittest
from unittest import mock

from gear_sonic.utils.mujoco_sim import base_sim


class _Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def time(self):
        return self.now

    def sleep(self, duration):
        self.now += max(0.0, duration)


class _Viewer:
    def __init__(self, env):
        self.env = env
        self.closed = False

    def is_running(self):
        return not self.closed and self.env.steps < 51

    def close(self):
        self.closed = True


class _SlowViewerEnv:
    def __init__(self, clock):
        self.clock = clock
        self.steps = 0
        self.last_render_step = 0
        self.render_gaps = []
        self.image_publish_process = None
        self.viewer = _Viewer(self)

    def sim_step(self):
        self.steps += 1
        # A loaded onscreen simulation can take slightly longer than its 5 ms
        # physics period. The scheduler must still yield to the viewer instead
        # of consuming its entire 250 ms catch-up allowance in one batch.
        self.clock.now += 0.006

    def update_viewer(self):
        self.render_gaps.append(self.steps - self.last_render_step)
        self.last_render_step = self.steps
        self.clock.now += 0.25

    def update_reward(self):
        pass

    def update_render_caches(self):
        pass


class BaseSimulatorSchedulingTest(unittest.TestCase):
    def test_slow_viewer_does_not_starve_state_synchronization(self):
        clock = _Clock()
        env = _SlowViewerEnv(clock)
        simulator = base_sim.BaseSimulator.__new__(base_sim.BaseSimulator)
        simulator.sim_dt = 0.005
        simulator.viewer_dt = 0.02
        simulator.reward_dt = 0.02
        simulator.image_dt = 1.0
        simulator.redis_client = None
        simulator.sim_env = env
        simulator._running = True

        with (
            mock.patch.object(base_sim.time, "monotonic", clock.monotonic),
            mock.patch.object(base_sim.time, "time", clock.time),
            mock.patch.object(base_sim.time, "sleep", clock.sleep),
        ):
            simulator.start()

        viewer_stride = round(simulator.viewer_dt / simulator.sim_dt)
        self.assertLessEqual(max(env.render_gaps), viewer_stride)

    def test_fall_reset_restores_configured_support_and_monotonic_time(self):
        env = base_sim.DefaultEnv.__new__(base_sim.DefaultEnv)
        env.mj_model = object()
        env.mj_data = SimpleNamespace(time=12.5)
        env.elastic_band = SimpleNamespace(enable=False)
        env.config = {"ENABLE_ELASTIC_BAND": True}

        def reset_data(_model, data):
            data.time = 0.0

        with (
            mock.patch.object(base_sim.mujoco, "mj_resetData", reset_data),
            mock.patch.object(base_sim.mujoco, "mj_forward"),
        ):
            env.reset()

        self.assertEqual(env.mj_data.time, 12.5)
        self.assertTrue(env.elastic_band.enable)


if __name__ == "__main__":
    unittest.main()
