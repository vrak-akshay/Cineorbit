"""
Reproducible tests for landmark_elimination.py (fixed seeds, synthetic scenes, no data files).

    python test_landmark_elimination.py -v        # run all tests
    python test_landmark_elimination.py --show    # watch the moving-object scenario live
"""
import sys
import unittest
from unittest import mock

import cv2
import numpy as np

import landmark_elimination as le
from landmark_elimination import (EliminationConfig, Landmark, LandmarkState,
                                  LandmarkTracker, estimate_camera_motion)

W, H = 640, 480


# --------------------------------------------------------------------------- #
# Synthetic scene helpers
# --------------------------------------------------------------------------- #
def make_texture(h, w, seed, n_shapes=350):
    rng = np.random.RandomState(seed)
    img = np.full((h, w), 128, np.uint8)
    for _ in range(n_shapes):
        x, y = int(rng.randint(0, w)), int(rng.randint(0, h))
        sw, sh = (int(v) for v in rng.randint(8, 40, size=2))
        c = int(rng.randint(0, 256))
        if rng.rand() < 0.7:
            cv2.rectangle(img, (x, y), (x + sw, y + sh), c, -1)
        else:
            cv2.circle(img, (x, y), sw // 2, c, -1)
    img = np.clip(img + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(img, (3, 3), 0.8)


BASE = make_texture(H, W, seed=1)


def camera_view(scene, k, deg_per_frame=0.0, shift_per_frame=(0.0, 0.0)):
    """Frame k of a camera that rotates about the image centre and/or translates."""
    M = cv2.getRotationMatrix2D((W / 2, H / 2), k * deg_per_frame, 1.0)
    M[:, 2] += np.array(shift_per_frame) * k
    return cv2.warpAffine(scene, M, (W, H), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REFLECT101)


def run(frames, tracker):
    tracker.initialize(frames[0])
    return [tracker.update(f) for f in frames[1:]]


def new_tracker(**cfg_kw):
    return LandmarkTracker(EliminationConfig(**cfg_kw), verbose=False)


class InjectingTracker(LandmarkTracker):
    """Corrupts selected LK outputs to emulate tracking errors."""
    def __init__(self, *a, injector=None, **k):
        super().__init__(*a, **k)
        self.injector = injector

    def _track(self, prev_gray, gray, pts):
        nxt, ok = super()._track(prev_gray, gray, pts)
        if self.injector is not None:
            nxt, ok = self.injector(self, nxt.copy(), ok.copy())
        return nxt, ok


def row_of(tracker, lid):
    return tracker._query_ids.index(lid) if lid in tracker._query_ids else None


# --------------------------------------------------------------------------- #
# A. Elimination-rule unit tests (no images)
# --------------------------------------------------------------------------- #
class TestTwoConditionRule(unittest.TestCase):
    T_OK, T_BAD = 1.0, 5.0

    def feed(self, lm, pattern):
        """pattern: string of 'x' (violation) / '.' (clean). Returns frame index of elimination."""
        for i, ch in enumerate(pattern, start=1):
            ev = lm.add_observation(self.T_BAD if ch == "x" else self.T_OK, i)
            if ev:
                return i
        return None

    def lm(self):
        return Landmark(0, (10, 10), EliminationConfig())

    def test_single_violation_does_not_eliminate(self):
        lm = self.lm()
        self.assertIsNone(self.feed(lm, "x" + "." * 30))
        self.assertIs(lm.state, LandmarkState.ACTIVE)

    def test_three_violations_in_window_do_not_eliminate(self):
        lm = self.lm()
        self.assertIsNone(self.feed(lm, "xxx" + "." * 7))     # window full, only 3
        self.assertEqual(lm.violations_in_window, 3)
        self.assertIs(lm.state, LandmarkState.ACTIVE)

    def test_not_evaluated_before_window_is_full(self):
        lm = self.lm()
        self.assertIsNone(self.feed(lm, "x" * 9))             # 9 obs, all violations: no evaluation yet
        self.assertIs(lm.state, LandmarkState.ACTIVE)
        ev = lm.add_observation(self.T_OK, 10)                # 10th valid obs fills the window
        self.assertIsNotNone(ev)                              # 9 violations >= 4 -> eliminated now
        self.assertEqual(ev.n_valid_obs, 10)

    def test_fourth_violation_in_full_window_eliminates(self):
        lm = self.lm()
        # violations at obs 5, 8, 11 (+ the 4th at 14): window at 14 = obs 5..14 -> 4 violations
        pat = "....x..x..x..x"
        self.assertEqual(self.feed(lm, pat), 14)
        self.assertEqual(lm.elimination.violations, 4)
        self.assertEqual(lm.elimination.n_valid_obs, 14)

    def test_old_violations_leave_the_window(self):
        lm = self.lm()
        self.assertIsNone(self.feed(lm, "xxx" + "." * 10))    # obs 1..13: violations aged out
        self.assertEqual(lm.violations_in_window, 0)
        lm.add_observation(self.T_BAD, 14)
        self.assertEqual(lm.violations_in_window, 1)
        self.assertEqual(len(lm.history), 10)

    def test_fourth_violation_outside_window_does_not_eliminate(self):
        lm = self.lm()
        # violations at obs 2,3 and 11: window at obs 11 = 2..11 -> 3 violations only
        self.assertIsNone(self.feed(lm, ".xx" + "." * 7 + "x"))
        self.assertEqual(lm.violations_in_window, 3)

    def test_threshold_is_strictly_greater_than(self):
        lm = self.lm()
        lm.add_observation(3.0, 1)
        self.assertEqual(lm.violations_in_window, 0)

    def test_invalid_observations_never_enter_history(self):
        lm = self.lm()
        lm.add_observation(self.T_OK, 1)
        lm.add_observation(float("nan"), 2)
        lm.mark_no_observation()
        self.assertEqual(len(lm.history), 1)
        self.assertEqual(lm.n_valid_obs, 1)

    def test_eliminated_is_terminal(self):
        lm = self.lm()
        self.feed(lm, "x" * 10)
        self.assertIs(lm.state, LandmarkState.ELIMINATED)
        self.assertIsNone(lm.add_observation(self.T_OK, 99))
        self.assertIs(lm.state, LandmarkState.ELIMINATED)

    def test_configurable(self):
        lm = Landmark(0, (0, 0), EliminationConfig(residual_threshold_px=1.0, window_size=5,
                                                   min_violations=2))
        self.assertEqual(self.feed(lm, "..x.x"), 5)

    def test_config_validation(self):
        with self.assertRaises(ValueError):
            EliminationConfig(window_size=3, min_violations=4)


# --------------------------------------------------------------------------- #
# B. Scenarios on synthetic video
# --------------------------------------------------------------------------- #
class TestScenarios(unittest.TestCase):
    # 1 ------------------------------------------------------------------
    def test_1_stationary_camera_stationary_scene(self):
        tr = new_tracker()
        results = run([BASE.copy() for _ in range(25)], tr)
        self.assertGreater(len(tr.landmarks), 50)
        self.assertTrue(all(r.motion.valid for r in results))
        self.assertEqual(results[-1].n_eliminated, 0)
        self.assertLess(max(max(r.residuals.values()) for r in results), 0.5)

    # 2 ------------------------------------------------------------------
    def test_2_camera_rotation_stationary_scene(self):
        tr = new_tracker()
        frames = [camera_view(BASE, k, deg_per_frame=1.0) for k in range(30)]
        results = run(frames, tr)
        self.assertGreaterEqual(sum(r.motion.valid for r in results), 28)
        self.assertEqual(results[-1].n_eliminated, 0)
        resid = np.concatenate([list(r.residuals.values()) for r in results if r.residuals])
        self.assertLess(np.median(resid), 1.0)

    def test_2b_compensation_differs_from_raw_displacement(self):
        """Camera pans 5 px/frame: raw displacement >> T, residual ~ 0, nobody eliminated."""
        tr = new_tracker()
        frames = [camera_view(BASE, k, shift_per_frame=(5.0, 2.0)) for k in range(25)]
        tr.initialize(frames[0])
        raw, res = [], []
        for f in frames[1:]:
            before = {lm.id: lm.pos.copy() for lm in tr.active_landmarks()}
            r = tr.update(f)
            for lm in tr.active_landmarks():
                if lm.id in before and lm.id in r.residuals:
                    raw.append(np.linalg.norm(lm.pos - before[lm.id]))
                    res.append(r.residuals[lm.id])
        self.assertGreater(np.median(raw), 3.0 * 1.5)    # raw motion would "violate" everywhere
        self.assertLess(np.median(res), 1.0)
        self.assertEqual(tr.landmarks_in_state(LandmarkState.ELIMINATED), [])

    # 3 ------------------------------------------------------------------
    def make_moving_object_scene(self, n=40):
        obj = make_texture(90, 90, seed=2, n_shapes=40)
        ox, oy, vx = 100, 200, 4
        frames = []
        for k in range(n):
            scene = BASE.copy()
            x = ox + vx * k
            scene[oy:oy + 90, x:x + 90] = obj
            frames.append(camera_view(scene, k, deg_per_frame=0.3))   # camera also rotating
        obj_rect = (ox, oy, ox + 90, oy + 90)
        swept = (ox, oy, ox + vx * (n - 1) + 90, oy + 90)
        return frames, obj_rect, swept

    def test_3_independently_moving_object(self):
        frames, (x0, y0, x1, y1), (sx0, sy0, sx1, sy1) = self.make_moving_object_scene()
        tr = new_tracker()
        tr.initialize(frames[0])
        init = {i: lm.pos.copy() for i, lm in tr.landmarks.items()}
        for f in frames[1:]:
            tr.update(f)
        m = 6
        on_obj = [i for i, p in init.items() if x0 + m < p[0] < x1 - m and y0 + m < p[1] < y1 - m]
        far_bg = [i for i, p in init.items()
                  if not (sx0 - 25 < p[0] < sx1 + 25 and sy0 - 25 < p[1] < sy1 + 25)]
        self.assertGreaterEqual(len(on_obj), 5)
        elim = lambda ids: sum(tr.landmarks[i].state is LandmarkState.ELIMINATED for i in ids)
        print(f"\n    [scenario 3] object landmarks eliminated: {elim(on_obj)}/{len(on_obj)}; "
              f"background landmarks (outside swept path) eliminated: {elim(far_bg)}/{len(far_bg)}")
        self.assertGreaterEqual(elim(on_obj), 0.7 * len(on_obj))
        self.assertEqual(elim(far_bg), 0)

    # 4 ------------------------------------------------------------------
    def test_4_temporary_tracking_errors(self):
        target = {}

        def injector(tr, nxt, ok):
            if "id" not in target:
                target["id"] = tr._query_ids[0]
            row = row_of(tr, target["id"])
            if row is None:
                return nxt, ok
            if tr.frame_idx in (12, 13):             # LK failure (2 frames)
                ok[row] = False
            if tr.frame_idx in (20, 21):             # wrong LK result (2 frames, 6 px off)
                nxt[row, 0] += 6.0
            return nxt, ok

        tr = InjectingTracker(EliminationConfig(), verbose=False, injector=injector)
        results = run([BASE.copy() for _ in range(30)], tr)
        lm = tr.landmarks[target["id"]]
        self.assertIsNot(lm.state, LandmarkState.ELIMINATED)
        self.assertIs(lm.state, LandmarkState.ACTIVE)             # recovered
        # frames 1-11 valid (11), 12-13 failed, 14 recovery (no obs), 15-29 valid (15)
        self.assertEqual(lm.n_valid_obs, 11 + 15)
        self.assertEqual(lm.total_violations, 2)                  # only the 2 injected errors
        lost_seen = any(r.n_lost >= 1 for r in results[11:13])
        self.assertTrue(lost_seen)                                # LOST, not ELIMINATED

    def test_4b_failed_lk_is_not_a_violation(self):
        """LK fails for a landmark for many frames: it is LOST, history stays empty."""
        target = {}

        def injector(tr, nxt, ok):
            target.setdefault("id", tr._query_ids[0])
            row = row_of(tr, target["id"])
            if row is not None and tr.frame_idx >= 3:
                ok[row] = False
            return nxt, ok

        tr = InjectingTracker(EliminationConfig(max_lost_frames=3), verbose=False, injector=injector)
        run([BASE.copy() for _ in range(25)], tr)
        lm = tr.landmarks[target["id"]]
        self.assertIs(lm.state, LandmarkState.LOST)
        self.assertTrue(lm.lost_permanently)
        self.assertEqual(lm.total_violations, 0)
        self.assertEqual(lm.n_valid_obs, 2)                       # frames 1-2 only

    # 5 ------------------------------------------------------------------
    def test_5_repeated_threshold_violations(self):
        target = {}

        def injector(tr, nxt, ok):
            target.setdefault("id", tr._query_ids[0])
            row = row_of(tr, target["id"])
            if row is not None and tr.frame_idx >= 8:
                nxt[row, 0] += 6.0 * (-1) ** tr.frame_idx    # alternating => bounded drift
            return nxt, ok

        tr = InjectingTracker(EliminationConfig(), verbose=False, injector=injector)
        tr.initialize(BASE.copy())
        elim_frame = None
        for _ in range(20):
            r = tr.update(BASE.copy())
            lm = tr.landmarks[target["id"]]
            if elim_frame is None and lm.state is LandmarkState.ELIMINATED:
                elim_frame = r.frame_idx
                self.assertEqual([e.landmark_id for e in r.events], [target["id"]])
            elif elim_frame is None:
                # frames 8,9,10 are violations 1..3 -> still ACTIVE through frame 10
                self.assertLessEqual(lm.violations_in_window, 3)
        # violations on frames 8,9,10 (3, window full at 10) -> 4th on frame 11 eliminates
        self.assertEqual(elim_frame, 11)
        self.assertEqual(tr.landmarks[target["id"]].elimination.violations, 4)
        others = [l for l in tr.landmarks.values() if l.id != target["id"]]
        self.assertTrue(all(l.state is not LandmarkState.ELIMINATED for l in others))

    # 6 ------------------------------------------------------------------
    def test_6_insufficient_correspondences(self):
        tr = new_tracker(min_correspondences=10_000)
        results = run([camera_view(BASE, k, shift_per_frame=(2, 0)) for k in range(25)], tr)
        self.assertTrue(all(not r.motion.valid for r in results))
        self.assertTrue(all(r.motion.reason == "insufficient_correspondences" for r in results))
        self.assertTrue(all(l.n_valid_obs == 0 for l in tr.landmarks.values()))
        self.assertEqual(results[-1].n_eliminated, 0)

    def test_6b_blank_frames_no_features(self):
        tr = new_tracker()
        blank = np.full((H, W), 90, np.uint8)
        results = run([blank] * 5, tr)
        self.assertEqual(len(tr.landmarks), 0)
        self.assertTrue(all(not r.motion.valid for r in results))

    def test_6c_many_lk_failures_in_one_frame(self):
        """Only 3 landmarks survive LK on frame 15 -> no model, no residuals, no violations."""
        def injector(tr, nxt, ok):
            if tr.frame_idx == 15:
                ok[3:] = False
            return nxt, ok

        tr = InjectingTracker(EliminationConfig(), verbose=False, injector=injector)
        results = run([BASE.copy() for _ in range(18)], tr)
        r15 = results[14]
        self.assertFalse(r15.motion.valid)
        self.assertEqual(r15.motion.reason, "insufficient_correspondences")
        self.assertEqual(r15.residuals, {})
        self.assertEqual(results[-1].n_eliminated, 0)

    # 7 ------------------------------------------------------------------
    def test_7_ransac_failure(self):
        tr = new_tracker()
        frames = [camera_view(BASE, k, shift_per_frame=(3, 1)) for k in range(25)]
        with mock.patch.object(cv2, "estimateAffinePartial2D", return_value=(None, None)):
            results = run(frames, tr)
        self.assertTrue(all(r.motion.reason == "ransac_failed" for r in results))
        self.assertTrue(all(r.residuals == {} for r in results))
        self.assertTrue(all(l.n_valid_obs == 0 for l in tr.landmarks.values()))
        self.assertEqual(results[-1].n_eliminated, 0)

    def test_7b_weak_consensus_rejected(self):
        cfg = EliminationConfig()
        n = 40
        pts = np.random.RandomState(0).rand(n, 2).astype(np.float32) * 300
        inl = np.zeros((n, 1), np.uint8)
        inl[:5] = 1
        with mock.patch.object(cv2, "estimateAffinePartial2D",
                               return_value=(np.array([[1, 0, 0], [0, 1, 0]], float), inl)):
            est = estimate_camera_motion(pts, pts, np.zeros(n, bool), cfg)
        self.assertFalse(est.valid)
        self.assertEqual(est.reason, "low_consensus")

    def test_7c_implausible_scale_rejected(self):
        cfg = EliminationConfig()
        n = 20
        pts = np.random.RandomState(0).rand(n, 2).astype(np.float32) * 300
        with mock.patch.object(cv2, "estimateAffinePartial2D",
                               return_value=(np.array([[3, 0, 0], [0, 3, 0]], float),
                                             np.ones((n, 1), np.uint8))):
            est = estimate_camera_motion(pts, pts, np.zeros(n, bool), cfg)
        self.assertFalse(est.valid)
        self.assertEqual(est.reason, "implausible_transform")

    def test_suspects_excluded_from_reference_set(self):
        cfg = EliminationConfig()
        rng = np.random.RandomState(0)
        prev = rng.rand(30, 2).astype(np.float32) * 300
        curr = prev + np.array([2.0, 1.0], np.float32)           # true global shift
        suspect = np.zeros(30, bool)
        suspect[:10] = True
        curr[:10] += rng.rand(10, 2).astype(np.float32) * 40     # suspects move on their own
        est = estimate_camera_motion(prev, curr, suspect, cfg)
        self.assertTrue(est.valid)
        self.assertEqual(est.n_reference, 20)                    # suspects not offered to RANSAC
        self.assertFalse(est.used_suspects_fallback)
        np.testing.assert_allclose(est.M[:, 2], [2.0, 1.0], atol=1e-3)


# --------------------------------------------------------------------------- #
# Live demo on the synthetic moving-object scene
# --------------------------------------------------------------------------- #
def demo_show():
    frames, _, _ = TestScenarios().make_moving_object_scene(n=60)
    tr = LandmarkTracker(EliminationConfig())
    tr.initialize(frames[0])
    for f in frames[1:]:
        r = tr.update(f)
        cv2.imshow("synthetic moving-object scene (q to quit)",
                   le.draw_overlay(cv2.cvtColor(f, cv2.COLOR_GRAY2BGR), tr, r))
        if cv2.waitKey(120) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if "--show" in sys.argv:
        demo_show()
    else:
        unittest.main()
