"""MVP 제어기 — Pure Pursuit(횡) + 목표속도 추종(종) + 안전 가드.

경로: Nx2 폴리라인 (build_route 출력). ego 기준점 = 후축 중심 (VTD와 동일).
"""
import numpy as np

MAX_STEER = 0.48
ACCEL_MIN, ACCEL_MAX = -6.0, 2.5     # 평가 감점 고려한 안락 한계
WHEELBASE = 2.944
A_LAT_MAX = 2.0                       # 곡률 감속 기준 [m/s²]
V_DEFAULT = 50 / 3.6                  # 국토교통부 도심 기본 50km/h


class PathTracker:
    def __init__(self, path_pts, v_limit=V_DEFAULT):
        self.pts = np.asarray(path_pts, float)
        d = np.hypot(*np.diff(self.pts, axis=0).T)
        self.s = np.concatenate([[0], np.cumsum(d)])
        self.v_limit = v_limit
        self._last_i = 0
        # 사전 곡률 → 지점별 속도 상한 (전방 최소값으로 미리 감속)
        self.curv = self._curvature()
        v_curve = np.sqrt(A_LAT_MAX / np.maximum(np.abs(self.curv), 1e-4))
        self.v_profile = np.minimum(v_limit, v_curve)
        # 뒤에서 앞으로 감속 한계 전파 (v² = v_next² + 2·a·ds)
        for i in range(len(self.v_profile) - 2, -1, -1):
            ds = self.s[i + 1] - self.s[i]
            self.v_profile[i] = min(self.v_profile[i],
                                    np.sqrt(self.v_profile[i + 1] ** 2 + 2 * 2.0 * ds))
        self.v_profile[-1] = 0.0  # 종점 정지

    def _curvature(self):
        p = self.pts
        k = np.zeros(len(p))
        if len(p) > 2:
            d1 = np.gradient(p, self.s, axis=0)
            d2 = np.gradient(d1, self.s, axis=0)
            num = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
            den = (d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5
            k = num / np.maximum(den, 1e-6)
        return k

    def localize(self, x, y, window=400):
        """경로상 최근접 인덱스 (직전 위치 주변 탐색으로 루프/교차 오매칭 방지)."""
        lo = max(0, self._last_i - 50)
        hi = min(len(self.pts), self._last_i + window)
        d = np.hypot(self.pts[lo:hi, 0] - x, self.pts[lo:hi, 1] - y)
        i = lo + int(np.argmin(d))
        self._last_i = i
        return i, d[i - lo]

    def finished(self, x, y, tol=3.0):
        return self._last_i >= len(self.pts) - 5 and \
            np.hypot(self.pts[-1, 0] - x, self.pts[-1, 1] - y) < tol


class Controller:
    def __init__(self, tracker: PathTracker):
        self.tr = tracker
        self.v_est = 0.0
        self._prev = None  # (x, y, t)

    def update(self, x, y, heading, dt=0.05):
        """상태 → (steering, accel, turn_signal, debug)."""
        # 속도 추정 (TCP엔 ego 속도가 없음 → pose 미분 + 저역필터)
        if self._prev is not None:
            v_raw = np.hypot(x - self._prev[0], y - self._prev[1]) / dt
            self.v_est += 0.4 * (v_raw - self.v_est)
        self._prev = (x, y)

        i, lat_err = self.tr.localize(x, y)

        # --- 횡: Pure Pursuit (후축 기준) ---
        ld = np.clip(0.6 * self.v_est + 3.0, 4.0, 18.0)
        s_target = self.tr.s[i] + ld
        j = int(np.searchsorted(self.tr.s, s_target))
        j = min(j, len(self.tr.pts) - 1)
        tx, ty = self.tr.pts[j]
        alpha = np.arctan2(ty - y, tx - x) - heading
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        steer = np.arctan2(2 * WHEELBASE * np.sin(alpha), ld)
        steer = float(np.clip(steer, -MAX_STEER, MAX_STEER))

        # --- 종: 전방 속도 프로파일 추종 ---
        v_target = float(self.tr.v_profile[min(j, len(self.tr.v_profile) - 1)])
        accel = float(np.clip(1.2 * (v_target - self.v_est), ACCEL_MIN, ACCEL_MAX))

        # --- 안전 가드 ---
        if not (np.isfinite(steer) and np.isfinite(accel)):
            steer, accel = 0.0, -2.0

        debug = dict(i=i, lat_err=float(lat_err), v_est=self.v_est,
                     v_target=v_target, ld=float(ld))
        return steer, accel, 0, debug
