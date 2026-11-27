"""UR5e 位置 IK：用 PhysX 雅可比做阻尼最小二乘，让腕部跟随目标点。

只做位置（3 维），姿态留给夹爪自身朝向；这是给录制用的"老师"，不是控制策略。
关节目标走 set_joint_position_targets，驱动参数用资产自带的那套，不覆盖。
"""
import math

import numpy as np


class ArmFollower:
    def __init__(self, prim_path="/World/Robot/Robot1Mount/ur5e",
                 eef_link="wrist_3_link", n_arm=6, gripper_dof=6,
                 damping=0.06, max_step=0.05):
        from isaacsim.core.prims import Articulation
        self.prim_path = prim_path
        self.art = Articulation(prim_path)
        self.art.initialize()
        self.n_dof = int(self.art.num_dof)
        self.dof_names = list(self.art.dof_names or [])
        self.n_arm = n_arm
        self.grip_idx = gripper_dof
        self.link_idx = self.art.get_link_index(eef_link)
        if self.link_idx is None:
            raise RuntimeError("%s 里找不到 link %s" % (prim_path, eef_link))
        # 有限差分标定：get_link_transforms 含固定 base（腕部=6），但 PhysX 的
        # 稠密雅可比不含 base，同一根腕部在雅可比里是 5。混用会越走越远（实测）。
        self.jac_idx = self.link_idx - 1
        # 不要给 articulation 的连杆另建 RigidPrim 视图：实测会在 app.update 时触发
        # "PxScene::fetchResults called illegally"，物理视图停更、关节读数变 4e8。
        # 张量视图自带的 get_link_transforms 一次给全部连杆（pos3+quat4, xyzw）。
        self.view = self.art._physics_view
        self.damping = damping
        self.max_step = max_step
        self.grip_max = math.radians(47.0)
        # 资产自带的隐式驱动刚度是 5e5 量级（URDF 导入按 N/m 写进来的），
        # 直接下发 0.05 rad 的目标就会把手臂打飞。重设成关节空间的合理量级。
        # 增益要压过重力静差：k=1000 时肩/肘静差 0.013–0.019 rad，
        # 而 IK 每帧只给 0.01 rad，指令会被静差吃掉（实测越追越偏）。
        kps = np.array([[3000.0, 3000.0, 2500.0, 1500.0, 1000.0, 800.0]])
        kds = np.array([[150.0, 150.0, 120.0, 80.0, 60.0, 50.0]])
        try:
            self.art.set_gains(kps=kps, kds=kds, joint_indices=list(range(n_arm)))
            self.art.set_max_efforts(np.array([[150.0] * n_arm]),
                                     joint_indices=list(range(n_arm)))
        except Exception as e:
            print("  !! 设置手臂增益失败（继续用资产自带值）：%r" % (e,))
        q = np.asarray(self.art.get_joint_positions(), dtype=np.float64)
        self.q0 = q.reshape(-1).copy()
        self.q_target = self.q0.copy()
        self.grip_target = 0.0
        self.last_err = None
        self._last_p = None
        self._last_quat = None
        try:
            self.state()
        except Exception:
            pass

    def reset(self):
        """每题开始把机械臂放回作者位（关节位直接 teleport，目标一起写）。"""
        self.art.set_joint_positions(self.q0.reshape(1, -1))
        self.art.set_joint_position_targets(self.q0.reshape(1, -1))
        self.q_target = self.q0.copy()
        self.grip_target = 0.0

    def state(self):
        q = np.asarray(self.art.get_joint_positions(), dtype=np.float64).reshape(-1)
        qd = np.asarray(self.art.get_joint_velocities(), dtype=np.float64).reshape(-1)
        try:
            T = np.asarray(self.view.get_link_transforms(),
                           dtype=np.float64)[0, self.link_idx]
            quat_xyzw = T[3:7]
            p = T[:3].copy()
            quat = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
            self._last_p, self._last_quat = p, quat
        except Exception:
            # tl.stop()/play() 之后的头一帧后端还没有 link transforms，回退到上次读数。
            if getattr(self, "_last_p", None) is None:
                raise
            p, quat = self._last_p, self._last_quat
        return q, qd, p, quat

    def step(self, target_xyz, gripper=0.0):
        q, qd, p, quat = self.state()
        err = np.asarray(target_xyz, dtype=np.float64) - p[:3]
        self.last_err = float(np.linalg.norm(err))
        J = np.asarray(self.art.get_jacobians(), dtype=np.float64)
        Jw = J[0, self.jac_idx, :3, :self.n_arm]
        A = Jw @ Jw.T + (self.damping ** 2) * np.eye(3)
        dq = Jw.T @ np.linalg.solve(A, err)
        n = float(np.linalg.norm(dq))
        if n > self.max_step:
            dq *= self.max_step / n
        self.grip_target = max(0.0, min(self.grip_max, float(gripper)))
        qt = q.copy()
        qt[:self.n_arm] = q[:self.n_arm] + dq
        if self.grip_idx < self.n_dof:
            qt[self.grip_idx] = self.grip_target
        self.q_target = qt
        self.art.set_joint_position_targets(qt.reshape(1, -1))
        return qt
