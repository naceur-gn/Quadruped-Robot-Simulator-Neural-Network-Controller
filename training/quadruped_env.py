import numpy as np

class QuadrupedEnv:
    """
    Vectorized quadruped simulation environment for RL-based target navigation.

    The neural network controls EVERYTHING:
      - 8 joint residuals (on top of a trotting CPG)
      - 1 yaw rate command  (how fast to turn left/right)
      - 1 forward speed command (how fast to walk forward)

    Observation (32 dims):
      [0:2]   heading error (sin, cos) to target in body frame
      [2]     normalized distance to target
      [3:6]   projected gravity in body frame
      [6:9]   angular velocity (roll_dot, yaw_dot, pitch_dot)
      [9:12]  linear velocity in body frame (vx, vy, vz)
      [12:20] joint position errors from nominal
      [20:28] joint velocities
      [28:30] gait clock (sin, cos)
      [30]    current yaw rate
      [31]    current forward speed

    Action (10 dims):
      [0:8]   joint residuals
      [8]     yaw rate command  (-1..+1 -> scaled to rad/s)
      [9]     forward speed command (-1..+1 -> scaled to m/s)
    """
    def __init__(self, num_envs=32, dt=0.01):
        self.num_envs = num_envs
        self.dt = dt
        self.obs_dim = 32
        self.action_dim = 10

        # Physical constants
        self.mass = 12.0
        self.gravity = 9.81
        self.inertia = np.array([0.09, 0.22, 0.18])  # Ixx, Iyy, Izz
        self.inv_inertia = 1.0 / self.inertia

        # Geometry
        self.hip_x = 0.1881
        self.hip_z = 0.1265
        self.thigh_len = 0.213
        self.calf_len = 0.213
        self.nominal_height = 0.2968

        # Nominal standing pose & trotting CPG amplitudes
        self.q_nominal = np.array([0.8, -1.6, 0.8, -1.6, 0.8, -1.6, 0.8, -1.6])
        self.cpg_amp_hip = 0.22
        self.cpg_amp_knee = 0.42
        self.action_scale = 0.25

        # Joint motor gains
        self.kp_joint = 65.0
        self.kd_joint = 3.5

        # Ground contact parameters
        self.k_ground = 1800.0
        self.c_ground = 80.0
        self.friction_mu = 0.8

        # Gait clock
        self.gait_freq = 1.6  # Hz

        # RL-controlled steering scales
        self.max_yaw_rate = 2.0       # rad/s max yaw rate
        self.max_fwd_speed = 0.8      # m/s max forward speed

        # Allocate state arrays
        self.pos = np.zeros((num_envs, 3))       # x, y, z
        self.vel = np.zeros((num_envs, 3))       # vx, vy, vz
        self.rpy = np.zeros((num_envs, 3))       # roll, pitch, yaw
        self.omega = np.zeros((num_envs, 3))     # wx, wy, wz
        self.q = np.zeros((num_envs, 8))         # joint positions
        self.dq = np.zeros((num_envs, 8))        # joint velocities
        self.phase = np.zeros(num_envs)          # gait phase (rad)
        self.prev_actions = np.zeros((num_envs, 10))

        # RL yaw/speed state
        self.cmd_yaw_rate = np.zeros(num_envs)
        self.cmd_fwd_speed = np.zeros(num_envs)

        # Target coordinate navigation
        self.target_pos = np.zeros((num_envs, 2))  # (x_target, z_target)
        self.prev_dist = np.zeros(num_envs)
        self.targets_reached = np.zeros(num_envs, dtype=np.int32)
        self.step_count = np.zeros(num_envs, dtype=np.int32)
        self.total_env_steps = 0
        self.max_episode_steps = 800

        self.reset()

    def _spawn_targets(self, env_ids):
        n = len(env_ids)
        if n == 0:
            return
        angles = np.random.uniform(0.0, 2.0 * np.pi, size=n)
        dists = np.random.uniform(1.5, 4.0, size=n)
        self.target_pos[env_ids, 0] = self.pos[env_ids, 0] + dists * np.cos(angles)
        self.target_pos[env_ids, 1] = self.pos[env_ids, 2] + dists * np.sin(angles)

        dx = self.target_pos[env_ids, 0] - self.pos[env_ids, 0]
        dz = self.target_pos[env_ids, 1] - self.pos[env_ids, 2]
        self.prev_dist[env_ids] = np.sqrt(dx * dx + dz * dz)

    def reset_idx(self, env_ids):
        n = len(env_ids)
        if n == 0:
            return
        self.pos[env_ids, 0] = 0.0
        self.pos[env_ids, 1] = self.nominal_height + np.random.uniform(-0.01, 0.01, size=n)
        self.pos[env_ids, 2] = 0.0

        self.vel[env_ids] = 0.0
        self.rpy[env_ids] = np.random.uniform(-0.04, 0.04, size=(n, 3))
        self.omega[env_ids] = 0.0

        self.q[env_ids] = self.q_nominal + np.random.uniform(-0.04, 0.04, size=(n, 8))
        self.dq[env_ids] = 0.0
        self.phase[env_ids] = np.random.uniform(0.0, 2.0 * np.pi, size=n)
        self.prev_actions[env_ids] = 0.0
        self.step_count[env_ids] = 0
        self.cmd_yaw_rate[env_ids] = 0.0
        self.cmd_fwd_speed[env_ids] = 0.0

        self._spawn_targets(env_ids)

    def reset(self):
        self.reset_idx(np.arange(self.num_envs))
        return self._get_obs()

    def _angle_wrap(self, a):
        """Wrap angle to [-pi, pi]"""
        return (a + np.pi) % (2.0 * np.pi) - np.pi

    def _get_obs(self):
        # 1. Heading error to target (2 dims: sin, cos of angle error)
        dx_w = self.target_pos[:, 0] - self.pos[:, 0]
        dz_w = self.target_pos[:, 1] - self.pos[:, 2]
        dist = np.maximum(1e-4, np.sqrt(dx_w * dx_w + dz_w * dz_w))

        # Angle from robot to target in world frame
        target_angle = np.arctan2(dz_w, dx_w)
        # Heading error = target_angle - current_yaw
        heading_err = self._angle_wrap(target_angle - self.rpy[:, 2])
        heading_sin = np.sin(heading_err)
        heading_cos = np.cos(heading_err)
        heading_features = np.stack([heading_sin, heading_cos], axis=1)  # (N, 2)

        # 2. Normalized distance
        norm_dist = np.tanh(dist / 4.0)[:, None]  # (N, 1)

        # 3. Projected gravity in body frame (3 dims)
        roll = self.rpy[:, 0]
        pitch = self.rpy[:, 1]
        gx = np.sin(pitch)
        gy = -np.cos(pitch) * np.cos(roll)
        gz = np.cos(pitch) * np.sin(roll)
        gravity_b = np.stack([gx, gy, gz], axis=1)  # (N, 3)

        # 4. Angular velocity (3 dims)
        omega = self.omega  # (N, 3)

        # 5. Linear velocity in body frame (3 dims)
        yaw = self.rpy[:, 2]
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        vx_b =  self.vel[:, 0] * cos_y + self.vel[:, 2] * sin_y
        vz_b = -self.vel[:, 0] * sin_y + self.vel[:, 2] * cos_y
        vy_b =  self.vel[:, 1]
        vel_b = np.stack([vx_b, vy_b, vz_b], axis=1)  # (N, 3)

        # 6. Joint errors from nominal standing pose (8 dims)
        q_err = self.q - self.q_nominal  # (N, 8)

        # 7. Joint velocities (8 dims)
        dq = self.dq  # (N, 8)

        # 8. Gait clock (2 dims)
        clock = np.stack([np.sin(self.phase), np.cos(self.phase)], axis=1)  # (N, 2)

        # 9. Current yaw rate and forward speed commands (2 dims) - feedback to NN
        cmd = np.stack([self.cmd_yaw_rate / self.max_yaw_rate,
                        self.cmd_fwd_speed / self.max_fwd_speed], axis=1)  # (N, 2)

        # Total: 2 + 1 + 3 + 3 + 3 + 8 + 8 + 2 + 2 = 32
        obs = np.concatenate([heading_features, norm_dist, gravity_b, omega, vel_b,
                              q_err, dq, clock, cmd], axis=1)
        return obs.astype(np.float32)

    def step(self, actions):
        actions = np.clip(actions, -1.0, 1.0)
        self.step_count += 1
        self.total_env_steps += self.num_envs

        # --- Extract NN steering commands ---
        joint_actions = actions[:, :8]
        yaw_rate_cmd = actions[:, 8] * self.max_yaw_rate    # scale [-1,1] -> [-2,2] rad/s
        fwd_speed_cmd = actions[:, 9] * self.max_fwd_speed  # scale [-1,1] -> [-0.8,0.8] m/s

        # Smooth command tracking (low-pass filter)
        alpha_yaw = 0.15
        alpha_spd = 0.15
        self.cmd_yaw_rate += alpha_yaw * (yaw_rate_cmd - self.cmd_yaw_rate)
        self.cmd_fwd_speed += alpha_spd * (fwd_speed_cmd - self.cmd_fwd_speed)

        # --- Apply yaw rate to orientation (NN controls turning!) ---
        self.rpy[:, 2] += self.cmd_yaw_rate * self.dt
        self.rpy[:, 2] = self._angle_wrap(self.rpy[:, 2])

        # --- Apply forward speed to velocity (NN controls forward motion!) ---
        yaw = self.rpy[:, 2]
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        # Desired world velocity from NN forward speed command
        des_vx = self.cmd_fwd_speed * cos_y
        des_vz = self.cmd_fwd_speed * sin_y
        # Strong velocity blend so the command actually moves the robot
        alpha_v = 0.3
        self.vel[:, 0] += alpha_v * (des_vx - self.vel[:, 0])
        self.vel[:, 2] += alpha_v * (des_vz - self.vel[:, 2])

        # Advance gait clock - scale frequency with speed (faster walking when faster)
        speed_ratio = np.clip(np.abs(self.cmd_fwd_speed) / self.max_fwd_speed, 0.3, 1.0)
        self.phase = (self.phase + 2.0 * np.pi * self.gait_freq * speed_ratio * self.dt) % (2.0 * np.pi)

        # Scale CPG amplitude with forward speed (bigger steps when moving faster)
        speed_scale = np.clip(np.abs(self.cmd_fwd_speed) / self.max_fwd_speed, 0.2, 1.0)
        amp_hip = self.cpg_amp_hip * speed_scale
        amp_knee = self.cpg_amp_knee * speed_scale

        # Base trotting CPG angles
        leg_phases = np.stack([
            self.phase,
            self.phase + np.pi,
            self.phase + np.pi,
            self.phase
        ], axis=1)  # (N, 4)

        q_base = np.zeros((self.num_envs, 8))
        for leg in range(4):
            phi = leg_phases[:, leg]
            q_base[:, 2 * leg]     = self.q_nominal[2 * leg]     + amp_hip  * np.sin(phi)
            q_base[:, 2 * leg + 1] = self.q_nominal[2 * leg + 1] - amp_knee * np.maximum(0.0, -np.cos(phi))

        # Target joint angles: CPG + RL policy residuals
        q_target = q_base + self.action_scale * joint_actions

        # Hip attachment layout
        hx = np.array([ self.hip_x,  self.hip_x, -self.hip_x, -self.hip_x])
        hz = np.array([-self.hip_z,  self.hip_z, -self.hip_z,  self.hip_z])

        # PD motor torques
        torque = self.kp_joint * (q_target - self.q) - self.kd_joint * self.dq
        torque = np.clip(torque, -35.0, 35.0)

        total_force = np.zeros((self.num_envs, 3))
        total_torque = np.zeros((self.num_envs, 3))

        total_force[:, 1] -= self.mass * self.gravity

        roll = self.rpy[:, 0]
        pitch = self.rpy[:, 1]

        cos_p, sin_p = np.cos(pitch), np.sin(pitch)
        cos_r, sin_r = np.cos(roll), np.sin(roll)

        # Compute ground reaction forces
        for leg in range(4):
            qh = self.q[:, 2 * leg]
            qk = self.q[:, 2 * leg + 1]
            dqh = self.dq[:, 2 * leg]
            dqk = self.dq[:, 2 * leg + 1]

            c_h = np.cos(qh)
            s_h = np.sin(qh)
            c_hk = np.cos(qh + qk)
            s_hk = np.sin(qh + qk)

            fx_rel = self.thigh_len * s_h + self.calf_len * s_hk
            fy_rel = -self.thigh_len * c_h - self.calf_len * c_hk

            v_rel_x = (self.thigh_len * c_h + self.calf_len * c_hk) * dqh + (self.calf_len * c_hk) * dqk
            v_rel_y = (self.thigh_len * s_h + self.calf_len * s_hk) * dqh + (self.calf_len * s_hk) * dqk

            hip_world_x = self.pos[:, 0] + (hx[leg] * cos_y - hz[leg] * sin_y)
            hip_world_y = self.pos[:, 1]
            hip_world_z = self.pos[:, 2] + (hx[leg] * sin_y + hz[leg] * cos_y)

            foot_x = hip_world_x + fx_rel * cos_y
            foot_y = hip_world_y + fy_rel
            foot_z = hip_world_z + fx_rel * sin_y

            rx = foot_x - self.pos[:, 0]
            ry = foot_y - self.pos[:, 1]
            rz = foot_z - self.pos[:, 2]

            w_cross_r_x = self.omega[:, 1] * rz - self.omega[:, 2] * ry
            w_cross_r_y = self.omega[:, 2] * rx - self.omega[:, 0] * rz
            w_cross_r_z = self.omega[:, 0] * ry - self.omega[:, 1] * rx

            foot_vx = self.vel[:, 0] + w_cross_r_x + v_rel_x * cos_y
            foot_vy = self.vel[:, 1] + w_cross_r_y + v_rel_y
            foot_vz = self.vel[:, 2] + w_cross_r_z + v_rel_x * sin_y

            penetration = np.maximum(0.0, -foot_y)
            fn = np.where(penetration > 0.0, self.k_ground * penetration - self.c_ground * foot_vy, 0.0)
            fn = np.maximum(0.0, fn)

            fx = -np.clip(self.c_ground * foot_vx, -self.friction_mu * fn, self.friction_mu * fn)
            fz = -np.clip(self.c_ground * foot_vz, -self.friction_mu * fn, self.friction_mu * fn)

            total_force[:, 0] += fx
            total_force[:, 1] += fn
            total_force[:, 2] += fz

            total_torque[:, 0] += (ry * fz - rz * fn)
            total_torque[:, 1] += (rz * fx - rx * fz)
            total_torque[:, 2] += (rx * fn - ry * fx)

        # Trunk dynamics
        acc_lin = total_force / self.mass
        acc_ang_x = total_torque[:, 0] / self.inertia[0]
        acc_ang_y = total_torque[:, 1] / self.inertia[1]
        acc_ang_z = total_torque[:, 2] / self.inertia[2]

        self.vel += acc_lin * self.dt
        self.pos += self.vel * self.dt

        self.omega[:, 0] += acc_ang_x * self.dt
        self.omega[:, 1] += acc_ang_y * self.dt
        self.omega[:, 2] += acc_ang_z * self.dt

        self.rpy[:, 0] += self.omega[:, 0] * self.dt
        self.rpy[:, 1] += self.omega[:, 2] * self.dt
        # Yaw already updated by NN yaw rate command above

        # Damping
        self.vel[:, 0] *= (1.0 - 0.1 * self.dt)
        self.vel[:, 2] *= (1.0 - 0.1 * self.dt)
        self.omega *= (1.0 - 0.5 * self.dt)

        # Joint dynamics
        acc_q = torque / 0.05
        self.dq += acc_q * self.dt
        self.dq = np.clip(self.dq, -15.0, 15.0)
        self.q += self.dq * self.dt
        self.q = np.clip(self.q, self.q_nominal - 0.65, self.q_nominal + 0.65)

        # --- Reward Calculation ---
        dx_w = self.target_pos[:, 0] - self.pos[:, 0]
        dz_w = self.target_pos[:, 1] - self.pos[:, 2]
        curr_dist = np.sqrt(dx_w * dx_w + dz_w * dz_w)

        # Heading error: how well is the robot facing the target?
        target_angle = np.arctan2(dz_w, dx_w)
        heading_err = self._angle_wrap(target_angle - self.rpy[:, 2])

        # Forward velocity projecting along the direction robot is facing
        fwd_vel = self.vel[:, 0] * cos_y + self.vel[:, 2] * sin_y

        # Velocity toward target (world frame)
        unit_dx = dx_w / np.maximum(curr_dist, 1e-4)
        unit_dz = dz_w / np.maximum(curr_dist, 1e-4)
        v_toward = self.vel[:, 0] * unit_dx + self.vel[:, 2] * unit_dz

        # 1. Progress toward target (distance reduction) - STRONG
        r_progress = 50.0 * (self.prev_dist - curr_dist)

        # 2. Velocity toward target reward - STRONG
        r_velocity = 8.0 * v_toward

        # 3. Forward speed reward (reward commanding forward motion)
        r_fwd_cmd = 2.0 * np.clip(self.cmd_fwd_speed, 0.0, self.max_fwd_speed) * np.cos(heading_err)

        # 4. Heading alignment (reward for facing the target)
        r_heading = 3.0 * np.cos(heading_err)

        # 5. Yaw rate penalty when already aligned
        r_yaw_smooth = -0.3 * np.square(self.cmd_yaw_rate) * np.square(np.cos(heading_err))

        # 6. Survival reward
        r_alive = 0.2

        # 7. Posture preservation (stay upright!)
        y_height = self.pos[:, 1]
        r_posture = -10.0 * np.square(y_height - self.nominal_height) \
                    - 1.5 * (roll**2 + pitch**2)

        # 8. Smooth action penalty
        r_act = -0.003 * np.sum(np.square(actions), axis=1) \
                - 0.003 * np.sum(np.square(actions - self.prev_actions), axis=1)
        self.prev_actions = actions.copy()

        # 9. Coordinate reached bonus
        reached = curr_dist < 0.50
        r_reached = np.where(reached, 80.0, 0.0)
        self.targets_reached[reached] += 1

        reward = r_progress + r_velocity + r_fwd_cmd + r_heading + \
                 r_yaw_smooth + r_alive + r_posture + r_act + r_reached

        # Fall detection
        fell = (y_height < 0.17) | (np.abs(roll) > 0.60) | (np.abs(pitch) > 0.60)
        timeout = self.step_count >= self.max_episode_steps
        dones = fell | timeout

        reward[fell] -= 40.0

        # Spawn new target for environments that reached the goal
        if np.any(reached & ~dones):
            reached_ids = np.where(reached & ~dones)[0]
            self._spawn_targets(reached_ids)

        self.prev_dist = curr_dist.copy()

        obs = self._get_obs()

        # Auto-reset fallen environments
        if np.any(dones):
            env_ids = np.where(dones)[0]
            self.reset_idx(env_ids)

        return obs, reward.astype(np.float32), dones, {
            "fell": fell,
            "timeout": timeout,
            "reached": reached,
            "dist": curr_dist,
            "heading_err": np.abs(heading_err)
        }
