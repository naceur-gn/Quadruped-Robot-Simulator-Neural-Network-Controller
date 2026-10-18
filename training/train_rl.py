import os
import sys
import json
import time
import argparse
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from quadruped_env import QuadrupedEnv

# --- Policy Network with EXACTLY 1 HIDDEN LAYER -------------------------------
class ActorCritic(nn.Module):
    def __init__(self, obs_dim=32, act_dim=10, hidden_dim=64):
        super().__init__()
        # Actor: 1 hidden layer  (32 -> 64 -> 10)
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, act_dim)
        )
        self.log_std = nn.Parameter(torch.full((act_dim,), -0.5))

        # Critic: 1 hidden layer
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1)
        )

    def get_action_and_value(self, x, action=None):
        mean = self.actor(x)
        std = torch.exp(self.log_std)
        dist = torch.distributions.Normal(mean, std)

        if action is None:
            action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        value = self.critic(x).squeeze(-1)
        return action, log_prob, entropy, value

    def act_deterministic(self, x):
        with torch.no_grad():
            return torch.tanh(self.actor(x))


def export_weights(model, json_path, cpp_header_path):
    """
    Exports the trained 1-hidden-layer Actor weights to JSON and C++ header.
    Now with 32 inputs and 10 outputs (8 joint + 1 yaw_rate + 1 fwd_speed).
    """
    linear1 = model.actor[0]  # Linear(32, 64)
    linear2 = model.actor[2]  # Linear(64, 10)

    w1 = linear1.weight.detach().cpu().numpy()  # (64, 32)
    b1 = linear1.bias.detach().cpu().numpy()    # (64,)
    w2 = linear2.weight.detach().cpu().numpy()  # (10, 64)
    b2 = linear2.bias.detach().cpu().numpy()    # (10,)

    input_dim = w1.shape[1]
    hidden_dim = w1.shape[0]
    output_dim = w2.shape[0]

    # 1. Export to JSON
    data = {
        "input_dim": input_dim,
        "hidden_dim": hidden_dim,
        "output_dim": output_dim,
        "W1": w1.tolist(),
        "b1": b1.tolist(),
        "W2": w2.tolist(),
        "b2": b2.tolist()
    }
    with open(json_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n[export] Saved JSON weights to {json_path}")

    # 2. Export to C++ header
    cpp_lines = [
        "#pragma once\n",
        "// Auto-generated 1-hidden-layer policy weights from PyTorch RL training\n",
        f"// Network: {input_dim} inputs -> {hidden_dim} hidden (Tanh) -> {output_dim} outputs\n",
        f"// Outputs [0..7]: joint residuals, [8]: yaw rate cmd, [9]: forward speed cmd\n",
        "namespace robot {\n",
        "struct PretrainedPolicy {\n",
        f"    static constexpr int kInputDim = {input_dim};\n",
        f"    static constexpr int kHiddenDim = {hidden_dim};\n",
        f"    static constexpr int kOutputDim = {output_dim};\n\n",
    ]

    # W1
    cpp_lines.append(f"    static constexpr float W1[{hidden_dim}][{input_dim}] = {{\n")
    for r in range(hidden_dim):
        vals = ", ".join(f"{w1[r, c]:.7f}f" for c in range(input_dim))
        cpp_lines.append(f"        {{{vals}}},\n")
    cpp_lines.append("    };\n\n")

    # b1
    b1_str = ", ".join(f"{b:.7f}f" for b in b1)
    cpp_lines.append(f"    static constexpr float b1[{hidden_dim}] = {{{b1_str}}};\n\n")

    # W2
    cpp_lines.append(f"    static constexpr float W2[{output_dim}][{hidden_dim}] = {{\n")
    for r in range(output_dim):
        vals = ", ".join(f"{w2[r, c]:.7f}f" for c in range(hidden_dim))
        cpp_lines.append(f"        {{{vals}}},\n")
    cpp_lines.append("    };\n\n")

    # b2
    b2_str = ", ".join(f"{b:.7f}f" for b in b2)
    cpp_lines.append(f"    static constexpr float b2[{output_dim}] = {{{b2_str}}};\n")
    cpp_lines.append("};\n} // namespace robot\n")

    with open(cpp_header_path, "w") as f:
        f.writelines(cpp_lines)
    print(f"[export] Saved C++ header weights to {cpp_header_path}")


def train(total_timesteps=500000, num_envs=32, num_steps=128, lr=3e-4):
    device = torch.device("cpu")
    print(f"==================================================================")
    print(f"  QUADRUPED RL TRAINING: NEURAL NETWORK NAVIGATION")
    print(f"==================================================================")
    print(f"  Architecture   : 1 Hidden Layer (32 obs -> 64 Tanh -> 10 actions)")
    print(f"  Actions        : 8 joint residuals + 1 yaw_rate + 1 fwd_speed")
    print(f"  Task           : NN steers & walks to reach target without falling")
    print(f"  Parallel Envs  : {num_envs}")
    print(f"  Rollout Steps  : {num_steps} per env ({num_envs * num_steps} steps/iter)")
    print(f"  Total Steps    : {total_timesteps:,} steps")
    print(f"  Device         : {device}")
    print(f"==================================================================\n")

    env = QuadrupedEnv(num_envs=num_envs)
    model = ActorCritic(obs_dim=env.obs_dim, act_dim=env.action_dim, hidden_dim=64).to(device)
    optimizer = optim.Adam(model.parameters(), lr=lr, eps=1e-5)

    batch_size = num_envs * num_steps
    num_updates = total_timesteps // batch_size

    # Rollout buffers
    obs_buf = torch.zeros((num_steps, num_envs, env.obs_dim), device=device)
    act_buf = torch.zeros((num_steps, num_envs, env.action_dim), device=device)
    logp_buf = torch.zeros((num_steps, num_envs), device=device)
    rew_buf = torch.zeros((num_steps, num_envs), device=device)
    done_buf = torch.zeros((num_steps, num_envs), device=device)
    val_buf = torch.zeros((num_steps, num_envs), device=device)

    obs = torch.tensor(env.reset(), dtype=torch.float32, device=device)
    start_time = time.time()
    ep_rewards = []
    ep_lens = []
    total_goals_reached = 0
    total_falls = 0

    cur_ep_rew = np.zeros(num_envs)
    cur_ep_len = np.zeros(num_envs)

    for update in range(1, num_updates + 1):
        # 1. Collect rollouts
        for step in range(num_steps):
            obs_buf[step] = obs
            with torch.no_grad():
                action, logp, _, val = model.get_action_and_value(obs)

            act_buf[step] = action
            logp_buf[step] = logp
            val_buf[step] = val

            # Step environment
            next_obs, rewards, dones, info = env.step(action.cpu().numpy())
            rew_buf[step] = torch.tensor(rewards, dtype=torch.float32, device=device)
            done_buf[step] = torch.tensor(dones, dtype=torch.float32, device=device)

            cur_ep_rew += rewards
            cur_ep_len += 1

            total_goals_reached += int(np.sum(info["reached"]))
            total_falls += int(np.sum(info["fell"]))

            for i in range(num_envs):
                if dones[i]:
                    ep_rewards.append(cur_ep_rew[i])
                    ep_lens.append(cur_ep_len[i])
                    cur_ep_rew[i] = 0.0
                    cur_ep_len[i] = 0

            obs = torch.tensor(next_obs, dtype=torch.float32, device=device)

        # 2. Compute GAE
        with torch.no_grad():
            _, _, _, next_val = model.get_action_and_value(obs)
            gamma = 0.99
            gae_lambda = 0.95
            advantages = torch.zeros_like(rew_buf, device=device)
            lastgaelam = 0.0
            for t in reversed(range(num_steps)):
                if t == num_steps - 1:
                    nextnonterminal = 1.0 - done_buf[t]
                    nextvalues = next_val
                else:
                    nextnonterminal = 1.0 - done_buf[t + 1]
                    nextvalues = val_buf[t + 1]
                delta = rew_buf[t] + gamma * nextvalues * nextnonterminal - val_buf[t]
                advantages[t] = lastgaelam = delta + gamma * gae_lambda * nextnonterminal * lastgaelam
            returns = advantages + val_buf

        # Flatten batch
        b_obs = obs_buf.reshape(-1, env.obs_dim)
        b_act = act_buf.reshape(-1, env.action_dim)
        b_logp = logp_buf.reshape(-1)
        b_adv = advantages.reshape(-1)
        b_ret = returns.reshape(-1)

        b_adv = (b_adv - b_adv.mean()) / (b_adv.std() + 1e-8)

        # 3. PPO Epochs
        ppo_epochs = 4
        minibatch_size = 256
        b_inds = np.arange(batch_size)

        for _ in range(ppo_epochs):
            np.random.shuffle(b_inds)
            for start in range(0, batch_size, minibatch_size):
                end = start + minibatch_size
                mb_inds = b_inds[start:end]

                _, new_logp, entropy, new_val = model.get_action_and_value(b_obs[mb_inds], b_act[mb_inds])
                log_ratio = new_logp - b_logp[mb_inds]
                ratio = torch.exp(log_ratio)

                mb_adv = b_adv[mb_inds]

                # Policy loss
                surr1 = ratio * mb_adv
                surr2 = torch.clamp(ratio, 0.8, 1.2) * mb_adv
                policy_loss = -torch.min(surr1, surr2).mean()

                # Value loss
                value_loss = 0.5 * ((new_val - b_ret[mb_inds]) ** 2).mean()

                # Entropy loss
                entropy_loss = entropy.mean()

                loss = policy_loss + 0.5 * value_loss - 0.01 * entropy_loss

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 0.5)
                optimizer.step()

        # Real-time formatted training progression
        current_steps = update * batch_size
        pct = (current_steps / total_timesteps) * 100.0
        elapsed = time.time() - start_time
        fps = int(current_steps / max(elapsed, 1e-4))
        avg_rew = np.mean(ep_rewards[-50:]) if len(ep_rewards) > 0 else 0.0
        avg_len = np.mean(ep_lens[-50:]) if len(ep_lens) > 0 else 0.0
        avg_dist = np.mean(info["dist"])
        avg_hdg = np.mean(info["heading_err"]) * (180.0 / np.pi)

        print(f"Step [{current_steps:7d} / {total_timesteps:7d} | {pct:5.1f}%] | "
              f"Rew: {avg_rew:+7.1f} | EpLen: {avg_len:5.1f} | "
              f"Goals: {total_goals_reached:4d} | Falls: {total_falls:4d} | "
              f"Dist: {avg_dist:4.2f}m | HdgErr: {avg_hdg:5.1f}° | FPS: {fps:5d}")

    # Export final weights
    os.makedirs("src/robot", exist_ok=True)
    export_weights(model, "training/model_weights.json", "src/robot/policy_weights_default.hpp")
    print(f"==================================================================")
    print(f"  TRAINING COMPLETE: {total_timesteps:,} steps in {time.time() - start_time:.1f}s")
    print(f"  Total Coordinates Reached : {total_goals_reached:,}")
    print(f"  Total Falls               : {total_falls:,}")
    print(f"  Final Mean Episode Length : {avg_len:.1f} steps")
    print(f"==================================================================")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Real RL Quadruped Training - Neural Network Navigation")
    parser.add_argument("--steps", type=int, default=500000, help="Total training steps (default: 500,000)")
    parser.add_argument("--envs", type=int, default=32, help="Parallel environments (default: 32)")
    parser.add_argument("--lr", type=float, default=3e-4, help="Learning rate (default: 3e-4)")
    args = parser.parse_args()

    train(total_timesteps=args.steps, num_envs=args.envs, lr=args.lr)
