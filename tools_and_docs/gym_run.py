import numpy as np
import gymnasium as gym
import argparse
import time
import itertools
import subprocess
import shutil

# from gymnasium.utils.env_checker import check_env
# from stable_baselines3 import PPO
# from stable_baselines3.common.env_checker import check_env as sb3_check_env

from aas_gym.aas_env import AASEnv


# Register the environment so we can create it with gym.make()
gym.register(
    id="AASEnv-v0",
    entry_point=AASEnv,
)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, default="step", choices=["step", "speedup", "vectorenv-speedup", "learn"])
    parser.add_argument("--repetitions", type=int, default=1),
    parser.add_argument("--autopilot", type=str, default="px4", choices=["px4", "ardupilot"])
    parser.add_argument("--camera", action=argparse.BooleanOptionalAction, default=True, help="Enable/Disable Camera")
    parser.add_argument("--lidar", action=argparse.BooleanOptionalAction, default=True, help="Enable/Disable Lidar")
    parser.add_argument("--num_quads", type=int, default=1)
    parser.add_argument("--gpu", action=argparse.BooleanOptionalAction, default=True, help="Enable/Disable GPU bindings")
    args = parser.parse_args()

    if args.mode == "step":
        env = gym.make(
            "AASEnv-v0",
            gym_freq_hz=1,
            autopilot=args.autopilot,
            camera=args.camera,
            lidar=args.lidar,
            num_quads=args.num_quads,
            render_mode="human",
            gpu=args.gpu
        )
        obs, info = env.reset()
        print(f"Reset result -- Obs: {obs}")
        for i in itertools.count():
            user_input = input("Press Enter to step, 'r' then Enter to reset, 'q' then Enter to exit...")
            stripped_input = user_input.strip().lower()
            if stripped_input and stripped_input in ('q', 'quit'):
                break
            if stripped_input and stripped_input in ('r', 'reset'):
                obs, info = env.reset()
                print(f"\nReset result -- Obs: {obs}")
            else:
                rnd_action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(rnd_action)
                print(f"\nStep {i} -- action: {rnd_action} result -- Obs: {obs}, Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}")
        print("\nClosing environment.")
        env.close()

    elif args.mode == "speedup":
        REPETITIONS = args.repetitions
        CTRL_FREQ_HZ = 50
        env = gym.make(
            "AASEnv-v0",
            instance=1,
            gym_freq_hz=CTRL_FREQ_HZ,
            autopilot=args.autopilot,
            camera=args.camera,
            lidar=args.lidar,
            num_quads=args.num_quads,
            render_mode="ansi", # "ansi" for progress bar, "human" for GUI
            gpu=args.gpu
        )
        TIME_TO_SIMULATE_SEC = 250
        STEPS = TIME_TO_SIMULATE_SEC * CTRL_FREQ_HZ
        print(f"Starting speed test: {REPETITIONS} runs of {STEPS} steps each.")
        run_times = []
        for _rep in range(REPETITIONS):
            obs, info = env.reset()
            start_time = time.time()
            for _step in range(STEPS):
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                if terminated or truncated:
                    obs, info = env.reset()
            duration = time.time() - start_time
            run_times.append(duration)
        avg_time = np.mean(run_times)
        std_time = np.std(run_times)
        all_speedups = [TIME_TO_SIMULATE_SEC / t for t in run_times]
        avg_speedup = np.mean(all_speedups)
        std_speedup = np.std(all_speedups)
        all_throughputs = [STEPS / t for t in run_times]
        avg_throughput = np.mean(all_throughputs)
        std_throughput = np.std(all_throughputs)
        print(f"\nAvg Duration:       {avg_time:.2f}s ± {std_time:.2f}s")
        print(f"Avg Step Time:      {(avg_time / STEPS) * 1000:.3f} ms")
        print(f"Avg Speedup:        {avg_speedup:.2f}x ± {std_speedup:.2f}x wall-clock")
        print(f"Avg Throughput:     {avg_throughput:.2f} ± {std_throughput:.2f} steps/second")
        env.close()

    elif args.mode == "vectorenv-speedup":
        REPETITIONS = args.repetitions
        NUM_ENVS = 2 # Number of parallel environments (adjust based on CPU/RAM and GPU/VRAM usage, check with htop and nvidia-smi)
        CTRL_FREQ_HZ = 50
        TIME_TO_SIMULATE_SEC = 250
        STEPS_PER_ENV = TIME_TO_SIMULATE_SEC * CTRL_FREQ_HZ 
        print(f"Starting parallel speed test: {REPETITIONS} runs with {NUM_ENVS} envs, stepping each for {STEPS_PER_ENV} steps")
        def make_env(rank, freq_hz):
            def _init():
                return gym.make(
                    "AASEnv-v0",
                    instance=rank,
                    gym_freq_hz=freq_hz,
                    autopilot=args.autopilot,
                    camera=args.camera,
                    lidar=args.lidar,
                    num_quads=args.num_quads,
                    render_mode=None,
                    gpu=args.gpu
                )
            return _init
        env_fns = [make_env(i, CTRL_FREQ_HZ) for i in range(NUM_ENVS)]
        envs = gym.vector.AsyncVectorEnv(env_fns)
        print("Running the test with render_mode=None")
        run_times = []
        for _rep in range(REPETITIONS):
            obs, info = envs.reset()
            start_time = time.time()
            for _step in range(STEPS_PER_ENV):
                actions = envs.action_space.sample() # Returns array of shape (NUM_ENVS, action_dim)
                obs, rewards, terminateds, truncateds, infos = envs.step(actions) # AsyncVectorEnv automatically resets individual envs when they terminate/truncate
            duration = time.time() - start_time
            run_times.append(duration)
        avg_time = np.mean(run_times)
        std_time = np.std(run_times)
        all_speedups = [(TIME_TO_SIMULATE_SEC * NUM_ENVS) / t for t in run_times]
        avg_speedup = np.mean(all_speedups)
        std_speedup = np.std(all_speedups)
        all_throughputs = [(STEPS_PER_ENV * NUM_ENVS) / t for t in run_times]
        avg_throughput = np.mean(all_throughputs)
        std_throughput = np.std(all_throughputs)
        print(f"\nAvg Duration:       {avg_time:.2f}s ± {std_time:.2f}s")
        print(f"Avg Speedup:        {avg_speedup:.2f}x ± {std_speedup:.2f}x wall-clock (aggregate)")
        print(f"Avg Throughput:     {avg_throughput:.2f} ± {std_throughput:.2f} steps/second")
        envs.close()

    elif args.mode == "learn":
        print("TODO")
        # env = gym.make("AASEnv-v0")
        # try:
        #     # check_env(env) # Throws warning
        #     # check_env(env.unwrapped)
        #     sb3_check_env(env)
        #     print("\nEnvironment passes all checks!")
        # except Exception as e:
        #     print(f"\nEnvironment has issues: {e}")

        # env.reset()
        # env.step(env.action_space.sample())
        # env.reset()

        # # Instantiate the agent
        # model = PPO("MlpPolicy", env, verbose=1, device='cpu')

        # # Train the agent
        # print("Training agent...")
        # model.learn(total_timesteps=40000)
        # print("Training complete.")

        # # Save the agent
        # model_path = "ppo_agent.zip"
        # model.save(model_path)
        # print(f"Model saved to {model_path}")

        # # Load and test the trained agent
        # del model # remove to demonstrate loading
        # model = PPO.load(model_path, device='cpu')

        # print("\nTesting trained agent...")
        # obs, info = env.reset()
        # for _ in range(800): # Run for 800 steps
        #     action, _states = model.predict(obs, deterministic=True)
        #     obs, reward, terminated, truncated, info = env.step(action)
            
        #     if terminated or truncated:
        #         print("Episode finished. Resetting.")
        #         obs, info = env.reset()
        
        # env.close()

    else:
        print(f"Unknown mode: {args.mode}")

def configure_host_x11():
    if not shutil.which("xhost"):
        print("Error: 'xhost' command not found. GUI rendering may fail.")
        return
    try: # Check if already configured
        output = subprocess.check_output(["xhost"], text=True)
        if "LOCAL:" in output or "local:docker" in output:
            return # Already configured, return silently
    except subprocess.CalledProcessError:
        pass
    print("Granting X Server access to Docker containers...")
    try:
        subprocess.run(["xhost", "+local:docker"], check=True)
        print("X Server access granted.")
    except subprocess.CalledProcessError as e:
        print(f"Warning: Could not configure xhost: {e}")

if __name__ == '__main__':
    configure_host_x11()
    main()
