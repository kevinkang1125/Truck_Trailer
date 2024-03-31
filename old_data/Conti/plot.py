import matplotlib.pyplot as plt
import numpy as np

reward_TD3 = np.loadtxt('./reward_TD3.txt')
reward_ppo = np.loadtxt('./reward_of_ppo.txt')
# aloss = np.loadtxt('./algo/DDPG/data/actor_loss_ddpg.txt')
# closs = np.loadtxt('./algo/DDPG/data/critic_loss_ddpg.txt')

episode1 = len(reward_TD3)
episode2 = len(reward_ppo)
x1 = range(episode1)
x2 = range(episode2)
ave = 10

ave_reward_TD3 = np.zeros(episode1)
ave_reward_ppo = np.zeros(episode2)
# ave_aloss = np.zeros(episode)
# ave_closs = np.zeros(episode)

for i in range(episode1):
    ave_reward_TD3[i] = np.mean(reward_TD3[max(0, i - ave):(i + 1)])
    # ave_aloss[i] = np.mean(aloss[max(0, i - ave):(i + 1)])
    # ave_closs[i] = np.mean(closs[max(0, i - ave):(i + 1)])
for m in range(episode2):
    ave_reward_ppo[m] = np.mean(reward_ppo[max(0, m - ave):(m + 1)])


# plt.figure(1)
plt.plot(x1, ave_reward_TD3, color='dodgerblue',label='TD3')
plt.plot(x1, reward_TD3, alpha=0.2, color='dodgerblue')
plt.plot(x2, ave_reward_ppo, color='darkorange',label='PPO')
plt.plot(x2, reward_ppo, alpha=0.2, color='darkorange')
plt.title('Reward')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.legend(loc = 'upper right')
plt.savefig('compare.png')

# plt.figure(2)
# plt.subplot(1,2,1)
# plt.plot(x, ave_aloss, color='black')
# plt.plot(x, aloss, alpha=0.2, color='black')
# plt.title('Actor loss')
# plt.xlabel('Episode')
# plt.subplot(1,2,2)
# plt.plot(x, ave_closs, color='black')
# plt.plot(x, closs, alpha=0.2, color='black')
# plt.title('Critic loss')
# plt.xlabel('Episode')

plt.show()

