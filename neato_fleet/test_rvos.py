#!/usr/bin/env python

import rvo2
import matplotlib.pyplot as plt
import numpy as np

sim = rvo2.PyRVOSimulator(1/60., 1.5, 5, 1.5, 2, 0.4, 2)

# Agents in a plus shape - each needs to switch to opposite position
a0 = sim.addAgent((2, 0))   # Right -> Left
a1 = sim.addAgent((0, 2))   # Top -> Bottom
a2 = sim.addAgent((-2, 0))  # Left -> Right
a3 = sim.addAgent((0, -2))  # Bottom -> Top

# Set preferred velocities to move to opposite positions
sim.setAgentPrefVelocity(a0, (-1, 0))  # Move left
sim.setAgentPrefVelocity(a1, (0, -1))  # Move down
sim.setAgentPrefVelocity(a2, (1, 0))   # Move right
sim.setAgentPrefVelocity(a3, (0, 1))   # Move up

print('Simulation has %i agents in it.' % sim.getNumAgents())
print('Running simulation - agents switching positions in plus formation')

# Store trajectories
agents = [a0, a1, a2, a3]
trajectories = {agent: [] for agent in agents}

for step in range(200):
    sim.doStep()

    # Record positions
    for agent in agents:
        pos = sim.getAgentPosition(agent)
        trajectories[agent].append(pos)

    if step < 20:  # Print first 20 steps
        positions = ['(%5.3f, %5.3f)' % sim.getAgentPosition(agent_no)
                     for agent_no in agents]
        print('step=%2i  t=%.3f  %s' % (step, sim.getGlobalTime(), '  '.join(positions)))

# Create visualization
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Plot 1: Trajectories with arrows
colors = ['red', 'blue', 'green', 'orange']
labels = ['Agent 0 (Right→Left)', 'Agent 1 (Top→Bottom)',
          'Agent 2 (Left→Right)', 'Agent 3 (Bottom→Top)']

for agent, color, label in zip(agents, colors, labels):
    traj = np.array(trajectories[agent])
    ax1.plot(traj[:, 0], traj[:, 1], '-', color=color, alpha=0.6, linewidth=2, label=label)
    ax1.scatter(traj[0, 0], traj[0, 1], color=color, s=150, marker='o',
                edgecolors='black', linewidths=2, zorder=5)
    ax1.scatter(traj[-1, 0], traj[-1, 1], color=color, s=150, marker='s',
                edgecolors='black', linewidths=2, zorder=5)

    # Add arrows to show direction
    for i in range(0, len(traj)-1, 20):
        dx = traj[i+1, 0] - traj[i, 0]
        dy = traj[i+1, 1] - traj[i, 1]
        ax1.arrow(traj[i, 0], traj[i, 1], dx, dy, head_width=0.15,
                 head_length=0.1, fc=color, ec=color, alpha=0.5)

# Add center crosshair
ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.3, linewidth=1)
ax1.axvline(x=0, color='gray', linestyle='--', alpha=0.3, linewidth=1)
ax1.scatter(0, 0, color='black', s=50, marker='x', zorder=10)

ax1.set_xlabel('X Position', fontsize=12)
ax1.set_ylabel('Y Position', fontsize=12)
ax1.set_title('Plus Formation - Agents Switching Positions\n(circles=start, squares=end)',
              fontsize=14, fontweight='bold')
ax1.legend(loc='best', fontsize=9)
ax1.grid(True, alpha=0.3)
ax1.axis('equal')

# Plot 2: X and Y positions over time
time_steps = np.arange(len(trajectories[a0])) * (1/60.)

for i, (agent, color, label) in enumerate(zip(agents, colors, labels)):
    traj = np.array(trajectories[agent])
    ax2.plot(time_steps, traj[:, 0], color=color, linewidth=2,
             linestyle='-', label=f'{label} (X)', alpha=0.7)
    ax2.plot(time_steps, traj[:, 1], color=color, linewidth=2,
             linestyle='--', label=f'{label} (Y)', alpha=0.7)

ax2.axhline(y=0, color='gray', linestyle=':', alpha=0.3, linewidth=1)
ax2.set_xlabel('Time (seconds)', fontsize=12)
ax2.set_ylabel('Position', fontsize=12)
ax2.set_title('Position Components Over Time\n(solid=X, dashed=Y)',
              fontsize=14, fontweight='bold')
ax2.legend(loc='best', fontsize=7, ncol=2)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
