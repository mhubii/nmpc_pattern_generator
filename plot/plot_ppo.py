import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

# get data
data = np.genfromtxt("../build/bin/example_ppo.csv", delimiter=",")

def animate():

    # plot everything
    #epochs = np.array([1,5,10,15,20])
    epochs = np.array([1])

    for e in epochs:

        fig, ax = plt.subplots()

        # setup all plots
        ax.plot(0, 0, 'x', c='black', label='Spawn')                      # spawn of the agent

        # adding a circle around the goal that indicates maximum distance to goal before the environment gets reset
        obstacle = plt.Circle((data[0,3], data[0,4]), data[0,5], linestyle='-', color='black', fill=True, label='Obstacle')
        ax.add_patch(obstacle)
        goal = plt.Circle((data[0,6], data[0,7]), data[0,5], linestyle='-', color='r', fill=True, label='Goal')
        ax.add_patch(goal)
        won_circle = plt.Circle((data[0,6], data[0,7]), 0.3, linestyle='--', color='r', fill=False)
        ax.add_patch(won_circle)
        lost_circle = plt.Circle((data[0,6], data[0,7]), 5.0, linestyle='--', color='gray', fill=False, label='Maximum Goal Distance')
        ax.add_patch(lost_circle)

        agent, = ax.plot(data[0,1], data[0,2], 'o', c='b', label='Agent') # agent
        agent_line, = ax.plot(data[0,1], data[0,2], '-', c='b')           # small tail following the agent

        # plot settings
        ax.set_xlabel('x / a.u.')
        ax.set_ylabel('y / a.u.')
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)
        ax.set_title("Agent in Test Environment")
        ax.legend()
        title = ax.text(0.15,0.85, "", bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
                    transform=ax.transAxes, ha="center")

        epoch_data = data[np.where(data[:,0]==e)]
        # tail for the agent
        tail = 0

        def animate(i):
            agent.set_data(epoch_data[i,1], epoch_data[i,2])
            global tail
            if (epoch_data[i,8] == 4): # STATUS enum in main.cpp, 4 = RESETTING
                tail = 0
            agent_line.set_data(epoch_data[i-tail:i,1], epoch_data[i-tail:i,2])
            if (tail <= 50):
                tail += 1
            obstacle.center = (epoch_data[i,3], epoch_data[i,4])
            goal.center = (epoch_data[i,6], epoch_data[i,7])
            won_circle.center = (epoch_data[i,6], epoch_data[i,7])
            lost_circle.center = (epoch_data[i,6], epoch_data[i,7])
            #title.set_text('Epoch {:1.0f}'.format(epoch_data[i,0]))
            return agent, agent_line, obstacle, goal, won_circle, lost_circle

        ani = animation.FuncAnimation(fig, animate, blit=True, interval=5, frames=2000)#epoch_data.shape[0])
        #plt.show()
        ani.save('img/epoch_{}.gif'.format(e), writer='imagemagick', fps=100)

def plot():

    # plot everything
    #epochs = np.array([1,5,10,15,20])
    epochs = np.array([1, 10])

    for e in epochs:

        fig, ax = plt.subplots()

        epoch_data = data[np.where(data[:,0]==e)]

        # setup all plots
        ax.plot(0, 0, 'x', c='black', label='Spawn')                      # spawn of the agent

        # adding a circle around the goal that indicates maximum distance to goal before the environment gets reset
        obstacle = plt.Circle((epoch_data[0,3], epoch_data[0,4]), epoch_data[0,5], linestyle='-', color='black', fill=True, label='Obstacle')
        ax.add_patch(obstacle)
        goal = plt.Circle((epoch_data[0,6], epoch_data[0,7]), epoch_data[0,5], linestyle='-', color='r', fill=True, label='Goal')
        ax.add_patch(goal)
        circle = plt.Circle((epoch_data[0,6], epoch_data[0,7]), 2.5, linestyle='--', color='gray', fill=False, label='Maximum Goal Distance')
        ax.add_patch(circle)

        agent, = ax.plot(epoch_data[:-1,1], epoch_data[:-1,2], 'o', c='b', label='Agent') # agent
        agent_line, = ax.plot(epoch_data[:-1,1], epoch_data[:-1,2], '-', c='b')           # small tail following the agent

        # plot settings
        ax.set_xlabel('x / a.u.')
        ax.set_ylabel('y / a.u.')
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-2.5, 2.5)
        ax.set_title("Agent in Test Environment")
        ax.legend()
        title = ax.text(0.15,0.85, "", bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
                        transform=ax.transAxes, ha="center")

        #plt.show()
        plt.savefig('img/epoch_{}.pdf'.format(e))


if __name__ == "__main__":
    
    animate()
    #plot()
