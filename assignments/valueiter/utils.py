import io
import base64
import matplotlib.pyplot as plt
from assignments.valueiter.gridworld import Action


def plotResults(world, results, show=True):
    """ Plot the grid world and the value iteration results """
    
    fig = plt.figure(figsize=(14,6))
    axs = fig.subplots(1,2)
    axs[0].imshow(world.getWorldImage())
    axs[0].axis('off')
    axs[0].set_title('Map')
    axs[1].imshow(results['values'], cmap='grey')
    axs[1].axis('off')
    axs[1].set_title('Value')
    for row in range(results['values'].shape[0]):
        for col in range(results['values'].shape[1]):
            act = results['policy'][row,col]
            if hasattr(act, 'value'):
                act = act.value
            if act == Action.UP.value:
                dx, dy =  0.0, -0.5
            elif act == Action.DOWN.value:
                dx, dy =  0.0,  0.5
            elif act == Action.LEFT.value:
                dx, dy = -0.5,  0.0
            elif act == Action.RIGHT.value:
                dx, dy =  0.5,  0.0
            else:
                dx, dy = 0.0, 0.0
            axs[1].annotate("", xytext=(col, row), xy=(col+dx, row+dy), arrowprops=dict(arrowstyle="->"))
            if act == Action.UP.value:
                axs[1].text(col, row+0.1, "{:.1f}".format(results['values'][row][col]), color='blue', fontsize=6)
            else:
                axs[1].text(col, row-0.1, "{:.1f}".format(results['values'][row][col]), color='blue', fontsize=6)
    plt.tight_layout()

    if show:
        plt.show()
    else:
        buf = io.BytesIO()
        plt.savefig(buf, format="jpg")
        buf.seek(0)
        plt.close()
        b64_string = base64.b64encode(buf.read()).decode("utf-8")
        return "data:image/jpg;base64,{}".format(b64_string)
