import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import json
from multiprocessing import Process


# Function to rotate the graph
def update_graph(num, ax, fig):
    ax.view_init(azim=num)
    return fig,


# Function to read the results from a JSON file
def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


# Function to set equal axes on the plot
def set_axes_equal(ax: plt.Axes):
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)


# Helper function to set equal axes on the plot
def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


# Function to generate a 3D plot
def plot_3d_process(points, obstacles, route, distance, animate, title, position):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i, point in enumerate(points):
        ax.scatter(point[0], point[1], point[2], c='black', marker='o', s=10)

    for obstacle in obstacles:
        center = obstacle[0]
        radius = obstacle[1]
        phi, theta = np.mgrid[0.0:2.0 * np.pi:100j, 0.0:np.pi:50j]
        x_sphere = center[0] + radius * np.sin(theta) * np.cos(phi)
        y_sphere = center[1] + radius * np.sin(theta) * np.sin(phi)
        z_sphere = center[2] + radius * np.cos(theta)
        ax.plot_surface(x_sphere, y_sphere, z_sphere, color='b', rstride=4, cstride=4, alpha=0.3)

    if len(route) != 0:
        route_points = [points[i] for i in route]
        route_x, route_y, route_z = zip(*route_points)
        ax.plot(route_x, route_y, route_z, linestyle='-', marker=',', c='black', linewidth=1)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Algorithm {title}')
    if distance != -1:
        ax.text2D(-0.35, -0.1, f'Tunnel length: {distance:.2f} m.', transform=ax.transAxes, fontsize=12)
    else:
        ax.text2D(-0.35, -0.1, f'No solution found!', transform=ax.transAxes, fontsize=12)

    ax.set_box_aspect([1, 1, 1])
    set_axes_equal(ax)

    mgr = plt.get_current_fig_manager()
    mgr.set_window_title(f'Algorithm {title}')
    mgr.window.geometry(position)
    if animate:
        anim = FuncAnimation(fig, update_graph, frames=np.arange(0, 360, 1), fargs=(ax, fig), interval=12, blit=False)
    else:
        anim = None

    plt.show()
    return anim


if __name__ == '__main__':
    rotation = True
    results_file_path = '../results.json'
    instance_file_path = '../instance.json'
    results_json = read_json_file(results_file_path)
    instance_json = read_json_file(instance_file_path)
    times = results_json['times']
    results = results_json['results']
    obstacles = instance_json['obstacles']
    points = instance_json['points']
    processes = []
    for x in range(4):
        match x:
            case 0:
                position = "+100+0"
                title = "A"
            case 1:
                position = "+740+0"
                title = "B"
            case 2:
                position = "+100+500"
                title = "C"
            case 3:
                position = "+740+500"
                title = "D"
            case _:
                position = "+0+0"
                title = "X"
        p = Process(target=plot_3d_process, args=(points, obstacles, results[x][0], results[x][1], rotation, title, position))
        processes.append(p)
        p.start()

    for p in processes:
        p.join()
