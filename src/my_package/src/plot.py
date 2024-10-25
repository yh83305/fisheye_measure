import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def read_values(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    data = {
        'original_axes': {},
        'direct': np.array([0, 0, 0]),
        'rotated_axes': {},
        'rotated_direct': np.array([0, 0, 0]),
        'adjusted_axes': {},
        'adjusted_direct': np.array([0, 0, 0]),
    }

    current_key = None
    for line in lines:
        if 'Original Axes' in line:
            current_key = 'original_axes'
        elif 'Direct Vector' in line:
            current_key = 'direct'
        elif 'Rotated Axes' in line:
            current_key = 'rotated_axes'
        elif 'Rotated Direct' in line:
            current_key = 'rotated_direct'
        elif 'Adjusted Axes' in line:
            current_key = 'adjusted_axes'
        elif 'Adjusted Direct' in line:
            current_key = 'adjusted_direct'
        elif ':' in line:
            if current_key in ['original_axes', 'rotated_axes', 'adjusted_axes']:
                axis, values = line.strip().split(': ')
                data[current_key][axis] = np.fromstring(values, sep=' ')
            elif current_key == 'direct':
                data[current_key] = np.fromstring(line.strip(), sep=' ')
            elif current_key == 'rotated_direct' or current_key == 'adjusted_direct':
                data[current_key] = np.fromstring(line.strip(), sep=' ')

    return data


def plot_axes(data):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Original Axes
    for key, vector in data['original_axes'].items():
        ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='r', label=f'Original {key}')

    # Rotated Axes
    for key, vector in data['rotated_axes'].items():
        ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='g', label=f'Rotated {key}')

    # Adjusted Axes
    for key, vector in data['adjusted_axes'].items():
        ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='b', label=f'Adjusted {key}')

    ax.quiver(0, 0, 0, data['direct'][0], data['direct'][1], data['direct'][2], color='k', label='Direct Vector')
    ax.quiver(0, 0, 0, data['rotated_direct'][0], data['rotated_direct'][1], data['rotated_direct'][2], color='c',
              label='Rotated Direct')
    ax.quiver(0, 0, 0, data['adjusted_direct'][0], data['adjusted_direct'][1], data['adjusted_direct'][2], color='m',
              label='Adjusted Direct')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


filename = 'values.txt'  # 替换为你保存的文件名
data = read_values(filename)
plot_axes(data)
