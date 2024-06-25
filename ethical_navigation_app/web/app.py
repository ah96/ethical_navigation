from flask import Flask, render_template, request, jsonify
from path_planning import PathPlanner
import numpy as np

app = Flask(__name__)

# Initial map with some obstacles (0 = free space, 1 = obstacle)
grid = np.zeros((20, 20))
grid[5:15, 10] = 1  # Example vertical obstacle
grid[10, 5:15] = 1  # Example horizontal obstacle

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get_map', methods=['GET'])
def get_map():
    return jsonify(grid.tolist())

@app.route('/start_planning', methods=['POST'])
def start_planning():
    data = request.json
    algorithm = data['algorithm']
    start = tuple(map(int, data['start'].split(',')))
    goal = tuple(map(int, data['goal'].split(',')))

    planner = PathPlanner(grid)  # Initialize the planner with the current grid

    if algorithm == 'a_star':
        path = planner.a_star(start, goal)
    elif algorithm == 'dijkstra':
        path = planner.dijkstra(start, goal)
    elif algorithm == 'rrt':
        path = planner.rrt(start, goal)
    elif algorithm == 'utilitarian':
        path = planner.utilitarian_path(start, goal)
    elif algorithm == 'deontological':
        path = planner.deontological_path(start, goal)
    elif algorithm == 'virtue_ethics':
        path = planner.virtue_ethics_path(start, goal)
    else:
        return jsonify({'status': 'error', 'message': 'Unknown algorithm'}), 400

    return jsonify({'status': 'success', 'path': path})

@app.route('/update_map', methods=['POST'])
def update_map():
    global grid
    grid_data = request.json['grid']
    grid = np.array(grid_data)
    return jsonify({'status': 'success'})

if __name__ == '__main__':
    app.run(debug=True)
