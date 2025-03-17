from flask import Flask, jsonify, render_template
import json
import os

app = Flask(__name__)

@app.route('/')
def index():
    """Serve the main HTML page with visualization."""
    return render_template('index.html')

@app.route('/position')
def get_position():
    """Read and return the latest TurtleBot position from JSON file."""
    try:
        with open("turtlebot_position.json", "r") as f:
            position = json.load(f)
    except FileNotFoundError:
        position = {"x": 0.0, "y": 0.0, "theta": 0.0}
    
    return jsonify(position)

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000, debug=True)
