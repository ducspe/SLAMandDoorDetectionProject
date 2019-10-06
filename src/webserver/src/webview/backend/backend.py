# REST API
from flask import Flask  # For REST API suppprt
from flask import request
from flask import jsonify
from flask import render_template
from flask import make_response
from flask_cors import CORS  # To call the API with cross-origins
from .helpers import *
import os

### Settings
dev = False
app = Flask(__name__, template_folder='static')
CORS(app)


class Report:

    def __init__(self):
        self.map_image = None
        self.annotated_map_image = None
        self.robot_location = None
        self.door_list = None
        self.detection_image = None

    def process_report(self, message):
        data = message.json

        if 'mapImage' in data:
            self.map_image = data['mapImage']
        if 'annotatedMapImage' in data:
            self.annotated_map_image = data['annotatedMapImage']
        if 'robotLocation' in data:
            self.robot_location = data['robotLocation']
        if 'doorList' in data:
            self.door_list = data['doorList']
        if 'detectionImage' in data:
            self.detection_image = data['detectionImage']

report = Report()


### Error handling
@app.errorhandler(404)
def not_found(error):
    return make_response(jsonify({'error': 'Not found'}))


### Frontend
@app.route('/')
def render_static():
    return render_template('index.html')


### Routes
@app.route('/report/', methods=['POST', 'GET'])
def set_map():
    if request.method == 'POST':
        if not request.json:
            return jsonify({'error': 'Not found'}), 404
        report.process_report(request)
        return jsonify({'message': 'Report received'}), 201

    else:
        return jsonify({
            'mapImage': report.map_image,
            'annotatedMapImage': report.annotated_map_image,
            'detectionImage': report.detection_image,
            'doorList': report.door_list
        }), 201


@app.route('/map/', methods=['GET'])
def get_map():
    if report.map_image is None:
        return jsonify({'message': 'No map image received (yet)'}), 404

    return jsonify({'mapImage': report.map_image}), 201


@app.route('/map/annotated/', methods=['GET'])
def get_annotated_map():
    if report.annotated_map_image is None:
        return jsonify({'message': 'No map image received (yet)'}), 404

    return jsonify({'annotatedMapImage': report.annotated_map_image}), 201


@app.route('/detection/', methods=['GET'])
def get_detection():
    if report.detection_image is None:
        return jsonify({'message': 'No detection image received (yet)'}), 404

    return jsonify({'detectionImage': report.detection_image}), 201


@app.route('/doors/list/', methods=['GET'])
def get_doors_list():
    if report.door_list is None:
        return jsonify({'message': 'No list of doors received (yet)'}), 404

    return jsonify({'doorList': report.door_list}), 201


@app.route('/shutdown/', methods=['GET'])
def stop():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    raise SystemExit()


def start():
    app.run(debug=True, port=5000, host='0.0.0.0')

if __name__ == '__main__':
    start()
