# Encode and decode message
import json
import base64

# Encode and decode image
import cv2
from helpers import *
import numpy as np

# For making requests
import requests

def send_dummy_detection_image_test():

    img = cv2.imread('mock_detection.png')
    img = np.array(img)

    message = {'detectionImage': encode_image(img, 'png')}

    request = requests.post('http://localhost:5000/detection/', json=message)
    print(request.text)


def receive_dummy_detection_image_test():
    request = requests.get('http://localhost:5000/detection/')

    image_base64 = request.json()['detectionImage']

    img, _ = decode_image(image_base64, False)

    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def send_dummy_map_image_test():

    img = cv2.imread('mock_map.png')
    img = np.array(img)

    message = {'mapImage': encode_image(img, 'png')}

    request = requests.post('http://localhost:5000/map/', json=message)
    print(request.text)


def receive_dummy_map_image_test():
    request = requests.get('http://localhost:5000/map/')

    image_base64 = request.json()['mapImage']

    img, _ = decode_image(image_base64, False)

    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def send_dummy_door_list_test():
    
    doors_list = [{
          'doorId': 1,
          'status': 'closed'
        },
        {
          'doorId': 2,
          'status': 'closed'
        },
        {
          'doorId': 3,
          'status': 'open'
        }]

    message = {'doorList': doors_list}

    request = requests.post('http://localhost:5000/doors/list/', json=message)
    print(request.text)
    


if __name__ == '__main__':
    send_dummy_detection_image_test()
    # receive_dummy_detection_image_test()
    send_dummy_map_image_test()
    # receive_dummy_map_image_test()
    send_dummy_door_list_test()