# Encode and decode message
import json
import base64
import cv2
import numpy as np
import io

def decode_image(img_base64, saveimg=False, name='received'):
    # Clean up the data, save image format
    img_export = img_base64.split(';base64,')
    img_format = img_export[0].split('data:image/')[1]
    img_base64 = img_export[1]
    img_base64 = base64.b64decode(img_base64)

    # img_format = re.search(r'data:image/(.*);',img_export.group(0)).group(1)

    # Open base64 as image file
    file_bytes = np.asarray(bytearray(img_base64), dtype=np.uint8)
    img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    # save image
    if saveimg:
        cv2.imwrite(name + '.png', img)

    # Convert read image file to numpy array
    img = np.array(img)

    return img, img_format


def encode_image(img, img_format, saveimg=False, name='encoded_img'):
    # Save array as image file
    if saveimg:
        cv2.imwrite(name + '.png', img)

    # Fix JPEG => JPG file ending
    if img_format == 'jpeg':
        img_format == 'jpg'

    # Encode array as file
    retval, img_output_64 = cv2.imencode('.' + img_format, img)

    # Encode file as base64
    img_data64 = base64.b64encode(img_output_64).decode('utf-8')

    # Format base64 string for HTML frontend
    img_base64 = 'data:image/' + img_format + ';base64,' + img_data64

    return img_base64


def make_response(message):
    # Return as JSON
    return json.dumps(message)