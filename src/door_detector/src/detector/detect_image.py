import argparse
from yolo import YOLO
import cv2
from PIL import Image as PIL_Image
import numpy as np
from datetime import datetime
import os

args = {
    "model_path": 'model/weights.h5',
    "anchors_path": 'model/anchors.txt',
    "classes_path": 'model/classes.txt',
    "score": 0.8,
    "iou": 0.45,
    "model_image_size": (608, 608),
    "gpu_num": 1,
}


def read_image(path):

    print('Reading image from {}'.format(path))
    cv2_img = cv2.imread(path)

    height, width, _ = cv2_img.shape
    print('image has width of {} and height of {}'.format(width, height))
    min_side_length = min(width, height)
    start_point_for_cropping = int((width - min_side_length) / 2)

    print('Crop image at {} for length {}'.format(start_point_for_cropping, min_side_length))

    cv2_img = cv2_img[:, start_point_for_cropping: (start_point_for_cropping + min_side_length)]
    cv2_img = cv2.resize(cv2_img, (608, 608), interpolation=cv2.INTER_AREA)

    return cv2_img


def convert_to_pil(cv2_img):
    pil_image = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
    return PIL_Image.fromarray(pil_image)


def main(path):
    output_path, _ = os.path.splitext(path)

    path = os.path.abspath(path)

    cv2_img = read_image(path)
    pil_image = convert_to_pil(cv2_img)

    yolo = YOLO(**args)
    r_image, out_boxes, out_scores, out_classes = yolo.detect_image(pil_image)
    yolo.close_session()

    cv2_image = cv2.cvtColor(np.array(r_image), cv2.COLOR_RGB2BGR)
    hash = str(datetime.now().strftime("%Y%m%d-%H%M%S"))
    cv2.imwrite(output_path + 'detection-' + hash + '.png', cv2_image)

    with open(output_path + 'detection-' + hash + '.txt', "w+") as file:
        file.write('bounding boxes: {}\n'.format(out_boxes))
        file.write('scores: {}\n'.format(out_scores))
        file.write('classes: {}\n'.format(out_classes))

    cv2.imshow('image', cv2_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        return


if __name__ == '__main__':
    # class YOLO defines the default value, so suppress any default here
    parser = argparse.ArgumentParser(argument_default=argparse.SUPPRESS)
    '''
    Command line options
    '''
    parser.add_argument(
        '--path', type=str,
        help='path to image'
    )

    FLAGS = parser.parse_args()

    main(FLAGS.path)
