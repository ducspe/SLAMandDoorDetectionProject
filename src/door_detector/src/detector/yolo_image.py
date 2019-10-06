from .yolo import YOLO


def detect_img(image):

    args = {
        "model_path": 'model/weights.h5',
        "anchors_path": 'model/anchors.txt',
        "classes_path": 'model/classes.txt',
        "score": 0.3,
        "iou": 0.45,
        "model_image_size": (416, 416),
        "gpu_num" : 1,
    }

    r_image, out_boxes, out_scores, out_classes = [image, None, None, None]

    yolo = YOLO(args)

    while True:
        r_image, out_boxes, out_scores, out_classes = yolo.detect_image(image)
    yolo.close_session()

    return r_image, out_boxes, out_scores, out_classes
