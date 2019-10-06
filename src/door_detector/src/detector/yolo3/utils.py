"""Miscellaneous utility functions."""

from functools import reduce

from PIL import Image
import numpy as np
import cv2


def compose(*funcs):
    """Compose arbitrarily many functions, evaluated left to right.

    Reference: https://mathieularose.com/function-composition-in-python/
    """
    # return lambda x: reduce(lambda v, f: f(v), funcs, x)
    if funcs:
        return reduce(lambda f, g: lambda *a, **kw: g(f(*a, **kw)), funcs)
    else:
        raise ValueError('Composition of empty sequence not supported.')


def letterbox_image(image, size):
    '''resize image with unchanged aspect ratio using padding'''
    iw, ih = image.size
    w, h = size
    scale = min(float(w) / iw, float(h))
    nw = int(iw*scale)
    nh = int(ih*scale)

    image = image.resize((nw,nh), Image.BICUBIC)
    new_image = Image.new('RGB', size, (128,128,128))
    new_image.paste(image, ((w-nw)//2, (h-nh)//2))
    return new_image


def rand(a=0, b=1):
    return np.random.rand()*(b-a) + a


def get_random_data(annotation_line, input_shape, random=True, max_boxes=20, jitter=.3, hue=.1, sat=1.5, val=1.5, proc_img=True):
    '''random preprocessing for real-time data augmentation'''
    line = annotation_line.split()

    # numpy array: BGR, 0-255
    image = cv2.imread(line[0])
    # height, width, channel
    ih, iw, _ = image.shape
    h, w = input_shape
    box = np.array([np.array(list(map(int,box.split(',')))) for box in line[1:]])

    if not random:
        # resize image
        scale = min(w/iw, h/ih)
        nw = int(iw*scale)
        nh = int(ih*scale)
        dx = (w-nw)//2
        dy = (h-nh)//2
        image_data=0
        if proc_img:
            # resize
            image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
            # convert into PIL Image object
            image = Image.fromarray(image[:, :, ::-1])
            new_image = Image.new('RGB', (w,h), (128,128,128))
            new_image.paste(image, (dx, dy))
            # convert into numpy array: RGB, 0-1
            image_data = np.array(new_image)/255.

        # correct boxes
        box_data = np.zeros((max_boxes,5))
        if len(box)>0:
            np.random.shuffle(box)
            if len(box)>max_boxes: box = box[:max_boxes]
            box[:, [0,2]] = box[:, [0,2]]*scale + dx
            box[:, [1,3]] = box[:, [1,3]]*scale + dy
            box_data[:len(box)] = box

        return image_data, box_data

    # resize image
    new_ar = w/h * rand(1-jitter,1+jitter)/rand(1-jitter,1+jitter)
    scale = rand(.25, 2)
    if new_ar < 1:
        nh = int(scale*h)
        nw = int(nh*new_ar)
    else:
        nw = int(scale*w)
        nh = int(nw/new_ar)

    # resize
    image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
    # convert into PIL Image object
    image = Image.fromarray(image[:, :, ::-1])

    # place image
    dx = int(rand(0, w-nw))
    dy = int(rand(0, h-nh))
    new_image = Image.new('RGB', (w,h), (128,128,128))
    new_image.paste(image, (dx, dy))
    # convert into numpy array: BGR, 0-255
    image = np.asarray(new_image)[:, :, ::-1]

    # rotation augment
    is_rot = False
    if is_rot:
        right = rand() < 0.5
        if right:
            image = image.transpose(1, 0, 2)[:, ::-1]
        else:
            image = image.transpose(1, 0, 2)[::-1]

    # convert into numpy array: RGB, 0-1
    image_data = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.0

    # correct boxes
    box_data = np.zeros((max_boxes,5))
    if len(box)>0:
        np.random.shuffle(box)
        box[:, [0,2]] = box[:, [0,2]]*nw/iw + dx
        box[:, [1,3]] = box[:, [1,3]]*nh/ih + dy
        if is_rot:
            if right:
                tmp = box[:, [0, 2]]
                box[:, [0,2]] = h - box[:, [3,1]]
                box[:, [1,3]] = tmp
            else:
                tmp = box[:, [2, 0]]
                box[:, [0,2]] = box[:, [1,3]]
                box[:, [1,3]] = w - tmp

        box[:, 0:2][box[:, 0:2]<0] = 0
        box[:, 2][box[:, 2]>w] = w
        box[:, 3][box[:, 3]>h] = h
        box_w = box[:, 2] - box[:, 0]
        box_h = box[:, 3] - box[:, 1]
        box = box[np.logical_and(box_w>1, box_h>1)] # discard invalid box
        if len(box)>max_boxes: box = box[:max_boxes]
        box_data[:len(box)] = box

    return image_data, box_data
