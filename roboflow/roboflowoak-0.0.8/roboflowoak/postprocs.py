import numpy as np

def non_max_suppression_fast(boxes, overlapThresh):
    if len(boxes) == 0:
        return []

    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")
    pick = []
    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]
    conf = boxes[:,4]

    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(conf)

    while len(idxs) > 0:

        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)

        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)
        overlap = (w * h) / area[idxs[:last]]
        idxs = np.delete(idxs, np.concatenate(([last],
            np.where(overlap > overlapThresh)[0])))

    return boxes[pick].astype("float")


def w_np_non_max_suppression(np_prediction, num_classes, conf_thres=0.5, nms_thres=0.4):
    np_box_corner = np.zeros(np_prediction.shape)
    np_box_corner[:, :, 0] = np_prediction[:, :, 0] - np_prediction[:, :, 2] / 2
    np_box_corner[:, :, 1] = np_prediction[:, :, 1] - np_prediction[:, :, 3] / 2
    np_box_corner[:, :, 2] = np_prediction[:, :, 0] + np_prediction[:, :, 2] / 2
    np_box_corner[:, :, 3] = np_prediction[:, :, 1] + np_prediction[:, :, 3] / 2

    np_prediction[:, :, :4] = np_box_corner[:, :, :4]
    batch_predictions = []
    for np_image_i, np_image_pred in enumerate(np_prediction):
        filtered_predictions = []
        np_conf_mask = (np_image_pred[:, 4] >= conf_thres).squeeze()

        np_image_pred = np_image_pred[np_conf_mask]
        if np_image_pred.shape[0] == 0:
            continue
        np_class_conf = np.max(np_image_pred[:, 5:5 + num_classes], 1)
        np_class_pred = np.argmax(np_image_pred[:, 5:5 + num_classes], 1)
        np_class_conf = np.expand_dims(np_class_conf, axis=1)
        np_class_pred = np.expand_dims(np_class_pred, axis=1)
        np_detections = np.append(np.append(np_image_pred[:, :5], np_class_conf, axis =1), np_class_pred, axis = 1)

        np_unique_labels = np.unique(np_detections[:, -1])

        for c in np_unique_labels:
            np_detections_class = np_detections[np_detections[:, -1] == c]
            np_detections_class = sorted(np_detections_class, key=lambda row: row[4], reverse=True)
            np_max_detections = []

            filtered_predictions.extend(non_max_suppression_fast(np.array(np_detections_class), nms_thres))
        batch_predictions.append(filtered_predictions)
    return batch_predictions

def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    if ratio_pad is None:
        gain = max(img1_shape) / max(img0_shape)
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    print(gain)
    print(pad)

    coords[0] -= pad[0]
    coords[2] -= pad[0]
    coords[1] -= pad[1]
    coords[3] -= pad[1]

    coords = [coord / gain for coord in coords]

    coords[0] = int(round(np.clip(coords[0], a_min = 0, a_max=img0_shape[1])))
    coords[2] = int(round(np.clip(coords[2], a_min = 0, a_max=img0_shape[1])))
    coords[1] = int(round(np.clip(coords[1], a_min = 0, a_max=img0_shape[0])))
    coords[3] = int(round(np.clip(coords[3], a_min = 0, a_max=img0_shape[0])))

    return coords

def process_detections(detections, input_dims, class_filter=[], class_names=[]):
    w, h = input_dims

    processed_detections = []
    for i, detection in enumerate(detections):

        x1, y1, x2, y2 = int(detection[0]) , int(detection[1]) , int(detection[2]) , int(detection[3])

        coords = scale_coords((input_dims[0],input_dims[1]), [x1,y1,x2,y2], (h,w))
        x1 = coords[0]
        y1 = coords[1]
        x2 = coords[2]
        y2 = coords[3]

        conf = detection[4]

        label_num = detection[-1]

        label = class_names[int(label_num)]
        if label in class_filter:
            processed_detection = [x1,y1,x2,y2,label,conf]
            processed_detections.append(processed_detection)
    return processed_detections
