import os
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

import numpy as np
import cv2
import torch
from torch.backends import cudnn
from torchvision import transforms
from face_detection import RetinaFace
import matplotlib
from PIL import Image
matplotlib.use('TkAgg')

from model import SixDRepNet
import utils


transformations = transforms.Compose([transforms.Resize(224),
                                      transforms.CenterCrop(224),
                                      transforms.ToTensor(),
                                      transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

cudnn.enabled = True
snapshot_path = "./6DRepNet_300W_LP_AFLW2000.pth"
model = SixDRepNet(backbone_name='RepVGG-B1g2',
                   backbone_file='',
                   deploy=True,
                   pretrained=False)

detector = RetinaFace(gpu_id=0)

saved_state_dict = torch.load(os.path.join(snapshot_path), map_location='cpu')
model.load_state_dict(saved_state_dict)
model.to("cuda")
model.eval()

cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
size = (width, height)
out = cv2.VideoWriter('./recordings/out.avi', fourcc, fps, size)

if not cap.isOpened():
    raise IOError("Cannot open webcam")

with torch.inference_mode():
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        faces = detector(frame)

        for box, landmarks, score in faces:
            if score < .95:
                continue
            x_min = int(box[0])
            y_min = int(box[1])
            x_max = int(box[2])
            y_max = int(box[3])
            bbox_width = abs(x_max - x_min)
            bbox_height = abs(y_max - y_min)

            x_min = max(0, x_min-int(0.2*bbox_height))
            y_min = max(0, y_min-int(0.2*bbox_width))
            x_max = x_max+int(0.2*bbox_height)
            y_max = y_max+int(0.2*bbox_width)

            img = frame[y_min:y_max, x_min:x_max]
            img = Image.fromarray(img)
            img = img.convert('RGB')
            img = transformations(img)

            img = torch.Tensor(img[None, :]).to("cuda")

            R_pred = model(img)

            euler = utils.compute_euler_angles_from_rotation_matrices(
                R_pred)*180/np.pi
            p_pred_deg = euler[:, 0].cpu()
            y_pred_deg = euler[:, 1].cpu()
            r_pred_deg = euler[:, 2].cpu()

            utils.determine_orientation_region(frame, y_pred_deg, p_pred_deg, r_pred_deg)
            utils.draw_axis(frame, y_pred_deg, p_pred_deg, r_pred_deg)
            utils.plot_pose_cube(frame, y_pred_deg, p_pred_deg, r_pred_deg, x_min + int(.5*(
                x_max-x_min)), y_min + int(.5*(y_max-y_min)), size=bbox_width)

        out.write(frame)
        cv2.imshow("Demo", frame)
        key = cv2.waitKey(1)
        if key % 256 == 27:
            break
    out.release()
    cap.release()
    cv2.destroyAllWindows()
