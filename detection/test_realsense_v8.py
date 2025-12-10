from ultralytics import YOLO
from ultralytics.yolo.v8.segment.predict import SegmentationPredictor
import cv2
import torch
import numpy as np 
import pyrealsense2 as rs

object_coordinates = []
cv_font = cv2.FONT_HERSHEY_SIMPLEX

config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

pipeline = rs.pipeline()
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

# Initialize video writer
video_writer = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (1280, 720))

# model = torch.hub.load('C:\Users\User\Documents\Ayan doc\coding\measure_object_distance\ultralytics\ultralytics', 'custom','"C:\Users\User\Documents\Ayan doc\coding\measure_object_distance\ultralytics\ultralytics\last_capsv8s_seg.pt"', source='local')
model= YOLO("last_capsv8s_seg.pt")
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
# cap = cv2.VideoCapture(0)
# intrinsics = cv2.VideoCapture.get_intrinsic_matrix(cap)
while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    img = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    # img = np.asanyarray(frame)
    # im0=img.copy()
    output = model(source=img)
    # detection_img = output.render()[0]
    # detection_img = output.render()[0]
    for result in output:
         if result is not None:
              im_res=result.plot()
              data = result.boxes.data.cpu().tolist()
              h, w = result.orig_shape #if normalize else (1, 1)
              for i, row in enumerate(data):
                box = {'x1': row[0] / w, 'y1': row[1] / h, 'x2': row[2] / w, 'y2': row[3] / h}
                x = int((row[0] + row[2])/2)
                y = int((row[1] + row[3])/2)
                dist = depth_frame.get_distance(x + 4, y + 8)*1000
                Xtarget = (dist*(x+4 - intr.ppx)/intr.fx - 35) #the distance from RGB camera to realsense center
                Ytarget = (dist*(y+8 - intr.ppy)/intr.fy)
                Ztarget = dist
                conf = row[4]
                id = int(row[5])
                name = result.names[id]
                coordinate_text = "(" + str(round(Xtarget)) + "," + str(round(Ytarget)) + "," + str(round(Ztarget)) + ")"
                confidence_text = f"{conf:.2f}"
                # Display coordinates
                cv2.putText(im_res, text=coordinate_text, org=(int((row[0] + row[2])/2), int((row[1] + row[3])/2)),
                fontFace = cv_font, fontScale = 0.7, color=(255,255,255), thickness=1, lineType = cv2.LINE_AA)

                if Ztarget > 0 and conf >= 0.89 and name == 'capsicum':
                    print(f"Detected {name} at {coordinate_text} with confidence: {confidence_text}")
                    cv2.imwrite('target_file.jpg', im_res)
                    pipeline.stop()
                    video_writer.release()
                    cv2.destroyAllWindows()
                    raise SystemExit

                # cv2.imshow('color image', im_res)
                video_writer.write(im_res)
                print(f"Detected {name} at {coordinate_text} with confidence: {confidence_text}")
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
pipeline.stop()
video_writer.release()
cv2.destroyAllWindows()
