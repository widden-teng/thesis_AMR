import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
import time
import numpy as np
from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions
cap = cv2.VideoCapture(4)
if not cap.isOpened():
  print("Error: Could not open webcam.")
  exit(1)

MARGIN=10
FONT_SIZE=1
HANDEDNESS_TEXT_COLOR=(88,205,54)
FONT_THICKNESS=1
model_path='./weight/hand_landmarker.task'


def draw_landmarks_on_image(rgb_image, detection_result,fps):
  hand_landmarks_list = detection_result.hand_landmarks
  handedness_list = detection_result.handedness
  annotated_image = np.copy(rgb_image)
  for idx in range(len(hand_landmarks_list)):
    hand_landmarks = hand_landmarks_list[idx]
    handedness = handedness_list[idx]
    
    # Draw the hand landmarks.
    hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    hand_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
    ])
    solutions.drawing_utils.draw_landmarks(
      annotated_image,
      hand_landmarks_proto,
      solutions.hands.HAND_CONNECTIONS,
      solutions.drawing_styles.get_default_hand_landmarks_style(),
      solutions.drawing_styles.get_default_hand_connections_style())

    # Get the top left corner of the detected hand's bounding box.
    height, width, _ = annotated_image.shape
    x_coordinates = [landmark.x for landmark in hand_landmarks]
    y_coordinates = [landmark.y for landmark in hand_landmarks]
    text_x = int(min(x_coordinates) * width)
    text_y = int(min(y_coordinates) * height) - MARGIN

    # Draw handedness (left or right hand) on the image.
    cv2.putText(annotated_image, f"{handedness[0].category_name}",
                (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
    cv2.putText(annotated_image,"FPS:"+str(fps).split(".")[0],(10,30),cv2.FONT_HERSHEY_SIMPLEX,1.0,(255,0,0),2,cv2.LINE_AA)
    return annotated_image

base_options=mp.tasks.BaseOptions(model_asset_path=model_path,
delegate=mp.tasks.BaseOptions.Delegate.GPU)
options = vision.HandLandmarkerOptions(base_options=base_options,
running_mode=mp.tasks.vision.RunningMode.VIDEO,
num_hands=2)
detector = vision.HandLandmarker.create_from_options(options)
while True:
  try:
    ret,frame=cap.read()
  except KeyboardInterrupt:
    break
  if not ret :
   break
  # cv2.imwrite("a.jpg",frame)
  
  # image = mp.Image.create_from_file("a.jpg")
  mp_image=mp.Image(image_format=mp.ImageFormat.SRGB,data=np.array(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)))
  timestap=cap.get(cv2.CAP_PROP_POS_MSEC)
  # STEP 4: Detect hand landmarks from the input image.
  start_time=time.time()
  try:
    detection_result = detector.detect_for_video(mp_image,int(timestap))

  #   print(fps)
  except KeyboardInterrupt:
    break
  end_time=time.time()
  fps=1/(end_time-start_time)

  # print(detection_result)

  # STEP 5: Process the classification result. In this case, visualize it.
  annotated_image = draw_landmarks_on_image(mp_image.numpy_view(), detection_result,fps)

  try:
    img_rgb = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR) 
    cv2.imshow("frame",img_rgb)
  except:
    print("no image!!!")
  key=cv2.waitKey(1)
  if key==27:
      exit
cv2.destroyAllWindows()