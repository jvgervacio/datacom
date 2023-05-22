import mediapipe as mp
import cv2
import datetime as dt 
from PIL import Image

BaseOptions = mp.tasks.BaseOptions
ImageClassifierResult = mp.tasks.vision.ImageClassifierResult
ImageClassifier = mp.tasks.vision.ImageClassifier
ImageClassifierOptions = mp.tasks.vision.ImageClassifierOptions
VisionRunningMode = mp.tasks.vision.RunningMode

def print_result(result: ImageClassifierResult, output_image: mp.Image, timestamp_ms: int):
     top_category = result.classifications[0].categories[0]
     print(f"{top_category.category_name} ({top_category.score:.2f})")
     
options = ImageClassifierOptions(
    base_options=BaseOptions(model_asset_path='efficientnet_lite0.tflite'),
    running_mode=VisionRunningMode.LIVE_STREAM,
    max_results=5,
    result_callback=print_result)

cap = cv2.VideoCapture(0)
with ImageClassifier.create_from_options(options) as classifier:
    while True:
        success, image = cap.read()
        if success:
            cv2.imshow('MediaPipe ImageClassifier', image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # bgr to rgb
            
            image = mp.Image(mp.ImageFormat.SRGB, image)
            # get image timestamp
            
            classification_result = classifier.classify_async(image, int(dt.datetime.now().timestamp() * 1000))
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
cap.release()
cv2.destroyAllWindows()