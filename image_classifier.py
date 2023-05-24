import cv2 as cv
from tflite_support.task import vision, core, processor
from tflite_support import metadata
# image classifier
# Load the TFLite model and allocate tensors.


class ImageClassifier:

    def __init__(self, model_path, threshold, number_of_threads):
        base_options = core.BaseOptions(
            file_name=model_path, num_threads=number_of_threads)
        classification_options = processor.ClassificationOptions(
            max_results=1, score_threshold=threshold)
        options = vision.ImageClassifierOptions(
            base_options=base_options, classification_options=classification_options)
        self.classifier = vision.ImageClassifier.create_from_options(options)

    def classify(self, image):
        rgb_image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        tensor_image = vision.TensorImage.create_from_array(rgb_image)
        categories = self.classifier.classify(
            tensor_image).classifications[0].categories

        if len(categories) > 0:
            category = categories[0]

            return category.category_name + " " + str(category.score)
        else:
            return None


if __name__ == "__main__":
    try:
        cap = cv.VideoCapture(0)
        classifier = ImageClassifier("can_and_bottle_model.tflite", 0.6, 1)
        while True:
            success, image = cap.read()
            if success:
                print(classifier.classify(image))
            else:
                break
    except Exception as e:
        print(e)
    finally:
        if cap:
            cap.release()
        cv.destroyAllWindows()
        print("Done")
        exit(0)
