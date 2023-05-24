import cv2 as cv

cam = cv.VideoCapture(0)
i = 0
while True:
    success, image = cam.read()
    key = cv.waitKey(1)
    if success:
        cv.imshow("image", image)
                  
    if key == ord("q"):
        print("Closing camera")
        break
    elif success and key == ord("c"):
        print("Capturing image")
        cv.imwrite(f"./empty/empty{i}.png", image)
        i+=1

cam.release()
cv.destroyAllWindows()
    
    