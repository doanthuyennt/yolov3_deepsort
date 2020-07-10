import cv2
def cropping_coors(image):
  fx = fy = 0.5
  image = cv2.resize(image,(0,0),fx = fx , fy = fy)
  (x,y,w,h) = cv2.selectROI("ROI select", image)
  cropped = image[y:y+h , x:x+w]
  cv2.imshow("img",cropped)
  cv2.waitKey(0)
  cv2.destroyAllWindows()
  return (int(x/fx),int(y/fy),int(w/fx),int(h/fy))
  
if __name__=="__main__":
    image = cv2.imread("test.jpg")
    cropping(image)