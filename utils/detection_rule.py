import cv2
def cropping_coors(image):
  print(cv2.__file__)
  fx = fy = 0.5
  image = cv2.resize(image,(0,0),fx = fx , fy = fy)
  (x,y,w,h) = cv2.selectROI(image,False,False)
  print(x,y,w,h)
  cropped = image[y:y+h, x:x+w ,:]
  return (int(x/fx),int(y/fy),int(w/fx),int(h/fy))

class DetectionRule:
	number_of_rules = 0
	def __init__(self,tl,br):
		self.tl = tl
		self.br = br
	def check(self,p,):
		if p[0] < self.tl[0] or p[0] > self.br[0]:
			return False
		if p[1] < self.tl[1] or p[1] > self.br[1]:
			return False
		return True
	def draw_bound(self,img):
		return img

    