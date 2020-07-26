import cv2

def draw_box(image,x_min,y_min,color,class_name = None,object_id = None):
  cv2.rectangle(img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), color, 2)
  if object_id == None: object_id = ''
  if class_name == None: class_name = ''
  cv2.rectangle(img, (int(x_min), int(y_min-30)), (int(bbox[0])+(len(class_name)+len(str(object_id)))*17, int(y_min)), color, -1)
  return image