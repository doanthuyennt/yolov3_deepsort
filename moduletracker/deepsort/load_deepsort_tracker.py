from . import preprocessing
from . import nn_matching
from .tracker import Tracker
from tools import generate_detections as gdet
from configs.config_utils.conf import Conf
import sys
import os

DEEPSORT_CONFIG = Conf("/home/mmlab/git_workspace/yolov3_deepsort/moduletracker/deepsort/model_config/deep_sort.config")
COSINE_CONFIG = Conf("/home/mmlab/git_workspace/yolov3_deepsort/moduletracker/deepsort/metric_config/cosine_distance.config")
def load_deepsort_tracker():
  # Definition of the parameters
  max_cosine_distance = COSINE_CONFIG["max_cosine_distance"]
  nn_budget = COSINE_CONFIG["nn_budget"]
  #initialize deep sort
  metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
  deep_sort_tracker = Tracker(metric)  
  return deep_sort_tracker
def load_encoder():
  model_filename = DEEPSORT_CONFIG["model_path"]
  encoder = gdet.create_box_encoder(model_filename, batch_size=1)
  return encoder