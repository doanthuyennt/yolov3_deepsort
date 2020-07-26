import sys
import os
sys.path.insert(0, "/home/mmlab/git_workspace/yolov3_deepsort/moduletracker")
from deepsort.load_deepsort_tracker import load_deepsort_tracker as DeepSortTracker
from deepsort.load_deepsort_tracker import load_encoder as Encoder
def load_tracker():
    return DeepSortTracker(),Encoder()

