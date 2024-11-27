import sys
import os
from pathlib import Path

current_folder_path = os.getcwd()
sys.path.append(
    current_folder_path+"/../../../devel/lib/python3/dist-packages/spiral_planner/srv")

sys.path.append(
    current_folder_path+"/tool")
