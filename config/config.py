import yaml
import numpy as np

with open("config/config.yaml", "r") as f:
    config = yaml.safe_load(f)

H=np.array(config["homography"])

NIR_WIDTH = config["nir"]["width"]
NIR_HEIGHT = config["nir"]["height"]