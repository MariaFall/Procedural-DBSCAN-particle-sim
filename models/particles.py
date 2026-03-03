import numpy as np

class cluster:
    def __init__(self, cluster_color=(255, 0, 0, 200), p_list=None):
        self.cluster_color = cluster_color
        self.p_list = p_list

class particle:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
        self.cluster_id = -1
        self.visited = False
