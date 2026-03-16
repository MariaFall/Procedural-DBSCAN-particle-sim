import numpy as np

class cluster:
    def __init__(self, cluster_color=(255, 0, 0, 200), p_list=None):
        self.cluster_color = cluster_color
        self.p_list = p_list if p_list is not None else []
        self.center_id = -1
        self.diameter = 0

class particle:
    def __init__(self, x=0.0, y=0.0, vx = 0.0, vy = 0.0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.cluster_id = -1
        self.particle_id = -1
        self.is_center = False
        self.visited = False
        self.center_dist = 0

# class outside_particle(particle):
#     def __init__(self, x= 0.0, y = 0.0):
#         pass