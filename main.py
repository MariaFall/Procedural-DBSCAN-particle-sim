import sys
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import ctypes
import os
import time

lib_ext = '.so'
lib_path = os.path.abspath(f"calc{lib_ext}")
c_calc = ctypes.CDLL(lib_path)

c_calc.run_dbscan.argtypes = [
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.c_int,
    ctypes.c_double,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_int
]
c_calc.run_dbscan.restype = None

c_calc.update_physics.argtypes = [
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.c_int,
    ctypes.c_double
]
c_calc.update_physics.restype = None

c_calc.get_cluster_lines.argtypes = [
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_int),
    ctypes.c_int,
    ctypes.c_double,
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int)
]
c_calc.get_cluster_lines.restype = None


class MovingPointsApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.num_points = 80
        self.radius = 0.35
        self.eps = 2.8
        self.minPts = 3
        self.cluster_limit = 7

        self.paused = False
        self.pause_start_time = 0
        
        self.main_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QtWidgets.QHBoxLayout(self.main_widget)
        
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setXRange(-10, 10)
        self.graph_widget.setYRange(-10, 10)
        self.main_layout.addWidget(self.graph_widget, stretch=4)
        
        self.control_layout = QtWidgets.QVBoxLayout()
        self.main_layout.addLayout(self.control_layout, stretch=1)
        
        self.control_layout.addWidget(QtWidgets.QLabel("EPS distance:"))
        self.eps_spin = QtWidgets.QDoubleSpinBox()
        self.eps_spin.setRange(0.1, 10.0)
        self.eps_spin.setValue(self.eps)
        self.eps_spin.setSingleStep(0.1)
        self.eps_spin.valueChanged.connect(self.update_params)
        self.control_layout.addWidget(self.eps_spin)
        
        self.control_layout.addWidget(QtWidgets.QLabel("MinPts:"))
        self.minpts_spin = QtWidgets.QSpinBox()
        self.minpts_spin.setRange(1, 100)
        self.minpts_spin.setValue(self.minPts)
        self.minpts_spin.valueChanged.connect(self.update_params)
        self.control_layout.addWidget(self.minpts_spin)
        
        self.control_layout.addWidget(QtWidgets.QLabel("Max Cluster Limit:"))
        self.limit_spin = QtWidgets.QSpinBox()
        self.limit_spin.setRange(1, 100)
        self.limit_spin.setValue(self.cluster_limit)
        self.limit_spin.valueChanged.connect(self.update_params)
        self.control_layout.addWidget(self.limit_spin)
        
        self.control_layout.addSpacing(20)
        
        self.control_layout.addWidget(QtWidgets.QLabel("Num Points:"))
        self.points_spin = QtWidgets.QSpinBox()
        self.points_spin.setRange(10, 5000)
        self.points_spin.setValue(self.num_points)
        self.points_spin.setSingleStep(50)
        self.control_layout.addWidget(self.points_spin)
        
        self.apply_btn = QtWidgets.QPushButton("Reset to Apply Changes")
        self.apply_btn.clicked.connect(self.apply_points)
        self.control_layout.addWidget(self.apply_btn)
        
        self.control_layout.addStretch()

        self.scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen(None))
        self.graph_widget.addItem(self.scatter)
        self.cluster_lines = {}
        
        self.init_arrays()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(20)
        
    def init_arrays(self):
        self.x = np.random.uniform(-9, 9, self.num_points).astype(np.float64)
        self.y = np.random.uniform(-9, 9, self.num_points).astype(np.float64)
        self.cluster_ids = np.zeros(self.num_points, dtype=np.int32)
        
        angles = np.random.uniform(0, 2 * np.pi, self.num_points)
        speeds = np.random.uniform(0.05, 0.15, self.num_points)
        self.vx = (np.cos(angles) * speeds).astype(np.float64)
        self.vy = (np.sin(angles) * speeds).astype(np.float64)
        
        max_lines = (self.num_points * (self.num_points - 1)) // 2
        self.out_x = np.zeros(max_lines * 2, dtype=np.float64)
        self.out_y = np.zeros(max_lines * 2, dtype=np.float64)
        self.out_cids = np.zeros(max_lines, dtype=np.int32)
        self.total_lines = ctypes.c_int(0)

        self.colors = [(100, 100, 100, 200)] + [
            (np.random.randint(50, 255), np.random.randint(50, 255), np.random.randint(50, 255), 200)
            for _ in range(self.num_points)
        ]
        self.brushes = [pg.mkBrush(c) for c in self.colors]
        
    def update_params(self):
        self.eps = self.eps_spin.value()
        self.minPts = self.minpts_spin.value()
        self.cluster_limit = self.limit_spin.value()

    def apply_points(self):
        self.num_points = self.points_spin.value()
        self.init_arrays()
        for line_item in self.cluster_lines.values():
            self.graph_widget.removeItem(line_item)
        self.cluster_lines.clear()

    def update_data(self):
        if self.paused:
            if time.time() - self.pause_start_time >= 0.1:
                self.paused = False
            else:
                return

        c_calc.update_physics(
            self.x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.vx.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.vy.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.num_points,
            self.radius
        )

        c_calc.run_dbscan(
            self.x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.num_points,
            self.eps,
            self.minPts,
            self.cluster_ids.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            self.cluster_limit - 1
        )
        
        c_calc.get_cluster_lines(
            self.x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.cluster_ids.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            self.num_points,
            self.eps,
            self.out_x.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.out_y.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            self.out_cids.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            ctypes.byref(self.total_lines)
        )

        unique_clusters = np.unique(self.cluster_ids)
        cluster_count = np.sum(unique_clusters > 0)

        current_brushes = [
            self.brushes[cid % len(self.brushes)] if cid > 0 else self.brushes[0]
            for cid in self.cluster_ids
        ]
        self.scatter.setData(x=self.x, y=self.y, brush=current_brushes)

        for line_item in self.cluster_lines.values():
            line_item.setData(x=[], y=[])
            
        count = self.total_lines.value
        if count > 0:
            valid_cids = self.out_cids[:count]
            valid_x = self.out_x[:count * 2]
            valid_y = self.out_y[:count * 2]
            
            for cid in unique_clusters:
                if cid == 0:
                    continue
                    
                mask = valid_cids == cid
                if not np.any(mask):
                    continue
                    
                mask2 = np.repeat(mask, 2)
                
                if cid not in self.cluster_lines:
                    pen = pg.mkPen(self.colors[cid % len(self.colors)], width=1.5)
                    self.cluster_lines[cid] = self.graph_widget.plot(pen=pen, connect='pairs')
                
                self.cluster_lines[cid].setData(x=valid_x[mask2], y=valid_y[mask2])

        if cluster_count >= self.cluster_limit:
            self.paused = True
            self.pause_start_time = time.time()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_win = MovingPointsApp()
    main_win.show()
    sys.exit(app.exec())
