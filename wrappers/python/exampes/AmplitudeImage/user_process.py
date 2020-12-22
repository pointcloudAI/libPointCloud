import main_window
import pyqtgraph as pg
import numpy as np
from PyQt5.QtCore import QTimer
import ShowDepthNoGUI
import PointCloud
import pyqtgraph.exporters
import os


class user_operation(main_window.Ui_MainWindow):
    def __init__(self, window):
        super().setupUi(window)

        self.look_timer = QTimer()
        self.look_timer.timeout.connect(self.user_look_timer_timeout_handler)
        self.look_timer.setTimerType(0)
        self.look_timer.start(20)

        # self.btn_save_image.clicked.connect(self.user_btn_save_image_click_handler)

        self.cameraSystem = None
        self.tt = 0

        pg.setConfigOptions(antialias=True)
        self.graphicsWidget = pg.GraphicsLayoutWidget()
        p1 = self.graphicsWidget.addPlot(title="image")

        # Item for displaying image data
        self.image = pg.ImageItem()

        p1.addItem(self.image)


        # Contrast/color control
        self.hist = pg.HistogramLUTItem()
        self.hist.setImageItem(self.image)
        self.graphicsWidget.addItem(self.hist)
        self.hist.setHistogramRange(0, 100)
        # Generate image data
        # data = np.random.random(size=(480, 640))
        # for i in range(0, 480):
        #     for j in range(0, 640):
        #         data[i][j] = 200
        # self.image.setImage(data)

        self.grid_graph.addWidget(self.graphicsWidget)

        # camera init
        self.cameraSystem = PointCloud.CameraSystem()

        self.devices = self.cameraSystem.scan()

        if len(self.devices) == 1:
            print(" Find one device.")
            self.user_camera_window = ShowDepthNoGUI.MainWindow(self.cameraSystem, self.devices)
            # key = input("Input enter key to quit.")
            # print(" Quit now.")
            # window.stop()
        else:
            print(" No device found.")

            print(" before del  cameraSystem.")
            del self.cameraSystem
            print(" after del  cameraSystem.")
            cameraSystem = None

    def user_btn_save_image_click_handler(self):
        file_save_dir_path = r'D:\Tfo_3-5_32'  # 想要保存在哪个文件夹路径下
        file_save_name = 'depth_data.xlsx'  # 保存的文件名字
        self.user_camera_window.user_write_raw_data_ok_flag(1)
        self.user_camera_window.user_write_depth_data_ok_flag(1)
        color_file_save_path = os.path.join(file_save_dir_path, file_save_name)  # 最终颜色文件存放的名字

        
        self.user_camera_window.user_write_raw_data_ok_flag(0)
        self.user_camera_window.user_write_depth_data_ok_flag(0)

    def user_look_timer_timeout_handler(self):

        # raw data
        if (self.user_camera_window.user_get_raw_data_ok_flag() == 1) and \
                (self.user_camera_window.user_get_depth_data_ok_flag() == 1):
            mat = np.rot90(ShowDepthNoGUI.user_image, -1)   # 图像数据选择，深度不需要，因为图像数据在这里从0，0开始显示
            self.image.setImage(mat)
            
            b, h = self.image.getHistogram(bins = 100)
            h = (h - 10).clip(0)
            i = np.nonzero(h)

            lower = b[i[0][0]]
            upper = b[i[0][-1]]
            self.levels = [100.0, 1048.0]
            self.levels[0] = float(lower)
            self.levels[1] = float(upper)
            self.image.setLevels(self.levels)

            ex = pyqtgraph.exporters.ImageExporter(self.graphicsWidget.scene())
            ex.export(fileName="one.jpg")

            # depth

            #depth and raw data clear
            self.user_camera_window.user_write_raw_data_ok_flag(0)
            self.user_camera_window.user_write_raw_data_ok_flag(0)
        else:
            pass

