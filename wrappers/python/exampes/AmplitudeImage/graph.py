from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import sys
import traceback

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
import user_process

if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = user_process.user_operation(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

