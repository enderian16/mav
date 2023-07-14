from PyQt5 import QtWidgets

from controller import MainWindow

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
#mavproxy.exe --master=tcp:127.0.0.1:5762 --out=udp:127.0.0.1:14550