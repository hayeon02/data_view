import sys
from PySide6.QtWidgets import QApplication, QWidget
from PySide6.QtGui import QPainter, QColor
from PySide6.QtCore import QRect

class MyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Draw Rectangle")
        self.setGeometry(100, 100, 300, 300)

    def paintEvent(self, event):
        painter = QPainter(self)

        # 사각형의 위치와 크기 정의
        rect = QRect(50, 50, 100, 150)

        # 사각형의 색상 설정
        painter.setBrush(QColor(0, 0, 0))

        # 사각형 그리기
        painter.drawRect(rect)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MyWidget()
    widget.show()
    sys.exit(app.exec())
