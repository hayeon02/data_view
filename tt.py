import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setup_ui()
        self.utm_x = []
        self.utm_y = []

    def setup_ui(self):
        self.setGeometry(200, 200, 1200, 700)

        btn1 = QPushButton("  파일 불러오기  ", self)
        btn1.clicked.connect(self.btn_fun_FileLoad)

        # Layout 설정
        layout = QVBoxLayout()
        layout.addWidget(btn1)

        # QSplitter를 사용하여 세로로 분할
        self.splitter = QSplitter(self)

        self.table = QTableView(self)
        self.model = QStandardItemModel(self)
        self.model.setHorizontalHeaderLabels(["utm_x", "utm_y", "heading", "add"])  # 열 헤더 설정
        self.table.setModel(self.model)  # 테이블에 모델 설정

        self.splitter.addWidget(self.table)

        # Matplotlib Figure와 Canvas 설정
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.splitter.addWidget(self.canvas)

        layout.addWidget(self.splitter)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    # 파일 불러오기
    def btn_fun_FileLoad(self):
        file, _ = QFileDialog.getOpenFileName(self, "File Load", 'D:/ubuntu/', 'Text File(*.txt);; All Files(*)')
        if file:
            self.load_data(file)
        else:
            print("파일 불러오기에서 문제가 발생하였습니다.")

    # 불러온 파일을 표 형식으로 시각화
    def load_data(self, filename):
        self.model.clear()  # 이전 데이터를 지우고 새로운 데이터를 추가
        self.model.setHorizontalHeaderLabels(["utm_x", "utm_y", "heading", "add"])  # 헤더 재설정
        self.utm_x.clear()
        self.utm_y.clear()

        try:
            with open(filename, 'r') as f:
                for line in f:
                    utm_x, utm_y, heading = line.strip().split()  # 공백으로 분리
                    self.model.appendRow([
                        QStandardItem(utm_x),
                        QStandardItem(utm_y),
                        QStandardItem(heading),
                        QStandardItem(""),
                    ])
                    self.utm_x.append(float(utm_x))  # UTM X 값 저장
                    self.utm_y.append(float(utm_y))  # UTM Y 값 저장

            self.table.resizeColumnsToContents()  # 열 크기 자동 조정
            self.visualize_data()  # 데이터가 로드되면 시각화 자동 실행
        except Exception as e:
            print(f"파일을 읽는 중 오류 발생: {e}")

    # 데이터 시각화 (꺾은선 그래프)
    def visualize_data(self):
        self.figure.clear()  # 이전 그래프 지우기
        ax = self.figure.add_subplot(111)

        # 꺾은선 그래프 그리기
        ax.plot(self.utm_x, self.utm_y, marker='o', linestyle='-', color='blue', label='UTM Path')
        ax.set_title("UTM Coordinates")
        ax.set_xlabel("UTM X")
        ax.set_ylabel("UTM Y")
        ax.grid()
        ax.axis('equal')  # 비율 유지
        ax.legend()  # 범례 추가
        self.canvas.draw()  # 그래프 업데이트


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mywindow = MyWindow()
    mywindow.show()
    app.exec_()
