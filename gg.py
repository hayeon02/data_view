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
        self.highlighted_point = None

    def setup_ui(self):
        self.setGeometry(200, 200, 900, 700)  # Width reduced to 900

        # Layout 설정
        layout = QVBoxLayout()

        btn1 = QPushButton("  파일 불러오기  ", self)
        btn1.clicked.connect(self.btn_fun_FileLoad)
        layout.addWidget(btn1)

        # QSplitter를 사용하여 세로로 분할
        self.splitter = QSplitter(self)

        self.table = QTableView(self)
        self.model = QStandardItemModel(self)
        self.model.setHorizontalHeaderLabels(["utm_x", "utm_y", "heading", "add"])  # 열 헤더 설정
        self.table.setModel(self.model)  # 테이블에 모델 설정

        # 행 클릭 시 시각화 업데이트
        self.table.selectionModel().selectionChanged.connect(self.highlight_selected_row)

        self.splitter.addWidget(self.table)

        # Matplotlib Figure와 Canvas 설정 (표 크기에 맞추어 축소)
        self.figure = plt.figure(figsize=(4, 2))  # 그래프 크기 조정
        self.canvas = FigureCanvas(self.figure)

        # Size policies to reduce extra space
        self.canvas.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.splitter.addWidget(self.canvas)

        # QSplitter 비율 설정 - 표가 더 많은 공간을 차지하도록 조정
        self.splitter.setStretchFactor(0, 5)  # 테이블이 5배 공간 차지
        self.splitter.setStretchFactor(1, 1)  # 그래프는 최소 공간 차지

        layout.addWidget(self.splitter)

        # 버튼을 그래프 아래에 배치
        btn2 = QPushButton("  멈춤  ", self)
        btn2.clicked.connect(self.add_stop)
        layout.addWidget(btn2)

        btn3 = QPushButton("  오르막  ", self)
        btn3.clicked.connect(self.add_uphill)
        layout.addWidget(btn3)

        btn4 = QPushButton("  삭제  ", self)
        btn4.clicked.connect(self.delete_add)
        layout.addWidget(btn4)

        btn5 = QPushButton("  저장  ", self)
        btn5.clicked.connect(self.save_data)
        layout.addWidget(btn5)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    # 파일 불러오기
    def btn_fun_FileLoad(self):
        file, _ = QFileDialog.getOpenFileName(self, "File Load", 'D:/ubuntu/', 'Text File(*.txt);; All Files (*)')
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

    # 멈춤 버튼 기능
    def add_stop(self):
        row = self.table.selectionModel().currentIndex().row()  # 현재 선택된 행
        if row >= 0:  # 유효한 행인 경우
            self.model.setItem(row, 3, QStandardItem("0"))  # 선택된 행의 'add' 열에 0 추가

    # 오르막 버튼 기능
    def add_uphill(self):
        row = self.table.selectionModel().currentIndex().row()  # 현재 선택된 행
        if row >= 0:  # 유효한 행인 경우
            self.model.setItem(row, 3, QStandardItem("1"))  # 선택된 행의 'add' 열에 1 추가

    # 삭제 버튼 기능
    def delete_add(self):
        row = self.table.selectionModel().currentIndex().row()  # 현재 선택된 행
        if row >= 0:  # 유효한 행인 경우
            self.model.setItem(row, 3, QStandardItem(""))  # 선택된 행의 'add' 열 삭제

    # 저장 버튼 기능
    def save_data(self):
        file_name, _ = QFileDialog.getSaveFileName(self, "Save File", '', "Text Files (*.txt);;All Files (*)")
        if file_name:
            with open(file_name, 'w') as f:
                for row in range(self.model.rowCount()):
                    utm_x = self.model.item(row, 0).text()
                    utm_y = self.model.item(row, 1).text()
                    heading = self.model.item(row, 2).text()
                    add = self.model.item(row, 3).text()
                    f.write(f"{utm_x} {utm_y} {heading} {add}\n")  # 데이터 저장

            # 저장 후 애플리케이션 종료
            self.close()  # 현재 창 닫기

    # 행 선택 시 해당 점을 빨간색으로 하이라이트
    def highlight_selected_row(self):
        selected_row = self.table.selectionModel().currentIndex().row()
        if selected_row >= 0:
            selected_x = float(self.model.item(selected_row, 0).text())
            selected_y = float(self.model.item(selected_row, 1).text())

            self.figure.clear()
            ax = self.figure.add_subplot(111)

            # 전체 경로 그리기
            ax.plot(self.utm_x, self.utm_y, marker='o', linestyle='-', color='blue', label='UTM Path')

            # 선택된 점을 빨간색으로 표시
            ax.plot(selected_x, selected_y, marker='o', color='red', markersize=10, label='Selected Point')

            ax.set_title("UTM Coordinates")
            ax.set_xlabel("UTM X")
            ax.set_ylabel("UTM Y")
            ax.grid()
            ax.axis('equal')
            ax.legend()

            self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mywindow = MyWindow()
    mywindow.show()
    app.exec_()
