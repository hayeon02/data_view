import sys
from fileinput import filename

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import pyqtgraph as pg
from functools import partial
from PyQt5.QtCore import QEvent, Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.utm_x = []
        self.utm_y = []
        self.lastClicked = []

        self.setupUI()
        self.setup()

    def setupUI(self):
        self.setGeometry(100, 100, 1500, 900)

        #파일 열기 버튼
        self.FileOpenBtn = QPushButton("파일 열기", self)
        self.FileOpenBtn.clicked.connect(self.FileOpen)

    #파일 불러오기
    def FileOpen(self):
        file, _ = QFileDialog.getOpenFileName(self, "File Loader", "D:/ubuntu/", 'csv File(*.csv);; Text File(*.txt);;  All Files(*)')

        if file:
            self.LoadData(file)
        else:
            print("파일 불러오기 실패")

    #파일(표형식)
    def DataTable(self):
        self.table = QTableView(self)
        self.table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel) #수평 스크롤 활성화
        self.table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents) #테이블 크기 조정 설정
        self.model = QStandardItemModel(self) #데이터 모델 설정
        self.splitter = QSplitter(self) #splitter:하나의 박스로 묶어서 크기 조절 가능하도록 하는 기능
        self.model.setHorizontalHeaderLabels(["x", "y", "heading", "add"]) #열 헤더 설정
        self.table.setModel(self.model)  # table에 model을 씌우는 형식
        self.table.horizontalHeader().setStretchLastSection(False) #마지막 열이 자동으로 늘어나지 않도록 설정
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.MultiSelection)  # 다중 선택 가능 모드
        self.table.viewport().installEventFilter(self)  # 이벤트 필터

        self.splitter.addWidget(self.table)

    def eventFilter(self, source, event):
        if event.type() == QEvent.MouseButtonPress and source is self.table.viewport():
            modifiers = QApplication.keyboardModifiers()
            if not (modifiers == Qt.ControlModifier or modifiers == Qt.ShiftModifier):
                self.table.clearSelection()
        return super(MainWindow, self).eventFilter(source, event)

    # 파일(그래프)
    def graph(self):
        self.graphData.clear()
        self.scatter = pg.ScatterPlotItem(self.utm_x, self.utm_y, symbol='o', symbolSize=10)
        self.scatter.sigClicked.connect(self.clicked)
        self.graphData.addItem(self.scatter)

    #포인트 클릭시 변화
    def clicked(self, plot, points):
        clickedPen = pg.mkPen('r', width=2)
        for p in self.lastClicked:
            p.resetPen()
        self.lastClicked = points

        for p in points:
            pos = p.pos()
            x, y = pos.x(), pos.y()
            if x in self.utm_x and y in self.utm_y:
                index = (self.utm_x.index(x)) + 1
                self.point(index)

            p.setPen(clickedPen)

    #클릭한 포인트 번호
    def point(self, index):
        self.point_label.setText(f"{index}")  # QLabel 업데이트
        self.point_label.setFont(QFont('Arial', 10))

    #add 작성 버튼 + 저장 버튼
    def btns(self):
        btn_table = QTableWidget(self)
        btn_table.setColumnCount(1)

        btn_table_column = ["버튼"]
        btn_table.setHorizontalHeaderLabels(btn_table_column)

        btn_table.setRowCount(12)

        btn_labels = ["전진", "후진", "일시 정지", "오르막 일시 정지", "신호등 감지 구간", "가속 구간", "끝 지점", "차선 무시 구간", "정밀 전진", "정밀 후진", "감속 구간", "저장"]

        for i, label in enumerate(btn_labels):
            btn = QPushButton(label)
            btn.clicked.connect(partial(self.button_clicked, i))  # 버튼 번호 전달
            btn_table.setCellWidget(i, 0, btn)

        return btn_table

    def button_clicked(self, index):
        if index <= 10:
            row = self.table.selectionModel().selectedRows()
            for row_index in row:
                row = row_index.row()
                self.model.setItem(row, 3, QStandardItem(str(index)))
        elif index == 11:
            file_name, _ = QFileDialog.getSaveFileName(self, "Save File", '', "csv Files (*.csv);;All Files (*)")
            if file_name:
                with open(file_name, 'w') as f:
                    for row in range(self.model.rowCount()):
                        utm_x = self.model.item(row, 0).text()
                        utm_y = self.model.item(row, 1).text()
                        heading = self.model.item(row, 2).text()
                        add = self.model.item(row, 3).text()
                        f.write(f"{utm_x} {utm_y} {heading} {add}\n")

                self.close()


    #파일 데이터 불러오기
    def LoadData(self, filename):
        self.model.clear() #테이블 데이터 삭제
        self.model.setHorizontalHeaderLabels(["x", "y", "heading", "add"]) #불러왔을 때의 헤더
        self.utm_x.clear() #좌표 데이터 초기화
        self.utm_y.clear()

        try:
            with open(filename, 'r') as f:
                for line in f:
                    utm_x, utm_y, heading = line.strip().split()
                    self.model.appendRow([
                        QStandardItem(utm_x),
                        QStandardItem(utm_y),
                        QStandardItem(heading),
                        QStandardItem(""),
                    ])
                    self.utm_x.append(float(utm_x))
                    self.utm_y.append(float(utm_y))

            self.table.resizeColumnsToContents() #열 크기 자동 조정
            self.graph()
        except Exception as e:
            print(f"파일을 읽는 중 오류 발생: {e}")

    #위치
    def setup(self):
        file_layout = QVBoxLayout()
        file_layout.addWidget(self.FileOpenBtn)

        self.DataTable()
        file_layout.addWidget(self.splitter)

        file_widget = QWidget()
        file_widget.setLayout(file_layout)

        self.point_label = QLabel(" ", self)
        btn_table = self.btns()
        etc_layout = QVBoxLayout()
        etc_layout.addWidget(self.point_label)
        etc_layout.addWidget(btn_table)

        data_layout = QHBoxLayout()
        data_layout.addWidget(file_widget)
        data_layout.addLayout(etc_layout)

        self.graphData = pg.PlotWidget()
        data_layout.addWidget(self.graphData)

        central_widget = QWidget()
        central_widget.setLayout(data_layout)
        self.setCentralWidget(central_widget)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = MainWindow()
    main.show()
    app.exec_()