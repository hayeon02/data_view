import sys
import csv
import rospy
import utm
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import pyqtgraph as pg
from functools import partial
from PyQt5.QtCore import *
from sensor_msgs.msg import NavSatFix

class MainWindow2(QMainWindow):
    gps_signal = pyqtSignal(float, float)
    # pyqtSignal 신호: 특정 이벤트가 발생했음을 다른 객체에 알려주는 역할
    def __init__(self):
        super().__init__()
        self.utm_x = []
        self.utm_y = []
        self.colors = []
        self.lastClicked = []
        self.current_point = None
        self.first_click_index = None

        self.latitude_utm = 0.0
        self.longitude_utm = 0.0

        self.setupUI()
        self.setup()

        self.gps_signal.connect(self.update_location)

    def setupUI(self):
        self.setGeometry(100, 100, 1400, 900)

        # 파일 열기 버튼
        self.FileOpenBtn = QPushButton("파일 열기", self)
        self.FileOpenBtn.clicked.connect(self.FileOpen)
        self.FileOpenBtn.setFixedWidth(600)

    # 파일 불러오기
    def FileOpen(self):
        file, _ = QFileDialog.getOpenFileName(self, "File Loader", "D:/ubuntu/", 'csv File(*.csv);; Text File(*.txt);; All Files(*)')

        if file:
            self.LoadData(file)
        else:
            print("파일 불러오기 실패")

    # 파일(표형식)
    def DataTable(self):
        self.table = QTableView(self)
        self.table.setFixedWidth(600)
        self.table.setFixedHeight(900)

        self.table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)  # 테이블 크기 조정 설정
        self.model = QStandardItemModel(self)  # 데이터 모델 설정
        self.splitter = QSplitter(self)  # splitter: 하나의 박스로 묶어서 크기 조절 가능하도록 하는 기능
        self.table.setModel(self.model)  # table에 model을 씌우는 형식
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows) # 행 전체 선택
        self.table.setSelectionMode(QAbstractItemView.MultiSelection) # 다중 선택 가능 모드
        self.table.viewport().installEventFilter(self)  # 이벤트 필터

        self.splitter.addWidget(self.table)

    # 전체 드레그 일괄 해제
    def eventFilter(self, source, event):
        if event.type() == QEvent.MouseButtonPress and source is self.table.viewport():
            modifiers = QApplication.keyboardModifiers()
            if not (modifiers == Qt.ControlModifier or modifiers == Qt.ShiftModifier):
                self.table.clearSelection()
        return super(MainWindow2, self).eventFilter(source, event)

    # 파일(그래프)
    def graph(self):
        self.graphData.clear()
        # self.colors = ['pink'] * len(self.utm_x)  # 초기 색상

        spots = [{'pos': (self.utm_x[i], self.utm_y[i]), 'brush': pg.mkBrush(self.colors[i])} for i in range(len(self.utm_x))]

        self.scatter = pg.ScatterPlotItem(spots=spots, symbol='o', size=10)
        self.scatter.sigClicked.connect(self.clicked)
        self.graphData.addItem(self.scatter)

    def update_graph(self):
        self.graphData.clear()
        # utm_x의 길이 만큼 변경
        spots = [{'pos': (self.utm_x[i], self.utm_y[i]), 'brush': pg.mkBrush(self.colors[i])} for i in range(len(self.utm_x))]
        self.scatter = pg.ScatterPlotItem(spots=spots, symbol='o', size=10)
        self.scatter.sigClicked.connect(self.clicked)
        self.graphData.addItem(self.scatter)

    # 포인트 클릭 시 변화
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

    # 클릭한 포인트 번호
    def point(self, index):
        self.point_label.setText(f"선택한 포인트: {index}")  # QLabel 업데이트
        self.point_label.setFont(QFont('Arial', 10))

    # add 작성 버튼 + 저장 버튼
    def btns(self):
        btn_table = QTableWidget(self)
        btn_table.setFixedWidth(120)
        btn_table.setFixedHeight(390)

        btn_table.setColumnCount(1)
        btn_table.setRowCount(12)
        btn_table_column = ["버튼"]
        btn_table.setHorizontalHeaderLabels(btn_table_column)

        btn_labels = ["전진", "후진", "일시 정지", "오르막 일시 정지", "신호등 감지 구간", "가속 구간", "끝 지점", "차선 무시 구간", "정밀 전진", "정밀 후진", "감속 구간", "저장"]

        for i, label in enumerate(btn_labels):
            btn = QPushButton(label)
            btn.clicked.connect(partial(self.button_clicked, i))  # 버튼 번호 전달
            btn_table.setCellWidget(i, 0, btn)

        return btn_table

    def button_clicked(self, index):
        if index <= 10:
            selected_rows = self.table.selectionModel().selectedRows() # selectedRows: 현재 선택된 행의 인텍스 반환
            for row_index in selected_rows: # 선택된 행 처리
                row = row_index.row() #선택된 행 번호 반환
                self.model.setItem(row, 5, QStandardItem(str(index))) # 선택된 행에 5번째 열을 index값으로 변경

                if index == 0:
                    self.colors[row] = 'green'
                elif index == 1:
                    self.colors[row] = 'blue'
                elif index == 2:
                    self.colors[row] = 'red'
                elif index == 3:
                    self.colors[row] = 'yellow'
                elif index == 4:
                    self.colors[row] = 'white'
                elif index == 5:
                    self.colors[row] = 'cyan'
                elif index == 6:
                    self.colors[row] = 'purple'
                elif index == 7:
                    self.colors[row] = 'orange'
                elif index == 8:
                    self.colors[row] = 'gray'
                elif index == 9:
                    self.colors[row] = 'black'
                elif index == 10:
                    self.colors[row] = 'brown'

            self.update_graph()

        elif index == 11:
            file_name, _ = QFileDialog.getSaveFileName(self, "Save File", '', "csv Files (*.csv);; All Files (*)")

             # .csv로 끝나지 않는 경우
            if file_name and not file_name.endswith('.csv'):
                file_name += '.csv'

            if file_name:
                with open(file_name, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["seq", "latitude", "longitude", "latitude_utm", "longitude_utm", "option"])
                    for row in range(self.model.rowCount()):
                        seq = self.model.item(row, 0).text()
                        latitude = self.model.item(row, 1).text()
                        longitude = self.model.item(row, 2).text()
                        latitude_utm = self.model.item(row, 3).text()
                        longitude_utm = self.model.item(row, 4).text()
                        option = self.model.item(row, 5).text()
                        writer.writerow([seq, latitude, longitude, latitude_utm, longitude_utm, option])

                self.close()

    # 파일 데이터 불러오기
    def LoadData(self, filename):
        self.model.clear()
        self.model.setHorizontalHeaderLabels(["seq", "latitude", "longitude", "latitude_utm", "longitude_utm", "option"])
        self.utm_x.clear()
        self.utm_y.clear()
        self.colors.clear()

        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    if len(row) != 6:
                        print("잘못된 데이터 형식")
                        continue

                    seq, latitude, longitude, utm_x, utm_y, option = row
                    self.model.appendRow([
                        QStandardItem(seq),
                        QStandardItem(latitude),
                        QStandardItem(longitude),
                        QStandardItem(utm_x),
                        QStandardItem(utm_y),
                        QStandardItem(option),
                    ])
                    self.utm_x.append(float(utm_x))
                    self.utm_y.append(float(utm_y))

                    if option == "0":
                        self.colors.append('green')
                    elif option == "1":
                        self.colors.append('blue')
                    elif option == "2":
                        self.colors.append('red')
                    elif option == "3":
                        self.colors.append('yellow')
                    elif option == "4":
                        self.colors.append('white')
                    elif option == "5":
                        self.colors.append('cyan')
                    elif option == "6":
                        self.colors.append('purple')
                    elif option == "7":
                        self.colors.append('orange')
                    elif option == "8":
                        self.colors.append('gray')
                    elif option == "9":
                        self.colors.append('black')
                    elif option == "10":
                        self.colors.append('brown')
                    else:
                        self.colors.append('pink')

            self.table.resizeColumnsToContents()  # 열 크기 자동 조정
            self.graph()

        except Exception as e:
            print(f"파일을 읽는 중 오류 발생: {e}")

        self.currentLocation()

    def gps_callback(self, data: NavSatFix):
        latitude = data.latitude
        longitude = data.longitude

        utm_gps = utm.from_latlon(latitude, longitude)

        self.latitude_utm: float = utm_gps[0]
        self.longitude_utm: float = utm_gps[1]

        self.gps_signal.emit(self.latitude_utm, self.longitude_utm)
        # emit: 신호를 받았을 때 호출되는 함수, 신호가 발생하면 그에 대응하여 작업 수행, 값을 전달하면서 신호 발생

    def update_location(self, latitude_utm, longitude_utm):
        if self.current_point is not None:
            self.graphData.removeItem(self.current_point)

        self.current_point = self.graphData.plot([latitude_utm], [longitude_utm], pen=None, symbol='+', symbolBrush='red', symbolSize=15)

        self.update_table()

    def currentLocation(self):
        rospy.init_node('current_gps')
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gps_callback)

    def current_data(self):
        self.current_table = QTableWidget(self)
        self.current_table.setFixedWidth(120)

        self.current_table.setColumnCount(1)
        self.current_table.setRowCount(2)
        current_table_column = ["현재 좌표(utm)"]
        self.current_table.setHorizontalHeaderLabels(current_table_column)

        self.current_table.setItem(0, 0, QTableWidgetItem(str(self.latitude_utm)))
        self.current_table.setItem(1, 0, QTableWidgetItem(str(self.longitude_utm)))
        # setItem: 테이블의 셀에 텍스트나 데이터를 설정할 떄 사용
        # setCellWidget: QWidget을 삽입할 때 사용

        return self.current_table

    def update_table(self):
        self.current_table.setItem(0, 0, QTableWidgetItem(str(self.latitude_utm)))
        self.current_table.setItem(1, 0, QTableWidgetItem(str(self.longitude_utm)))

    # 위치
    def setup(self):
        file_layout = QVBoxLayout()
        file_layout.addWidget(self.FileOpenBtn)
        self.DataTable()
        file_layout.addWidget(self.splitter)

        file_widget = QWidget()
        file_widget.setLayout(file_layout)

        btn_table = self.btns()
        etc_layout = QVBoxLayout()
        etc_layout.addWidget(btn_table)

        self.point_label = QLabel("선택한 포인트: ", self)
        etc_layout.addWidget(self.point_label)

        self.current_data_table = self.current_data()
        etc_layout.addWidget(self.current_data_table)

        data_layout = QHBoxLayout()
        data_layout.addWidget(file_widget)
        data_layout.addLayout(etc_layout)

        self.graphData = pg.PlotWidget()
        data_layout.addWidget(self.graphData)

        central_widget = QWidget()
        central_widget.setLayout(data_layout)
        self.setCentralWidget(central_widget)

        self.table.selectionModel().selectionChanged.connect(self.selection_changed)

    def selection_changed(self, selected, deselected):
        # 선택된 행의 인덱스 추출
        selected_rows = self.table.selectionModel().selectedRows()

        if selected_rows:
            selected_index = selected_rows[0].row()  # 다중 선택은 첫 번째만 사용

            # Shift 키가 눌렸는지 확인
            modifiers = QApplication.keyboardModifiers()
            if modifiers == Qt.ShiftModifier and self.first_click_index is not None:
                # Shift 키가 눌린 상태에서 첫 번째 클릭과 두 번째 클릭 사이의 범위를 선택
                selection_model = self.table.selectionModel()
                selection_range = QItemSelection(
                    self.model.index(min(self.first_click_index, selected_index), 0),
                    self.model.index(max(self.first_click_index, selected_index), self.model.columnCount() - 1)
                )
                selection_model.select(selection_range, QItemSelectionModel.Select)

            else:
                # Shift 키가 눌리지 않았을 경우 첫 번째 클릭 위치 기록
                self.first_click_index = selected_index

            # 선택된 포인트를 빨간색으로, 나머지는 원래 색상으로 설정
            self.update_graph(selected_index)

            # 선택된 포인트 번호를 라벨에 업데이트
            self.point(selected_index + 1)

    def update_graph(self, selected_index=None):
        self.graphData.clear()  # 그래프 초기화

        spots = []
        for i in range(len(self.utm_x)):
            # 선택된 포인트는 빨간색, 나머지는 원래 색상으로
            color = 'red' if i == selected_index else self.colors[i]
            spots.append({'pos': (self.utm_x[i], self.utm_y[i]), 'brush': pg.mkBrush(color)})

        self.scatter = pg.ScatterPlotItem(spots=spots, symbol='o', size=10)
        self.graphData.addItem(self.scatter)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = MainWindow2()
    main.show()
    app.exec_()