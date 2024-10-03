import sys
import pandas as pd
import csv
import utm
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import pyqtgraph as pg
from functools import partial
from PyQt5.QtCore import *

import rospy
from sensor_msgs.msg import NavSatFix

class MainWindow(QMainWindow):
    gps_signal = pyqtSignal(float, float)
    def __init__(self):
        super().__init__()
        self.utm_x = []
        self.utm_y = []
        self.colors = []
        self.last_clicked = []
        self.current_point = None
        self.first_click_index = None

        self.latitude_utm = 0.0
        self.longitude_utm = 0.0

        # 선택한 포인트 출력
        self.point_table = QTableWidget(self)
        self.point_table.setColumnCount(1)
        self.point_table.setRowCount(1)
        point_table_column = ['선택한 포인트']
        self.point_table.setHorizontalHeaderLabels(point_table_column)
        self.point_table.setFixedWidth(120)
        self.point_table.setFixedHeight(60)

        self.setup_ui()
        self.setup()

        self.gps_signal.connect(self.update_location)

    def setup_ui(self):
        self.setWindowTitle('GPS Waypoint')
        self.setGeometry(100, 100, 1400, 900)

        #파일 열기 버튼 생성
        self.file_open_btn = QPushButton("파일 열기", self)
        self.file_open_btn.clicked.connect(self.open_file)
        self.file_open_btn.setFixedWidth(600)

    #파일 불러오기
    def open_file(self):
        file, _ = QFileDialog.getOpenFileName(self, "File Loader", "D:/ubuntu/", 'csv File(*.csv);; Text File(*.txt);; All Files(*)')
        if file:
            self.load_data(file)
        else:
            print("파일 불러오기 실패")

    def data_table(self):
        self.table = QTableView(self)
        self.table.setFixedWidth(600)
        self.table.setFixedHeight(900)

        # self.table.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)  # 테이블 크기 조정 설정
        self.model = QStandardItemModel(self)  # 데이터 모델 설정
        self.splitter = QSplitter(self)  # splitter: 하나의 박스로 묶어서 크기 조절 가능하도록 하는 기능
        self.table.setModel(self.model)  # table에 model을 씌우는 형식
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)  # 행 전체 선택
        self.table.setSelectionMode(QAbstractItemView.MultiSelection)  # 다중 선택 가능 모드
        self.table.viewport().installEventFilter(self)  # 이벤트 필터

        self.splitter.addWidget(self.table)

    # 전체 드레그 일괄 해제
    def eventFilter(self, source, event):
        if event.type() == QEvent.MouseButtonPress and source is self.table.viewport():
            modifiers = QApplication.keyboardModifiers()
            if not (modifiers == Qt.ControlModifier or modifiers == Qt.ShiftModifier):
                self.table.clearSelection()
        return super(MainWindow, self).eventFilter(source, event)

    # 파일(그래프)
    def graph(self):
        self.graphData.clear()
        # self.colors = ['pink'] * len(self.utm_x)  # 초기 색상

        spots = [{'pos': (self.utm_x[i], self.utm_y[i]), 'brush': pg.mkBrush(self.colors[i])} for i in
                 range(len(self.utm_x))]

        self.scatter = pg.ScatterPlotItem(spots=spots, symbol='o', size=10)
        self.scatter.sigClicked.connect(self.clicked)
        self.graphData.addItem(self.scatter)

    # 포인트 클릭 시 변화
    def clicked(self, plot, points):
        clickedPen = pg.mkPen(None)  # 테두리 없앰

        for p in self.last_clicked:
            p.setPen(clickedPen)

        self.last_clicked = points
        clickedPen = pg.mkPen('r', width=2)

        for p in points:
            p.setPen(clickedPen)

    def update_graph(self, selected_index=None):
        spots = []
        for i in range(len(self.utm_x)):
            # 선택된 포인트는 빨간색 테두리로 설정, 나머지는 원래 색상 유지
            if i == selected_index:
                spots.append({
                    'pos': (self.utm_x[i], self.utm_y[i]),
                    'brush': pg.mkBrush(self.colors[i]),  # 원래 색상 유지
                    'pen': pg.mkPen('r', width=2)  # 빨간색 테두리 추가
                })
            else:
                spots.append({
                    'pos': (self.utm_x[i], self.utm_y[i]),
                    'brush': pg.mkBrush(self.colors[i])  # 원래 색상
                })

        if hasattr(self, 'scatter'):
            self.scatter.setData(spots=spots)
        else:
            self.scatter = pg.ScatterPlotItem(spots=spots, symbol='o', size=10)
            self.scatter.sigClicked.connect(self.clicked)
            self.graphData.addItem(self.scatter)

    # 클릭한 포인트 번호
    def point(self, index):
        self.point_table.setItem(0, 0, QTableWidgetItem(str(index)))

    # add 작성 버튼 + 저장 버튼
    def btns(self):
        btn_table = QTableWidget(self)
        btn_table.setFixedWidth(120)
        btn_table.setFixedHeight(390)

        btn_table.setColumnCount(1)
        btn_table.setRowCount(12)
        btn_table_column = ["버튼"]
        btn_table.setHorizontalHeaderLabels(btn_table_column)

        btn_labels = ["전진", "후진", "일시 정지", "오르막 일시 정지", "신호등 감지 구간", "가속 구간", "끝 지점", "차선 무시 구간", "정밀 전진", "정밀 후진",
                      "감속 구간", "저장"]

        for i, label in enumerate(btn_labels):
            btn = QPushButton(label)
            btn.clicked.connect(partial(self.button_clicked, i))  # 버튼 번호 전달
            btn_table.setCellWidget(i, 0, btn)

        return btn_table

    def button_clicked(self, index):
        if index <= 10:
            selected_rows = self.table.selectionModel().selectedRows()  # selectedRows: 현재 선택된 행의 인텍스 반환
            for row_index in selected_rows:  # 선택된 행 처리
                row = row_index.row()  # 선택된 행 번호 반환
                self.model.setItem(row, 5, QStandardItem(str(index)))  # 선택된 행에 5번째 열을 index값으로 변경

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
                try:
                    # 데이터를 저장하는 부분
                    with open(file_name, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(["seq", "latitude", "longitude", "x", "y", "option"])
                        for row in range(self.model.rowCount()):
                            seq = self.model.item(row, 0).text()
                            latitude = self.model.item(row, 1).text()
                            longitude = self.model.item(row, 2).text()
                            x = self.model.item(row, 3).text()
                            y = self.model.item(row, 4).text()
                            option = self.model.item(row, 5).text()
                            writer.writerow([seq, latitude, longitude, x, y, option])
                    print(f"파일 저장 성공: {file_name}")
                except Exception as e:
                    print(f"파일 저장 중 오류 발생: {e}")

                self.close()

    def load_data(self, file):
        self.model.clear()
        self.model.setHorizontalHeaderLabels(["seq", "latitude", "longitude", "x", "y", "option"])
        self.utm_x.clear()
        self.utm_y.clear()
        self.colors.clear()

        try:
            df = pd.read_csv(file, dtype={'option': str})

            for _, row in df.iterrows():
                seq = row['seq']
                latitude = row['latitude']
                longitude = row['longitude']
                x = row['x']  # 'latitude_utm' 대신 'x'로 수정
                y = row['y']  # 'longitude_utm' 대신 'y'로 수정
                option = row['option']

                self.model.appendRow([
                    QStandardItem(str(seq)),
                    QStandardItem(str(latitude)),
                    QStandardItem(str(longitude)),
                    QStandardItem(str(x)),
                    QStandardItem(str(y)),
                    QStandardItem(str(option)),
                ])

                self.utm_x.append(float(x))
                self.utm_y.append(float(y))

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

            self.table.resizeColumnsToContents()
            self.graph()

        except Exception as e:
            print(f"파일 읽는 중 오류 발생: {e}")

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
        self.current_table.setFixedHeight(90)

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

    def delete_b(self):
        self.delete_btn = QPushButton('삭제', self)
        self.delete_btn.clicked.connect(self.delete_btn_clicked)
        return self.delete_btn

    def delete_btn_clicked(self):
        # 선택된 행을 가져옴
        selected_rows = self.table.selectionModel().selectedRows()

        if selected_rows:
            # 선택된 행을 내림차순으로 정렬하여 삭제 (역순으로 삭제해야 인덱스 오류 방지)
            for row in sorted(selected_rows, key=lambda x: x.row(), reverse=True):
                self.model.removeRow(row.row())

            # 그래프와 데이터를 다시 업데이트
            self.update_data_after_delete()

    def update_data_after_delete(self):
        # 테이블에서 삭제된 후의 데이터를 다시 불러와서 업데이트
        self.utm_x.clear()
        self.utm_y.clear()
        self.colors.clear()

        # 테이블의 남은 데이터를 기반으로 다시 좌표 및 색상 정보 업데이트
        for row in range(self.model.rowCount()):
            x = float(self.model.item(row, 3).text())
            y = float(self.model.item(row, 4).text())
            option = self.model.item(row, 5).text()

            self.utm_x.append(x)
            self.utm_y.append(y)

            # 색상 업데이트
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

        # 그래프 업데이트
        self.update_graph()

    def add_point_to_table(self, x, y):
        if self.selected_row_for_insertion is not None:
            row = self.selected_row_for_insertion + 1
        else:
            row = self.model.rowCount()

        self.model.insertRow(row)
        self.model.setItem(row, 0, QStandardItem(str(row + 1)))  # seq 번호
        self.model.setItem(row, 1, QStandardItem(str(self.latitude_utm)))  # latitude (여기서는 사용자의 현재 UTM 좌표)
        self.model.setItem(row, 2, QStandardItem(str(self.longitude_utm)))  # longitude (여기서는 사용자의 현재 UTM 좌표)
        self.model.setItem(row, 3, QStandardItem(str(x)))  # x (UTM)
        self.model.setItem(row, 4, QStandardItem(str(y)))  # y (UTM)
        self.model.setItem(row, 5, QStandardItem(str(0)))  # 기본 옵션 값

        # 좌표 그래프 업데이트
        self.utm_x.append(x)
        self.utm_y.append(y)
        self.colors.append('green')  # 기본 색상 추가
        self.update_graph()

    # 위치
    def setup(self):
        file_layout = QVBoxLayout()
        file_layout.addWidget(self.file_open_btn)
        self.data_table()
        file_layout.addWidget(self.splitter)

        file_widget = QWidget()
        file_widget.setLayout(file_layout)

        btn_table = self.btns()
        etc_layout = QVBoxLayout()
        etc_layout.addWidget(btn_table)

        self.current_data_table = self.current_data()
        etc_layout.addWidget(self.current_data_table)

        etc_layout.addWidget(self.point_table)

        self.delete_button = self.delete_b()
        etc_layout.addWidget(self.delete_button)

        spacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)
        etc_layout.addItem(spacer)

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
        # 선택된 행의 인덱스를 추출
        selected_rows = self.table.selectionModel().selectedRows()

        if selected_rows:
            selected_index = selected_rows[0].row()  # 다중 선택 중 첫 번째만 사용

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

            for p in self.last_clicked:
                p.setPen(pg.mkPen(None))  # 기본 상태로 복구

            self.last_clicked = []

            # 선택된 포인트를 빨간색으로, 나머지는 원래 색상으로 설정
            self.update_graph(selected_index)

            # 선택된 포인트 번호를 라벨에 업데이트
            self.point(selected_index + 1)

            # 선택된 포인트를 그래프에서 클릭한 것처럼 처리
            if self.scatter is not None and selected_index < len(self.scatter.points()):
                clicked_points = [self.scatter.points()[selected_index]]
                self.clicked(self.graphData, clicked_points)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = MainWindow()
    main.show()
    app.exec_()