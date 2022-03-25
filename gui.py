from PyQt5.QtWidgets import QComboBox, QMainWindow, QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QCheckBox
import sys

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.cable = False
        self.rigidlink = False
        self.threeMAV = False

        self.printstatement = 'Use check box menu to select a setting'
        self.combobox1 = QComboBox()
        self.combobox2 = QComboBox()
        self.combobox3 = QComboBox()
        
        self.l0 = QLabel()
        self.l1 = QLabel()
        self.l2 = QLabel()
        self.l3 = QLabel()
        self.l4 = QLabel()

        self.l0.setText("Use this gui to print out the launch commend")
        self.l1.setText("Select number of MAV(s) [select one]:")
        self.l2.setText("Select connection mechanism [select one]:")
        self.l3.setText("Select payload type [select one]: ")
        self.l4.setText("Close Window to reset the check list")
        # check box
        self.checkBox1 = QCheckBox()
        self.checkBox2 = QCheckBox()
        self.checkBox3 = QCheckBox()
        self.checkBox4 = QCheckBox()

        self.checkBox1.setText("One")
        self.checkBox1.setChecked(False)
        self.checkBox1.stateChanged.connect(lambda:self.ptmass())
        self.checkBox2.setText("Three")
        self.checkBox2.setChecked(False)
        self.checkBox2.stateChanged.connect(lambda:self.three())
        self.checkBox3.setText("Four")
        self.checkBox3.setChecked(False)
        self.checkBox3.stateChanged.connect(lambda:self.Coop4())
        self.checkBox4.setText("Six")
        self.checkBox4.setChecked(False)
        self.checkBox4.stateChanged.connect(lambda:self.Coop6())

        self.checkBox5 = QCheckBox()
        self.checkBox6 = QCheckBox()
 
        self.checkBox5.setText("Rigid Link")
        self.checkBox5.setChecked(False)
        self.checkBox5.stateChanged.connect(lambda:self.rlink3())
        self.checkBox6.setText("Cable")
        self.checkBox6.setChecked(False)
        self.checkBox6.stateChanged.connect(lambda:self.cable3())

        self.checkBox7 = QCheckBox()
        self.checkBox8 = QCheckBox()
        self.checkBox7.setText("Point Mass")
        self.checkBox7.setChecked(False)
        self.checkBox7.stateChanged.connect(lambda:self.ptmass())
        self.checkBox8.setText("Non Point Mass")
        self.checkBox8.setChecked(False)
        self.checkBox8.stateChanged.connect(lambda:self.nonptmass())


        # button
        self.b1 = QPushButton("Print")
        self.b1.setCheckable(False)
        self.b1.toggle()
        self.b1.clicked.connect(self.btnstate1)

        '''self.b2 = QPushButton("reset")
        self.b2.setCheckable(False)
        self.b2.toggle()
        self.b2.clicked.connect(self.btnstate2)'''

        # set up layout
        layout = QVBoxLayout()
        layout.addWidget(self.l0)
        layout.addWidget(self.l1)
        layout.addWidget(self.checkBox1)
        layout.addWidget(self.checkBox2)
        layout.addWidget(self.checkBox3)
        layout.addWidget(self.checkBox4)
        layout.addWidget(self.l2)
        layout.addWidget(self.checkBox5)
        layout.addWidget(self.checkBox6)
        layout.addWidget(self.l3)
        layout.addWidget(self.checkBox7)
        layout.addWidget(self.checkBox8)
        layout.addWidget(self.b1)
        # layout.addWidget(self.b2)
        layout.addWidget(self.l4)
        container = QWidget()
        container.setLayout(layout)

        self.setCentralWidget(container)
    
    def nonptmass(self):
        self.checkBox1.setDisabled(True)
        self.checkBox7.setDisabled(True)
        self.checkBox8.setDisabled(True)

    def cable3(self):
        if self.threeMAV:
            self.checkBox5.setDisabled(True)
            self.checkBox6.setDisabled(True)
            self.printstatement = "roslaunch rotor_tm rotortm_cable_3.launch"
        else:
            self.cable = True
            self.checkBox5.setDisabled(True)
            self.checkBox6.setDisabled(True)
        

    def rlink3(self):
        self.checkBox1.setDisabled(True)
        self.checkBox2.setChecked(True)
        self.checkBox2.setDisabled(True)
        self.checkBox3.setDisabled(True)
        self.checkBox4.setDisabled(True)
        self.checkBox5.setDisabled(True)
        self.checkBox6.setDisabled(True)
        self.checkBox7.setDisabled(True)
        self.checkBox8.setChecked(True)
        self.checkBox8.setDisabled(True)
        self.printstatement = "roslaunch rotor_tm rotortm_rlink_3.launch"
        self.rigidlink = True

    def three(self):
        self.checkBox1.setDisabled(True)
        self.checkBox2.setDisabled(True)
        self.checkBox3.setDisabled(True)
        self.checkBox4.setDisabled(True)
        self.checkBox7.setDisabled(True)
        self.checkBox8.setChecked(True)
        self.checkBox8.setDisabled(True)
        if self.cable:
            self.printstatement = "roslaunch rotor_tm rotortm_cable_3.launch"
        elif self.rigidlink:
            self.printstatement = "roslaunch rotor_tm rotortm_rlink_3.launch"
        self.threeMAV = True

    def ptmass(self):
        self.checkBox1.setChecked(True)
        self.checkBox1.setDisabled(True)
        self.checkBox2.setDisabled(True)
        self.checkBox3.setDisabled(True)
        self.checkBox4.setDisabled(True)
        self.checkBox5.setDisabled(True)
        self.checkBox6.setChecked(True)
        self.checkBox6.setDisabled(True)
        self.checkBox7.setChecked(True)
        self.checkBox7.setDisabled(True)
        self.checkBox8.setDisabled(True)
        self.printstatement = "roslaunch rotor_tm rotortm_ptmass.launch"

    def Coop4(self):
        self.checkBox1.setDisabled(True)
        self.checkBox2.setDisabled(True)
        self.checkBox3.setChecked(True)
        self.checkBox3.setDisabled(True)
        self.checkBox4.setDisabled(True)
        self.checkBox5.setDisabled(True)
        self.checkBox6.setChecked(True)
        self.checkBox6.setDisabled(True)
        self.checkBox7.setDisabled(True)
        self.checkBox8.setChecked(True)
        self.checkBox8.setDisabled(True)
        self.printstatement = "roslaunch rotor_tm rotortm_cable_4.launch"

    def Coop6(self):
        self.checkBox1.setDisabled(True)
        self.checkBox2.setDisabled(True)
        self.checkBox3.setDisabled(True)
        self.checkBox4.setChecked(True)
        self.checkBox4.setDisabled(True)
        self.checkBox5.setDisabled(True)
        self.checkBox6.setChecked(True)
        self.checkBox6.setDisabled(True)
        self.checkBox7.setDisabled(True)
        self.checkBox8.setChecked(True)
        self.checkBox8.setDisabled(True)
        self.printstatement = "roslaunch rotor_tm rotortm_cable_6.launch"

    def btnstate1(self):
      if self.b1.isChecked():
         pass
      else:
         print(self.printstatement)

    '''def btnstate2(self):
      if self.b1.isChecked():
         pass
      else:
         # set all check box state to True
        self.checkBox1.setDisabled(False)
        self.checkBox1.setChecked(False)
        self.checkBox2.setDisabled(False)
        self.checkBox2.setChecked(False)
        self.checkBox3.setDisabled(False)
        self.checkBox3.setChecked(False)
        self.checkBox4.setDisabled(False)
        self.checkBox4.setChecked(False)
        self.checkBox5.setDisabled(False)
        self.checkBox5.setChecked(False)
        self.checkBox6.setDisabled(False)
        self.checkBox6.setChecked(False)
        self.checkBox7.setDisabled(False)
        self.checkBox7.setChecked(False)
        self.checkBox8.setDisabled(False)
        self.checkBox8.setChecked(False)'''



app = QApplication(sys.argv)
w = MainWindow()
w.show()
app.exec_()