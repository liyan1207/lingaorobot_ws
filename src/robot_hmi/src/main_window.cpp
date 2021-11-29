/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_hmi/main_window.hpp"
#include <QScrollBar>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked();
    }
    connect(ui.horizontalSlider_linera,SIGNAL(valueChanged(int)),this,SLOT(slot_linera_value_change(int)));
    connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(slot_raw_value_change(int)));
    connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_n,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_br,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));


    //init ui
    speed_x_dashBoard=new CCtrlDashBoard(ui.widget_speed_x);
    speed_y_dashBoard=new CCtrlDashBoard(ui.widget_speed_y);
    speed_x_dashBoard->setGeometry(ui.widget_speed_x->rect());
    speed_y_dashBoard->setGeometry(ui.widget_speed_y->rect());
    speed_x_dashBoard->setValue(0);
    speed_y_dashBoard->setValue(0);
    ui.horizontalSlider_linera->setValue(50);
    ui.horizontalSlider_raw->setValue(50);
    //connectsaf
    connect(&qnode,SIGNAL(speed_vel(float,float)),this,SLOT(slot_update_dashboard(float,float)));
    connect(&qnode,SIGNAL(power_vel(float)),this,SLOT(slot_update_power(float)));
    connect(&qnode,SIGNAL(image_val(QImage)),this,SLOT(slot_update_image(QImage)));
    connect(ui.pushButton_sub_image,SIGNAL(clicked()),this,SLOT(slot_sub_image()));

    connect(ui.laser_btn,SIGNAL(clicked()),this,SLOT(slot_quick_cmd_clicked()));




    //开一个终端运行非驱动指令
    m_proces_bash = new QProcess;
    m_proces_bash->start("bash");
    m_proces_bash->waitForStarted();
    connect(m_proces_bash, SIGNAL(readyReadStandardOutput()), this, SLOT(readBashStandardOutputInfo()));
    connect(m_proces_bash, SIGNAL(readyReadStandardError()), this, SLOT(readBashStandardErrorInfo()));

//    //单独开一个终端运行融合驱动
//    proces_bash = new QProcess;
//    proces_bash->start("bash");
//    proces_bash->waitForStarted();
//    //启动驱动与 IMU、RTK、融合驱动
//    QString strCmd = "roslaunch lingao_gnss robot.launch";
//    proces_bash->write(strCmd.toLocal8Bit() + '\n');//bash执行命令

//    //单独开一个终端运行摄像头驱动
//    proces_camera_bash= new QProcess;
//    proces_camera_bash->start("bash");
//    proces_camera_bash->waitForStarted();
//    //启动摄像头驱动
//    QString strCmdCamera = "roslaunch lingao_bringup lingao_camera.launch";
//    proces_camera_bash->write(strCmd.toLocal8Bit() + '\n');//bash执行命令

}
void MainWindow::slot_quick_cmd_clicked()
{
    laser_cmd=new QProcess;
    laser_cmd->start("bash");
    laser_cmd->write(ui.textEdit_laser_cmd->toPlainText().toLocal8Bit()+'\n');
    connect(laser_cmd,SIGNAL(readyReadStandardError()),this,SLOT(slot_quick_output()));
    connect(laser_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));
}
void MainWindow::slot_quick_output()
{
    ui.textEdit_quick_output->append("<font color=\"#FF0000\">"+laser_cmd->readAllStandardError()+"</font>");
    ui.textEdit_quick_output->append("<font color=\"#FFFFFF\">"+laser_cmd->readAllStandardOutput()+"</font>");
}
void MainWindow::slot_update_image(QImage im)
{
    ui.label_image->setPixmap(QPixmap::fromImage(im));
}
void MainWindow::slot_sub_image()
{
    qnode.sub_image(ui.lineEdit_image_topic->text());
}
void MainWindow::slot_update_power(float value)
{
      ui.label_power_val->setText(QString::number(value).mid(0,5)+"V");
      double n=(value-10.5)/(12.5-10.5);
      int val=n*100;
      ui.progressBar->setValue(val);
}
void MainWindow::slot_update_dashboard(float x,float y)
{
    ui.label_dir_x->setText(x>0?"正向":"反向");
    ui.label_dir_y->setText(y>0?"正向":"反向");
    speed_x_dashBoard->setValue(abs(x)*100);
    speed_y_dashBoard->setValue(abs(y)*100);
}
void MainWindow::slot_pushbtn_click()
{
  QPushButton* btn=qobject_cast<QPushButton*> (sender());
  char k=btn->text().toStdString()[0];
  bool is_all=ui.checkBox_is_all->isChecked();
  float linear=ui.label_linera->text().toFloat()*0.01;
  float angular=ui.label_raw->text().toFloat()*0.01;

  switch (k) {
    case 'i':
      qnode.set_cmd_vel(is_all?'I':'i',linear,angular);
      break;
  case 'u':
    qnode.set_cmd_vel(is_all?'U':'u',linear,angular);
    break;
  case 'o':
    qnode.set_cmd_vel(is_all?'O':'o',linear,angular);
    break;
  case 'j':
    qnode.set_cmd_vel(is_all?'J':'j',linear,angular);
    break;
  case 'l':
    qnode.set_cmd_vel(is_all?'L':'l',linear,angular);
    break;
  case 'm':
    qnode.set_cmd_vel(is_all?'M':'m',linear,angular);
    break;
  case ',':
    qnode.set_cmd_vel(is_all?'<':',',linear,angular);
    break;
  case '.':
    qnode.set_cmd_vel(is_all?'>':'.',linear,angular);
    break;
  }
}
void MainWindow::slot_linera_value_change(int value)
{
    ui.label_linera->setText(QString::number(value));
}
void MainWindow::slot_raw_value_change(int value)
{
    ui.label_raw->setText(QString::number(value));
}
MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 * 连接设备
 */
void MainWindow::on_button_connect_clicked() {
  if ( ui.checkbox_use_environment->isChecked() ) { //使用环境变量
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
           ui.line_edit_host->text().toStdString()) ) { // ros master 连接失败
			showNoMasterMessage();
    } else { // ros master 连接成功
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);

      //小车上位机已经运行驱动
      //复位升降结构Z轴
      QString strCmdResetZ = "rosservice call /lingao_base/linear_motion_sys_init \"AxisName: 'z'\"";
      writeCmd(strCmdResetZ);
      //复位升降结构X轴
      QString strCmdResetX = "rosservice call /lingao_base/linear_motion_sys_init \"AxisName: 'x'\"";
      writeCmd(strCmdResetX);
      //复位相机
      QString strCmdResetP = "rosservice call /lingao_base/linear_motion_sys_init \"AxisName: 'p'\"";
      writeCmd(strCmdResetP);

      //监控x、z轴位置


		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_hmi");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_hmi");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace robot_hmi








/**
 * @brief MainWindow::readBashStandardOutputInfo
 * 读取命令并执行
 */
void robot_hmi::MainWindow::readBashStandardOutputInfo()
{
  QByteArray cmdout = m_proces_bash->readAllStandardOutput();//读bash执行命令后的输出
  if(!cmdout.isEmpty()){
      ui.textEdit_bashmsg->append(QString::fromLocal8Bit(cmdout));//显示输出结果
  }
  QScrollBar* scroll = ui.textEdit_bashmsg->verticalScrollBar();
  scroll->setSliderPosition(scroll->maximum());//光标定位到最后一行
}

/**
 * @brief MainWindow::readBashStandardErrorInfo
 * 读取异常信息
 */
void robot_hmi::MainWindow::readBashStandardErrorInfo()
{
  QByteArray cmdout = m_proces_bash->readAllStandardError();//读bash执行命令后的错误信息
  if(!cmdout.isEmpty()){
      ui.textEdit_bashmsg->append(QString::fromLocal8Bit(cmdout));//显示
  }
  QScrollBar* scroll = ui.textEdit_bashmsg->verticalScrollBar();
  scroll->setSliderPosition(scroll->maximum());//光标定位到最后一行
}

/**
 * @brief MainWindow::on_pushBtn_okx_clicked
 * 设置x轴位置
 */
void robot_hmi::MainWindow::on_pushBtn_okx_clicked()
{
  QString strCmd = "rosservice call /lingao_base/linear_motion_sys_set \"AxisName: 'x'";
  writeCmd(strCmd);

  QString strx = ui.spinBox_x->text();
  strCmd = "distance: ";
  strCmd.append(strx);
  writeCmd(strCmd);

  QString strxspeed = ui.spinBox_xspeed->text();
  strCmd = "speed: ";
  strCmd.append(strxspeed);
  strCmd.append("\""); //命令内容
  writeCmd(strCmd);
}

/**
 * @brief MainWindow::on_pushBtn_okz_clicked
 * 设置z轴位置
 */
void robot_hmi::MainWindow::on_pushBtn_okz_clicked()
{
  QString strCmd = "rosservice call /lingao_base/linear_motion_sys_set \"AxisName: 'z'";
  writeCmd(strCmd);

  QString strz = ui.spinBox_z->text();
  strCmd = "distance: ";
  strCmd.append(strz);
  writeCmd(strCmd);

  QString strzspeed = ui.spinBox_zspeed->text();
  strCmd = "speed: ";
  strCmd.append(strzspeed);
  strCmd.append("\""); //命令内容
  writeCmd(strCmd);
}

/**
 * @brief MainWindow::on_pushBtn_okcamera_clicked
 * 设置相机位置
 */
void robot_hmi::MainWindow::on_pushBtn_okcamera_clicked()
{
  QString strCmd = "rosservice call /lingao_base/linear_motion_sys_set \"AxisName: 'p'";
  writeCmd(strCmd);

  QString strcamera = ui.spinBox_camera->text();
  strCmd = "distance: ";
  strCmd.append(strcamera);
  writeCmd(strCmd);

  QString strcameraspeed = ui.spinBox_cameraspeed->text();
  strCmd = "speed: ";
  strCmd.append(strcameraspeed);
  strCmd.append("\""); //命令内容
  writeCmd(strCmd);
}

/**
 * @brief MainWindow::writeCmd
 * @param strCmd
 * 执行bash命令
 */
void robot_hmi::MainWindow::writeCmd(QString strCmd)
{
  ui.textEdit_bashmsg->append("Linux:~$ "+strCmd);//显示命令
  m_proces_bash->write(strCmd.toLocal8Bit() + '\n');//bash执行命令
}

