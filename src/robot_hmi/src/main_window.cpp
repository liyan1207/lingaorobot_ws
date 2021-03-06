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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

    QObject::connect(ui.actionconnect, SIGNAL(triggered()), this, SLOT(connectEquipment()));

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
//    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
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
//    speed_x_dashBoard=new CCtrlDashBoard(ui.widget_speed_x);
//    speed_y_dashBoard=new CCtrlDashBoard(ui.widget_speed_y);
//    speed_x_dashBoard->setGeometry(ui.widget_speed_x->rect());
//    speed_y_dashBoard->setGeometry(ui.widget_speed_y->rect());
//    speed_x_dashBoard->setValue(0);
//    speed_y_dashBoard->setValue(0);
    ui.horizontalSlider_linera->setValue(10);
    ui.horizontalSlider_raw->setValue(10);

    //connectsaf
//    connect(&qnode,SIGNAL(speed_vel(float,float)),this,SLOT(slot_update_dashboard(float,float)));
    connect(&qnode,SIGNAL(power_vel(float)),this,SLOT(slot_update_power(float)));
    connect(&qnode,SIGNAL(image_val(QImage)),this,SLOT(slot_update_image(QImage)));
    connect(&qnode,SIGNAL(image_val0(QImage)),this,SLOT(slot_show_image0(QImage)));
    connect(&qnode,SIGNAL(image_val1(QImage)),this,SLOT(slot_show_image1(QImage)));
    connect(&qnode,SIGNAL(image_val2(QImage)),this,SLOT(slot_show_image2(QImage)));

    connect(ui.pushButton_sub_image,SIGNAL(clicked()),this,SLOT(slot_sub_image()));
    connect(ui.laser_btn,SIGNAL(clicked()),this,SLOT(slot_quick_cmd_clicked()));

    //????????????????????????????????????
    m_proces_bash = new QProcess;
    m_proces_bash->start("bash");
    m_proces_bash->waitForStarted();
    connect(m_proces_bash, SIGNAL(readyReadStandardOutput()), this, SLOT(readBashStandardOutputInfo()));
    connect(m_proces_bash, SIGNAL(readyReadStandardError()), this, SLOT(readBashStandardErrorInfo()));
}

void MainWindow::slot_update_image(QImage im)
{
    ui.label_image->setPixmap(QPixmap::fromImage(im));
}

void MainWindow::slot_show_image0(QImage im0)
{
    ui.label_image0->setPixmap(QPixmap::fromImage(im0));
}
void MainWindow::slot_show_image1(QImage im1)
{
    ui.label_image1->setPixmap(QPixmap::fromImage(im1));
}
void MainWindow::slot_show_image2(QImage im2)
{
    ui.label_image2->setPixmap(QPixmap::fromImage(im2));
}

void MainWindow::slot_sub_image()
{
//    qnode.sub_image(ui.lineEdit_image_topic->text());
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
//    ui.label_dir_x->setText(x>0?"??????":"??????");
//    ui.label_dir_y->setText(y>0?"??????":"??????");
//    speed_x_dashBoard->setValue(abs(x)*100);
//    speed_y_dashBoard->setValue(abs(y)*100);
}
void MainWindow::slot_pushbtn_click()
{
  QPushButton* btn=qobject_cast<QPushButton*> (sender());
  char k=btn->text().toStdString()[0];
  bool is_all=false;//ui.checkBox_is_all->isChecked();
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
 * ????????????
 */
void MainWindow::on_button_connect_clicked() {
  if ( ui.checkbox_use_environment->isChecked() ) { //??????????????????
		if ( !qnode.init() ) {
			showNoMasterMessage();
    }
    else {
			ui.button_connect->setEnabled(false);
		}
  }
  else
  {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
           ui.line_edit_host->text().toStdString()) ) { // ros master ????????????
			showNoMasterMessage();
    }
    else
    { // ros master ????????????
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
//			ui.line_edit_topic->setReadOnly(true);
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









/**
 * @brief MainWindow::readBashStandardOutputInfo
 * ?????????????????????
 */
void robot_hmi::MainWindow::readBashStandardOutputInfo()
{
  QByteArray cmdout = m_proces_bash->readAllStandardOutput();//???bash????????????????????????
  if(!cmdout.isEmpty()){
    ui.textEdit_bashmsg->append("<font color=\"#FFFFFF\">Linux:~$ "+QString::fromLocal8Bit(cmdout)+"</font>");//??????????????????
  }
  QScrollBar* scroll = ui.textEdit_bashmsg->verticalScrollBar();
  scroll->setSliderPosition(scroll->maximum());//???????????????????????????
}

/**
 * @brief MainWindow::readBashStandardErrorInfo
 * ??????????????????
 */
void robot_hmi::MainWindow::readBashStandardErrorInfo()
{
  QByteArray cmdout = m_proces_bash->readAllStandardError();//???bash??????????????????????????????
  if(!cmdout.isEmpty()){
    ui.textEdit_bashmsg->append("<font color=\"#FFFFFF\">Linux:~$ "+QString::fromLocal8Bit(cmdout)+"</font>");//????????????
  }
  QScrollBar* scroll = ui.textEdit_bashmsg->verticalScrollBar();
  scroll->setSliderPosition(scroll->maximum());//???????????????????????????
}

QString getmulti(QString str)
{
  double x = str.toDouble() * 10;
  QString strre = QString::number(x);

  return strre;
}

/**
 * @brief MainWindow::on_pushBtn_okx_clicked
 * ??????x?????????
 */
void robot_hmi::MainWindow::on_pushBtn_okx_clicked()
{
  QString strCmdsource = "source ~/lingao_ws/devel/setup.bash";
  writeCmd(strCmdsource);

  QString strCmd = "rosservice call /lingao_base/linear_motion_sys_set \"AxisName: 'x'";
  writeCmd(strCmd);

  QString strx = ui.doubleSpinBox_y->text();

  strx = getmulti(strx);

  strCmd = "distance: ";
  strCmd.append(strx);
  writeCmd(strCmd);

  QString strxspeed = ui.doubleSpinBox_ys->text();

  strxspeed = getmulti(strxspeed);

  strCmd = "speed: ";
  strCmd.append(strxspeed);
  strCmd.append("\""); //????????????
  writeCmd(strCmd);
}

/**
 * @brief MainWindow::on_pushBtn_okz_clicked
 * ??????z?????????
 */
void robot_hmi::MainWindow::on_pushBtn_okz_clicked()
{
  QString strCmdsource = "source ~/lingao_ws/devel/setup.bash";
  writeCmd(strCmdsource);

  QString strCmd = "rosservice call /lingao_base/linear_motion_sys_set \"AxisName: 'z'";
  writeCmd(strCmd);

  QString strz = ui.doubleSpinBox_z->text();

  strz = getmulti(strz);

  strCmd = "distance: ";
  strCmd.append(strz);
  writeCmd(strCmd);

  QString strzspeed = ui.doubleSpinBox_zs->text();

  strzspeed = getmulti(strzspeed);

  strCmd = "speed: ";
  strCmd.append(strzspeed);
  strCmd.append("\""); //????????????
  writeCmd(strCmd);
}

/**
 * @brief MainWindow::on_pushBtn_okcamera_clicked
 * ??????????????????
 */
void robot_hmi::MainWindow::on_pushBtn_okcamera_clicked()
{
  QString strCmdsource = "source ~/lingao_ws/devel/setup.bash";
  writeCmd(strCmdsource);

  QString strCmd = "rosservice call /lingao_base/linear_motion_sys_set \"AxisName: 'p'";
  writeCmd(strCmd);

  QString strcamera = ui.spinBox_camera->text();

  strCmd = "distance: ";
  strCmd.append(strcamera);
  writeCmd(strCmd);

  QString strcameraspeed = ui.spinBox_cameraspeed->text();

  strCmd = "speed: ";
  strCmd.append(strcameraspeed);
  strCmd.append("\""); //????????????
  writeCmd(strCmd);
}

/**
 * @brief MainWindow::writeCmd
 * @param strCmd
 * ??????bash??????
 */
void robot_hmi::MainWindow::writeCmd(QString strCmd)
{
  ui.textEdit_bashmsg->append("<font color=\"#FFFFFF\">Linux:~$ "+strCmd+"</font>");//????????????
  m_proces_bash->write(strCmd.toLocal8Bit() + '\n');//bash????????????
}

void MainWindow::connectEquipment()
{
//  connectdev = new DialogConnectDevice(NULL);
//  connectdev->show();
}

void MainWindow::slot_quick_cmd_clicked()
{
  laser_cmd=new QProcess;
  laser_cmd->start("bash");
  ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">Linux:~$ "+ui.textEdit_laser_cmd->toPlainText()+"</font>");//????????????
  laser_cmd->write(ui.textEdit_laser_cmd->toPlainText().toLocal8Bit()+'\n');
  connect(laser_cmd,SIGNAL(readyReadStandardError()),this,SLOT(slot_quick_output()));
  connect(laser_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));
}

/**
 * @brief MainWindow::slot_quick_output
 * ?????????????????????
 */
void MainWindow::slot_quick_output()
{
    ui.textEdit_initoutput->append("<font color=\"#FF0000\">"+laser_cmd->readAllStandardError()+"</font>");
    ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">"+laser_cmd->readAllStandardOutput()+"</font>");
}



/**
 * ????????????????????????????????????
 */
void robot_hmi::MainWindow::on_pushBtn_inithard_clicked()
{  
  proces_hard=new QProcess;
  proces_hard->start("bash");
  ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">Linux:~$ "+ui.textEdit_inithard->toPlainText()+"</font>");//????????????
  proces_hard->write(ui.textEdit_inithard->toPlainText().toLocal8Bit()+'\n');
  connect(proces_hard,SIGNAL(readyReadStandardError()),this,SLOT(slot_inithard_output()));
  connect(proces_hard,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_inithard_output()));

  //?????????????????????????????????
  QString strCmdsource = "source ~/lingao_ws/devel/setup.bash";
  writeCmd(strCmdsource);
  //??????????????????Z???
  QString strCmdResetZ = "rosservice call /lingao_base/linear_motion_sys_init \"AxisName: 'z'\"";
  writeCmd(strCmdResetZ);
  //??????????????????X???
  QString strCmdResetX = "rosservice call /lingao_base/linear_motion_sys_init \"AxisName: 'x'\"";
  writeCmd(strCmdResetX);
}

void MainWindow::slot_inithard_output()
{
    ui.textEdit_initoutput->append("<font color=\"#FF0000\">"+proces_hard->readAllStandardError()+"</font>");
    ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">"+proces_hard->readAllStandardOutput()+"</font>");
}

void robot_hmi::MainWindow::on_pushBtn_initcamera_clicked()
{
  proces_camera=new QProcess;
  proces_camera->start("bash");
  ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">Linux:~$ "+ui.textEdit_initcamera->toPlainText()+"</font>");//????????????
  proces_camera->write(ui.textEdit_initcamera->toPlainText().toLocal8Bit()+'\n');
  connect(proces_camera,SIGNAL(readyReadStandardError()),this,SLOT(slot_initcamera_output()));
  connect(proces_camera,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_initcamera_output()));

  //?????????????????????????????????
  QString strCmdsource = "source ~/lingao_ws/devel/setup.bash";
  writeCmd(strCmdsource);
  //????????????
  QString strCmdResetP = "rosservice call /lingao_base/linear_motion_sys_init \"AxisName: 'p'\"";
  writeCmd(strCmdResetP);
}

void MainWindow::slot_initcamera_output()
{
    ui.textEdit_initoutput->append("<font color=\"#FF0000\">"+proces_camera->readAllStandardError()+"</font>");
    ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">"+proces_camera->readAllStandardOutput()+"</font>");
}

void robot_hmi::MainWindow::on_pushBtn_initlaser_clicked()
{
  proces_laser=new QProcess;
  proces_laser->start("bash");
  ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">Linux:~$ "+ui.textEdit_initlaser->toPlainText()+"</font>");//????????????
  proces_laser->write(ui.textEdit_initlaser->toPlainText().toLocal8Bit()+'\n');
  connect(proces_laser,SIGNAL(readyReadStandardError()),this,SLOT(slot_initlaser_output()));
  connect(proces_laser,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_initlaser_output()));
}

void MainWindow::slot_initlaser_output()
{
    ui.textEdit_initoutput->append("<font color=\"#FF0000\">"+proces_laser->readAllStandardError()+"</font>");
    ui.textEdit_initoutput->append("<font color=\"#FFFFFF\">"+proces_laser->readAllStandardOutput()+"</font>");
}


/**
 * @brief robot_hmi::MainWindow::on_pushBtn_stop_clicked
 * ??????
 */
void robot_hmi::MainWindow::on_pushBtn_stop_clicked()
{
  qnode.set_cmd_vel('I',0,0);
}

void robot_hmi::MainWindow::on_quit_button_clicked()
{

}

void robot_hmi::MainWindow::on_pushButton_camera0_clicked()
{
  qnode.sub_image0(topic_name0);
}

void robot_hmi::MainWindow::on_pushButton_camera1_clicked()
{
  qnode.sub_image1(topic_name1);
}

void robot_hmi::MainWindow::on_pushButton_camera2_clicked()
{
  qnode.sub_image2(topic_name2);
}

void robot_hmi::MainWindow::on_pushBtn_rgboper_clicked()
{
  QImage img1;
  img1.load ("/home/y/??????/testrgboper.png");                                        // ??????????????????
  QPixmap pixmap(QPixmap::fromImage (img1));               // ??????Pixmap???
  ui.label_image0->setPixmap (pixmap);                           // ??????????????????

  QImage img = ui.label_image0->pixmap()->toImage();    // ??????RGB???????????????



  Blue(img, ui.label_imagergboper, "b");
}


//void split_gray(Mat src)
//{
//  int size = src.rows * src.cols * 3;
//  Mat b(src.rows, src.cols, CV_8UC1);
//  Mat g(src.rows, src.cols, CV_8UC1);
//  Mat r(src.rows, src.cols, CV_8UC1);

//  Mat out[] = {b, g, r};
//  split(src, out);

//  for(int i=0; i<size; i+=3)
//  {
//    b.data[i/3] = src.data[i];
//    g.data[i/3] = src.data[i+1];
//    r.data[i/3] = src.data[i+2];
//  }

//  imshow("b", b);
//  imshow("g", g);
//  imshow("r", r);
//}

/***************************************
 Function: Channel::Blue(QImage img, QLabel *imgLabel)
 Description: ??????????????????
 Called By: MainWindow::channelBlue()
 Input: ??????????????????Path??????????????????imgLabel
 Output: "??????????????????"
 Return: ??????????????????Path??????????????????imgLabel
 Others: ???????????????????????????????????????0
***************************************/
void robot_hmi::MainWindow::Blue(QImage img, QLabel *imgLabel, QString chanel)
{
    unsigned char *redData;                                // ???????????????????????????????????????????????????
    unsigned char *greenData;
    unsigned char *blueData;

    unsigned char *data = img.bits ();                      // ?????????????????????????????????
    int width = img.width ();                               // ????????????
    int height = img.height ();                             // ????????????
    int bytePerLine = img.bytesPerLine();                   // ?????????????????????

    redData = new unsigned char [bytePerLine * height];    // ??????????????????
    greenData = new unsigned char [bytePerLine * height];
    blueData = new unsigned char [bytePerLine * height];

    unsigned char red = 0;                                 // ??????
    unsigned char green = 0;
    unsigned char blue = 0;
    for (int i = 0; i < height; i++)                        // ???????????????
    {
        for ( int j = 0; j < width; j++ )                   // ???????????????
        {
            red = *(data);                                 // ???????????????????????????
            green = *(data+1);
            blue = *(data+2);

            redData[i * bytePerLine + j * 3] = red;
            greenData[i * bytePerLine + j * 3]=green;
            blueData[i * bytePerLine + j * 3]=blue;       // ???????????????

            data += 1;                                      // ????????????????????????
        }
    }

    QImage redImage(redData, width, height, bytePerLine, QImage::Format_Grayscale8);
    QPixmap pixmapred(QPixmap::fromImage (redImage));        // ??????????????????
    ui.label_imagergboper->setPixmap (pixmapred);                          // ??????????????????

    QImage greenImage(greenData, width, height, bytePerLine, QImage::Format_Grayscale8);
    QPixmap pixmapgreen(QPixmap::fromImage (greenImage));        // ??????????????????
    ui.label_image1->setPixmap (pixmapgreen);                          // ??????????????????

    QImage blueImage(blueData, width, height, bytePerLine, QImage::Format_Grayscale8);
    QPixmap pixmap2(QPixmap::fromImage (blueImage));        // ??????????????????
    ui.label_image2->setPixmap (pixmap2);                          // ??????????????????
}


}  // namespace robot_hmi
