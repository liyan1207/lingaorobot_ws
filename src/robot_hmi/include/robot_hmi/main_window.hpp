/**
 * @file /include/robot_hmi/main_window.hpp
 *
 * @brief Qt based gui for robot_hmi.
 *
 * @date November 2010
 **/
#ifndef robot_hmi_MAIN_WINDOW_H
#define robot_hmi_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "CCtrlDashBoard.h"
#include <QImage>
#include <QProcess>

//#include "dialogconnectdevice.h"
//#include "ui_dialogconnectdevice.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_hmi {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
  QString topic_name0="/camera1/image_raw";
  QString topic_name1="/camera2/image_raw";
  QString topic_name2="/camera3/image_raw";

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
        void on_button_connect_clicked();
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_linera_value_change(int);
    void slot_raw_value_change(int);
    void slot_pushbtn_click();
    void slot_update_dashboard(float,float);
    void slot_update_power(float);
    void slot_update_image(QImage);
    void slot_sub_image();
    void slot_quick_cmd_clicked();
    void slot_quick_output();
    void slot_inithard_output();
    void slot_initcamera_output();
    void slot_initlaser_output();
    void slot_show_image0(QImage);
    void slot_show_image1(QImage);
    void slot_show_image2(QImage);
    void Blue(QImage img, QLabel *imgLabel,  QString chanel);

private slots:
    void on_pushBtn_okx_clicked();
    void on_pushBtn_okz_clicked();
    void on_pushBtn_okcamera_clicked();
    void readBashStandardOutputInfo();
    void readBashStandardErrorInfo();
    void writeCmd(QString);
    void connectEquipment();
    void on_pushBtn_inithard_clicked();
    void on_pushBtn_initcamera_clicked();
    void on_pushBtn_initlaser_clicked();
    void on_pushBtn_stop_clicked();
    void on_quit_button_clicked();
    void on_pushButton_camera0_clicked();
    void on_pushButton_camera1_clicked();
    void on_pushButton_camera2_clicked();

    void on_pushBtn_rgboper_clicked();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QProcess *laser_cmd;
    QProcess* m_proces_bash;
    QProcess* proces_laser;
    QProcess* proces_hard;
    QProcess* proces_camera;
//    DialogConnectDevice *connectdev;
    //    CCtrlDashBoard* speed_x_dashBoard;
    //    CCtrlDashBoard* speed_y_dashBoard;

};

}  // namespace robot_hmi

#endif // robot_hmi_MAIN_WINDOW_H
