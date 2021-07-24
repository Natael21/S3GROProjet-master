#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QCloseEvent>
#include <QDebug>
#include <QtWidgets>
#include <QJsonObject>
#include <QJsonDocument>
#include <QSerialPortInfo>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts>
#include <QGraphicsView>

// Propres librairies
#include "csvwriter.h"
#include "serialprotocol.h"

// Classe definissant l'application
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    const qint32 BAUD_RATE = 115200;

    explicit MainWindow(int updateRate, QWidget *parent = nullptr);
    explicit MainWindow(QWidget *parent = nullptr);
    virtual ~MainWindow() override;
    void closeEvent(QCloseEvent *event) override;

    void sendMessage(QString msg);
    void setUpdateRate(int rateMs);

    void onPeriodicUpdate();
    void onMessageReceived(QString);

private slots:
    void receiveFromSerial(QString);
    void sendPulseSetting();
    void sendPulseStart();
    void manageRecording(int);
    void changeJsonKeyValue();
    void startSerialCom(QString);
    void addFormes();
    void sendPID();
    void sendStart();
    void sendStop();
    void sendPosition();
    void showPopUp();

private:
    void connectTimers(int updateRate);
    void connectButtons();
    void connectSerialPortRead();
    void connectSpinBoxes();
    void startRecording();
    void stopRecording();
    void connectTextInputs();
    void connectComboBox();
    void portCensus();

    bool record = false;
    CsvWriter* writer_;
    QTimer updateTimer_;
    QString msgReceived_{""};
    QString msgBuffer_{""};
    SerialProtocol* serialCom_=nullptr;

    QString JsonKey_;
    QLineSeries series_;
    QChart chart_;
    QGraphicsScene scene;
    QMovie *movie = new QMovie("../identification_Qt/image/ajax-loader.gif");
    QLabel *label;

    double distance_obstacle = 0;
    double distance_depot = 0;
    double positionVoiture = 75;
    double positionObstacle = 0;
    double longeurPendule = 40;
    double positionDepot = 0;
    double anglePendule = -30;
    double largeurRobot = 50;
    double hauteurRobot = 30;
    double diametreRoue = 12;

    bool sapinLacher = false;
    bool distance_envoyer = false;

protected:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
