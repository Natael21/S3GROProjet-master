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
#include <QPixmap>
#include <QObject>
#include <QSound>
#include <QGraphicsPixmapItem>
#include <QPropertyAnimation>

// Propres librairies
#include "csvwriter.h"
#include "serialprotocol.h"
#include "scene.h"

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
    void sendPID();
    void sendStart();
    void sendStop();
    void sendPosition();

   // void setUpMarioTimer(double lastposvoiture,double positionVoiture);// ??

private:
    void connectTimers(int updateRate);
    void connectButtons();
    void connectSerialPortRead();
    void connectSpinBoxes();
    void startRecording();
    void stopRecording();
    void connectTextInputs();
    void connectComboBox();
    void addFormes();
    void portCensus();
    void setUpMarioTimer(double lastanglePendule,double anglePendule,double lastposvoiture,double positionVoiture,int updateRate);
    void showPopUp();
    void showGIF();

    bool record = false;
    CsvWriter* writer_;
    QTimer updateTimer_;
    QString msgReceived_{""};
    QString msgBuffer_{""};
    SerialProtocol* serialCom_=nullptr;

    QString JsonKey_;
    QLineSeries series_;
    QChart chart_;
    QGraphicsScene *scene;    //Création Scene
    QMovie *movie = new QMovie(":/image/WeDidIt.gif");
    QLabel *label;
    QTimer *marioTimer;


    int sapin =                  0;

    double distance_obstacle =   0;
    double distance_depot =      0;
    double lastposvoiture =      0;
    double positionVoiture =     602;//peut peut etre enlever et garder dans la classe
    double positionObstacle =    0;//same 350,-18 a 620
    double longeurPendule =      40;
    double positionDepot =       500;
    double lastanglePendule =    0;
    double anglePendule =        20;
    double largeurRobot =        50;
    double hauteurRobot =        10;
    double diametreRoue =        12;
    double afficher     =        0;

    bool casZero     =           false;
    bool sapinLacher =           false;
    bool distance_envoyer =      false;

protected:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
