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
    void showGIF();

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

public:// pour avoir positionVoiture dans la classe car

    QGraphicsScene scene;
    QMovie *movie = new QMovie(":/image/WeDidIt.gif");
    QGraphicsPixmapItem * pixItem = new QGraphicsPixmapItem(QPixmap(":/image/sky.png"));
    QLabel *label;

    int sapin =                  0;

    double distance_obstacle =   0;
    double distance_depot =      0;
    double positionVoiture =     0;//peut peut etre enlever et garder dans la classe
    double positionObstacle =    350;//same
    double longeurPendule =      40;
    double positionDepot =       500;
    double anglePendule =        0;
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
