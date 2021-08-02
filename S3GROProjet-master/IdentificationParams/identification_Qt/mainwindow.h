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

#include "car.h"
#include "pipe.h"
#include "flag.h"
#include "pendule.h"

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

    QColor colorRed = Qt::red;
    QColor colorBlack = Qt::black;
    QColor colorBlue = Qt::darkGray;
    QColor colorGreen = Qt::green;

    QBrush brushRed = Qt::SolidPattern;
    QBrush brushBlack = Qt::SolidPattern;
    QBrush brushBlue = Qt::SolidPattern;
    QBrush brushGreen = Qt::SolidPattern;

    QGraphicsPixmapItem * pixItem;

    QRectF rail;
    QRectF panierGauche;
    QRectF panierMillieu;
    QRectF panierDroite;


    CarItem * camion;
    PipeItem * pipe;
    FlagItem * flag;
    PenduleItem * pendule;

    int sapin =                  0;

    double distance_obstacle =   100;
    //double distance_depot =      500;
   // double lastposvoiture =      0;
    double positionVoiture =     0;//peut peut etre enlever et garder dans la classe max = 602
    double positionObstacle =    0;//same 350,-18 a 620
    double longeurPendule =      40;
    double positionDepot =       500;
    //double lastanglePendule =    0;
    double anglePendule =        -45;
    double largeurRobot =        50;
    double hauteurRobot =        10;
    double diametreRoue =        12;
    double afficher     =        0;
    double covertisseurMagique = (602/2);

    bool casZero     =           false;
    bool sapinLacher =           false;
    bool distance_envoyer =      false;

protected:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
