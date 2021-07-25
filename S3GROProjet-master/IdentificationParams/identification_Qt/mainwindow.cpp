#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "car.h"
#include "pipe.h"
#include "flag.h"

MainWindow::MainWindow(int updateRate, QWidget *parent):
    QMainWindow(parent)
{
    // Constructeur de la classe
    // Initialisation du UI
    ui = new Ui::MainWindow;
    ui->setupUi(this);


    // Initialisation du graphique
    ui->graph->setChart(&chart_);
    chart_.setTitle("Donnees brutes");
    chart_.legend()->hide();
    chart_.addSeries(&series_);

    //Permet de mettre de la couleur sur les boutons
    ui->Stop->setStyleSheet("QPushButton { background-color: red; }\n"
                          "QPushButton:enabled { background-color: rgb(200,0,0); }\n");

    ui->StartButton->setStyleSheet("QPushButton { background-color: green; }\n"
                          "QPushButton:enabled { background-color: rgb(0,255,0); }\n");

    // Fonctions de connections events/slots
    connectTimers(updateRate);
    connectButtons();
    connectSpinBoxes();
    connectTextInputs();
    connectComboBox();
    addFormes();
    //showGIF();

    // Recensement des ports
    portCensus();

    // initialisation du timer
    updateTimer_.start();

}

MainWindow::~MainWindow(){
    // Destructeur de la classe
    updateTimer_.stop();
    if(serialCom_!=nullptr){
      delete serialCom_;
    }
    delete ui;
}

void MainWindow::showPopUp()
{
    QMessageBox msg;
    msg.setText("Tu dois initialiser la position de l'obstacle et du panier dans la deuxième page");
    msg.exec();
}

void MainWindow::showGIF()
{
    QSound ohYeah("../identification_Qt/sound/WeDidIt.wav");
    ohYeah.play();

    QMessageBox msg;
        //msg.setText("This closes in 10 seconds");

        int cnt = 10;

        QTimer cntDown;
        QObject::connect(&cntDown, &QTimer::timeout, [&msg,&cnt, &cntDown]()->void{
                             if(--cnt < 0){
                                 cntDown.stop();
                                 msg.close();
                             } else {
                                 msg.setWindowTitle("AMAZING");
                                 msg.setStandardButtons(nullptr);
                             }
                         });

            // create Label
            msg.setIconPixmap(QPixmap("../identification_Qt/image/WeDidIt.gif"));
            QLabel *icon_label = msg.findChild<QLabel *>("qt_msgboxex_icon_label");
            icon_label->setFixedWidth(1000);
            icon_label->setFixedHeight(600);
            icon_label->setMovie(movie);
            movie->setScaledSize(QSize(1000, 600));
            movie->start();

            cntDown.start(300);
            msg.exec();
}

void MainWindow::closeEvent(QCloseEvent *event){
    // Fonction appelee lorsque la fenetre est detruite
    event->accept();
}

void MainWindow::receiveFromSerial(QString msg){
    // Fonction appelee lors de reception sur port serie
    // Accumulation des morceaux de message
    msgBuffer_ += msg;

    //Si un message est termine
    if(msgBuffer_.endsWith('\n')){
        // Passage ASCII vers structure Json
        QJsonDocument jsonResponse = QJsonDocument::fromJson(msgBuffer_.toUtf8());

        // Analyse du message Json
        if(~jsonResponse.isEmpty()){
            QJsonObject jsonObj = jsonResponse.object();
            QString buff = jsonResponse.toJson(QJsonDocument::Indented);

            // Affichage des messages Json
            ui->textBrowser->setText(buff.mid(2,buff.length()-4));
            //ui->Etat->setText(jsonObj["Etat"].toString());

            positionVoiture = jsonObj["cur_pos"].toDouble();
            positionObstacle = jsonObj["position_obstacle"].toDouble();
            positionDepot = jsonObj["position_depot"].toDouble();
            anglePendule = jsonObj["cur_angle"].toDouble();
            sapinLacher = jsonObj["sapin_lacher"].toBool();
            casZero     = jsonObj["casZero"].toBool();

            // Affichage des donnees dans le graph
            if(jsonObj.contains(JsonKey_)){
                double time = jsonObj["time"].toDouble();
                series_.append(time, jsonObj[JsonKey_].toDouble());
                // Mise en forme du graphique (non optimal)
                chart_.removeSeries(&series_);
                chart_.addSeries(&series_);
                chart_.createDefaultAxes();
            }

            // Fonction de reception de message (vide pour l'instant)
            msgReceived_ = msgBuffer_;
            onMessageReceived(msgReceived_);

            // Si les donnees doivent etre enregistrees
            if(record){
                writer_->write(jsonObj);
            }
        }
        // Reinitialisation du message tampon
        msgBuffer_ = "";
    }

    //Section servant de loop----------------------------------------
    if(casZero)
    {
        afficher = 0;
    }

    if(sapinLacher && afficher != 1)
    {
       showGIF();
       afficher = 1;
    }

    addFormes();
}

void MainWindow::connectTimers(int updateRate){
    // Fonction de connection de timers
    connect(&updateTimer_, &QTimer::timeout, this, [this]{onPeriodicUpdate();});
    updateTimer_.start(updateRate);
}

void MainWindow::connectSerialPortRead(){
    // Fonction de connection au message de la classe (serialProtocol)
    connect(serialCom_, SIGNAL(newMessage(QString)), this, SLOT(receiveFromSerial(QString)));
}

void MainWindow::connectButtons(){
    // Fonction de connection du boutton Send
    connect(ui->pulseButton, SIGNAL(clicked()), this, SLOT(sendPulseStart()));
    connect(ui->checkBox, SIGNAL(stateChanged(int)), this, SLOT(manageRecording(int)));
    connect(ui->pushButton_Params, SIGNAL(clicked()), this, SLOT(sendPID()));
    connect(ui->StartButton, SIGNAL(clicked()), this, SLOT(sendStart()));
    connect(ui->Stop, SIGNAL(clicked()), this, SLOT(sendStop()));
    connect(ui->envoiDistance, SIGNAL(clicked()), this, SLOT(sendPosition()));
}

void MainWindow::sendPosition()
{
    distance_obstacle = ui->distanceObstacle->text().toDouble();
    distance_depot = ui->distanceDepot->text().toDouble();

    QJsonArray array = { QString::number(distance_obstacle, 'f', 0),
                         QString::number(distance_depot, 'f', 0)
                       };

    QJsonObject jsonObject
    {
        {"Distance", array}
    };

    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);

    distance_envoyer = true;
}

void MainWindow::connectSpinBoxes(){
    // Fonction de connection des spin boxes
    connect(ui->DurationBox, SIGNAL(valueChanged(int)), this, SLOT(sendPulseSetting()));
    connect(ui->PWMBox, SIGNAL(valueChanged(double)), this, SLOT(sendPulseSetting()));
}

void MainWindow::connectTextInputs(){
    // Fonction de connection des entrees de texte
    connect(ui->JsonKey, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->JsonKey->text();
}

void MainWindow::connectComboBox(){
    // Fonction de connection des entrees deroulantes
    connect(ui->comboBoxPort, SIGNAL(activated(QString)), this, SLOT(startSerialCom(QString)));
}

void MainWindow::portCensus(){
    // Fonction pour recenser les ports disponibles
    ui->comboBoxPort->clear();
    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts()) {
        ui->comboBoxPort->addItem(port.portName());
    }
}

void MainWindow::startSerialCom(QString portName){
    // Fonction SLOT pour demarrer la communication serielle
    qDebug().noquote() << "Connection au port"<< portName;
    if(serialCom_!=nullptr){
        delete serialCom_;
    }
    serialCom_ = new SerialProtocol(portName, BAUD_RATE);
    connectSerialPortRead();
}

void MainWindow::addFormes()
{
    //Changer le ratio pour la longeur du pendule et l'angle et la position de la voiture/objets/etc.----------------------------------------
    //Mettre la scene vide
    scene.clear();
   // scene.setSceneRect(15,190,700,300);
    scene.addItem(pixItem);

    //qleft : positif (-> droite); negatif (<- gauche)
    //qtop : positif (bas); negatif(haut)

    QColor colorRed = Qt::red;
    QColor colorBlack = Qt::black;
    QColor colorBlue = Qt::darkGray;
    QColor colorGreen = Qt::green;

    QBrush brushRed = Qt::SolidPattern;
    QBrush brushBlack = Qt::SolidPattern;
    QBrush brushBlue = Qt::SolidPattern;
    QBrush brushGreen = Qt::SolidPattern;

    brushRed.setColor(colorRed);
    brushBlack.setColor(colorBlack);
    brushBlue.setColor(colorBlue);
    brushGreen.setColor(colorGreen);

    //Rail qui donne le point initiale des autres formes
    scene.addRect(QRectF(5,350, 720, 7), colorBlack);

    //Voiture
    //QRectF rectVoiture1 = QRectF(positionVoiture+15, 250, largeurRobot, hauteurRobot);
    //QRectF rectVoiture2 = QRectF(positionVoiture+15, 250, largeurRobot/2, hauteurRobot+10);
   // scene.addRect(rectVoiture1, colorRed, brushRed);
    //scene.addRect(rectVoiture2, colorRed, brushRed);
    CarItem * camion = new CarItem();

    scene.addItem(camion);

    //Roue voiture
    //QRectF ellipseRoue1 = QRectF(positionVoiture, -12, diametreRoue, diametreRoue);
   // QRectF ellipseRoue2 = QRectF(positionVoiture+largeurRobot, -12, diametreRoue, diametreRoue);
   // scene.addEllipse(ellipseRoue1, colorBlack, brushBlack);
   // scene.addEllipse(ellipseRoue2, colorBlack, brushBlack);

    //Pendule
    //addLine(x1,y1,x2,y2) y2 = longeur pendule
    double x2 = 10+(tan(anglePendule*(3.1416/180)))*longeurPendule;
    double x1 = 80+positionVoiture;
    double y1 = 345;
    double y2 = 350+longeurPendule;

    QLine pendule = QLine(x1, y1, x2+x1, y2);
    scene.addLine(pendule, colorRed);

    //Sapin
    if(!sapinLacher)
    {
        QRectF sapin = QRectF(x2+(x1-5), y2, 10, 10);
        scene.addRect(sapin, colorGreen, brushGreen);
    }
    else
    {
        sapin++;
    }

    if(sapin == 1)
    {
        QRectF sapin1 = QRectF(positionDepot+23, 75, 10, 10);
        scene.addRect(sapin1, colorGreen, brushGreen);
        qDebug() << "sapin == 1";
    }
    else if (sapin == 2)
    {
        QRectF sapin1 = QRectF(positionDepot+23, 75, 10, 10);
        QRectF sapin2 = QRectF(positionDepot+8, 75, 10, 10);
        scene.addRect(sapin1, colorGreen, brushGreen);
        scene.addRect(sapin2, colorBlack, brushGreen);
    }
    else if (sapin == 3)
    {
        QRectF sapin1 = QRectF(positionDepot+23, 75, 10, 10);
        QRectF sapin2 = QRectF(positionDepot+8, 75, 10, 10);
        QRectF sapin3 = QRectF(positionDepot+23, 63, 10, 10);
        scene.addRect(sapin1, colorGreen, brushGreen);
        scene.addRect(sapin2, colorBlack, brushGreen);
        scene.addRect(sapin3, colorBlue, brushGreen);
    }
    else if (sapin == 4)
    {
        QRectF sapin1 = QRectF(positionDepot+23, 435, 10, 10);
        QRectF sapin2 = QRectF(positionDepot+8, 435, 10, 10);
        QRectF sapin3 = QRectF(positionDepot+23, 425, 10, 10);
        QRectF sapin4 = QRectF(positionDepot+8, 425, 10, 10);
        scene.addRect(sapin1, colorGreen, brushGreen);
        scene.addRect(sapin2, colorBlack, brushGreen);
        scene.addRect(sapin3, colorBlue, brushGreen);
        scene.addRect(sapin4, colorRed, brushGreen);

    }

    //Obstacle
    //QRectF obstacle = QRectF(positionObstacle, 50, 10, 40);
    //scene.addRect(obstacle, colorBlack, brushBlack);

    PipeItem * pipe = new PipeItem();

    scene.addItem(pipe);

    //Panier
    QRectF panierGauche = QRectF(positionDepot, 425, 5, 20);
    QRectF panierMillieu = QRectF(positionDepot, 445, 40, 5);
    QRectF panierDroite = QRectF(positionDepot+35, 425, 5, 20);
    scene.addRect(panierGauche, colorBlue, brushBlue);
    scene.addRect(panierMillieu, colorBlue, brushBlue);
    scene.addRect(panierDroite, colorBlue, brushBlue);

    FlagItem * flag = new FlagItem();

    scene.addItem(flag);

    //Ajout des forme dans le graphique
    //scene.setBackgroundBrush(Qt::white);
    ui->Graphique->setScene(&scene);
}

void MainWindow::changeJsonKeyValue(){
    // Fonction SLOT pour changer la valeur de la cle Json
    series_.clear();
    JsonKey_ = ui->JsonKey->text();
}

void MainWindow::sendPID(){
    // Fonction SLOT pour envoyer les paramettres de pulse
    double goal = ui->lineEdit_DesVal->text().toDouble();
    double Kp = ui->lineEdit_Kp->text().toDouble();
    double Ki = ui->lineEdit_Ki->text().toDouble();
    double Kd = ui->lineEdit_Kd->text().toDouble();
    double thresh = ui->lineEdit_Thresh->text().toDouble();
    // pour minimiser le nombre de decimales( QString::number)

    QJsonArray array = { QString::number(Kp, 'f', 2),
                         QString::number(Ki, 'f', 2),
                         QString::number(Kd, 'f', 2),
                         QString::number(thresh, 'f', 2),
                         QString::number(goal, 'f', 2)
                       };
    QJsonObject jsonObject
    {
        {"setGoal", array}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendStart(){
    if(distance_envoyer)
    {
        QJsonObject jsonObject
        {
             {"Start", 0}
        };

        QJsonDocument doc(jsonObject);
        QString strJson(doc.toJson(QJsonDocument::Compact));
        sendMessage(strJson);
    }
    else
    {
        showPopUp();
    }
}

void MainWindow::sendStop()
{
    QJsonObject jsonObject
    {
        {"Stop", 100}
    };

    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendPulseSetting(){
    // Fonction SLOT pour envoyer les paramettres de pulse
    double PWM_val = ui->PWMBox->value();
    int duration_val = ui->DurationBox->value();
    QJsonObject jsonObject
    {// pour minimiser le nombre de decimales( QString::number)
        {"pulsePWM", QString::number(PWM_val)},
        {"pulseTime", duration_val}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendPulseStart(){
    // Fonction SLOT pour envoyer la commande de pulse
    QJsonObject jsonObject
    {
        {"pulse", 1}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}

void MainWindow::sendMessage(QString msg){
    // Fonction SLOT d'ecriture sur le port serie
    if(serialCom_==nullptr){
        qDebug().noquote() <<"Erreur aucun port serie !!!";
        return;
    }
    serialCom_->sendMessage(msg);
    qDebug().noquote() <<"Message du RPI: "  <<msg;
}

void MainWindow::setUpdateRate(int rateMs){
    // Fonction d'initialisation du chronometre
    updateTimer_.start(rateMs);
}

void MainWindow::manageRecording(int stateButton){
    // Fonction SLOT pour determiner l'etat du bouton d'enregistrement
    if(stateButton == 2){
        startRecording();
    }
    if(stateButton == 0){
        stopRecording();
    }
}

void MainWindow::startRecording(){
    // Fonction SLOT pour creation d'un nouveau fichier csv
    record = true;
    writer_ = new CsvWriter("/home/pi/Desktop/");
    ui->label_pathCSV->setText(writer_->folder+writer_->filename);
}

void MainWindow::stopRecording(){
    // Fonction permettant d'arreter l'ecriture du CSV
    record = false;
    delete writer_;
}
void MainWindow::onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    // Decommenter la ligne suivante pour deverminage
    // qDebug().noquote() << "Message du Arduino: " << msg;
}

void MainWindow::onPeriodicUpdate(){
    // Fonction SLOT appelee a intervalle definie dans le constructeur
    qDebug().noquote() << "*";
}
