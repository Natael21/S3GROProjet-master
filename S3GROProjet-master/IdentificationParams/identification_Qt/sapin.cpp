#include "sapin.h"
/*
SapinItem::SapinItem():
  sapin(new QGraphicsPixmapItem(QPixmap(":/image/sapin.png")))

{
   sapin->setPos(5,270);
   addToGroup(sapin);
    xAnimation = new QPropertyAnimation(this,"x",this);
    xAnimation->setStartValue(5);//a changer
    xAnimation->setEndValue(225);//a changer
    xAnimation->setDuration(1000);
    xAnimation->start();
}

CarItem::~CarItem()//tester si efface
{

}
void CarItem::receiveFromSerial(QString msg){ //il se peut que il y ait un prob de com avec le header mainwindow.h
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

            anglePendule = jsonObj["cur_angle"].toDouble();


        // Fonction de reception de message (vide pour l'instant)
        msgReceived_ = msgBuffer_;
        onMessageReceived(msgReceived_);

    // Reinitialisation du message tampon
    msgBuffer_ = "";


                                }
                                }
}

qreal CarItem::x() {
       return positionVoiture;
}

void CarItem::setX(qreal x)
{
    m_x = x;
}
void CarItem::onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    // Decommenter la ligne suivante pour deverminage
    // qDebug().noquote() << "Message du Arduino: " << msg;
}

*/
