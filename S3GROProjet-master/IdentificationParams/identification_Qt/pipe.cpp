#include "pipe.h"

PipeItem::PipeItem() :
    pipe(new QGraphicsPixmapItem(QPixmap("../identification_Qt/image/pipe.png")))
{
    pipe->setPos(positionObstacle,395);
    addToGroup(pipe);
}


PipeItem::~PipeItem()
{

}
void PipeItem::receiveFromSerial(QString msg){ //il se peut que il y ait un prob de com avec le header mainwindow.h
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

        positionObstacle = jsonObj["position_obstacle"].toDouble();


        // Fonction de reception de message (vide pour l'instant)
        msgReceived_ = msgBuffer_;
        onMessageReceived(msgReceived_);

    // Reinitialisation du message tampon
    msgBuffer_ = "";


                                }
                                }
}

qreal PipeItem::x() {
       return positionObstacle;
}

void PipeItem::setX(qreal x)
{
    m_x = x;
}
void PipeItem::onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    // Decommenter la ligne suivante pour deverminage
    // qDebug().noquote() << "Message du Arduino: " << msg;
}
