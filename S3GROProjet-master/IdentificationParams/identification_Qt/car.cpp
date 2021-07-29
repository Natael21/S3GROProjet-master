#include "car.h"
#include <QDebug>

CarItem::CarItem(double currentpos, double newpos):
  vehicule(new QGraphicsPixmapItem(QPixmap(":/image/car.png")))

{
    //setPixmap(pixmap);
    startpositionVehicule = currentpos;
    newpositionVehicule = newpos;
    qDebug()<<"startpositionVehicule"<< startpositionVehicule;
    qDebug()<<"newpositionVehicule"<< newpositionVehicule;
    vehicule->setPos(currentpos,270);
    addToGroup(vehicule);
    xAnimation = new QPropertyAnimation(this,"x",this);
    xAnimation->setStartValue(currentpos);//a changer
    xAnimation->setEndValue(newpos);//a changer
    xAnimation->setEasingCurve(QEasingCurve::Linear);
    xAnimation->setDuration(1000);//compter cmb de temps prend d'aller a une position à l'autre

   connect(xAnimation,&QPropertyAnimation::finished,[=](){
        qDebug()<<"Animation Mario Fini";
        scene()->removeItem(this);
        delete this;
    });

    xAnimation->start();
}

CarItem::~CarItem()//tester si efface
{
    qDebug()<< "Mario est mort";

}
qreal CarItem::x() {
    return m_x;
}

void CarItem::setX(qreal x)
{
    qDebug() << "Mario position:"<< x;
    moveBy(x-m_x,0);
    m_x = x;
}
