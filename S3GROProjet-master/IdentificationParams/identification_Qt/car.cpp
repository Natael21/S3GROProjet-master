#include "car.h"
#include <QDebug>

CarItem::CarItem(double currentpos):
  vehicule(new QGraphicsPixmapItem(QPixmap(":/image/car.png")))

{
    vehicule->setPos(currentpos,270);
    addToGroup(vehicule);
//    xAnimation = new QPropertyAnimation(this,"x",this);
//    xAnimation->setStartValue(currentpos);//a changer
//    xAnimation->setEndValue(newpos);//a changer
//    xAnimation->setEasingCurve(QEasingCurve::Linear);
//    xAnimation->setDuration(1100);//compter cmb de temps prend d'aller a une position à l'autre 1100

//   connect(xAnimation,&QPropertyAnimation::finished,[=](){
//        //qDebug()<<"Animation Mario Fini";
//        scene()->removeItem(this);
//        delete this;
//    });

    //xAnimation->start();
}

CarItem::~CarItem()//tester si efface
{
   // qDebug()<< "Mario est mort";

}
qreal CarItem::getX() {
    return m_x;
}

void CarItem::setX(qreal x)
{
    qDebug() << "Mario position:"<< x;
    moveBy(x,0);
    m_x = x;
}
