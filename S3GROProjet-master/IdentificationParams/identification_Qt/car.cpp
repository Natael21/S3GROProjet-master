#include "car.h"


CarItem::CarItem(double position):
  vehicule(new QGraphicsPixmapItem(QPixmap(":/image/car.png")))

{
    positionVehicule = position;
    vehicule->setPos(position+5,270);
    addToGroup(vehicule);
    xAnimation = new QPropertyAnimation(this,"x",this);
    xAnimation->setStartValue(5);//a changer
    xAnimation->setEndValue(602);//a changer
    xAnimation->setDuration(1000);
    xAnimation->start();
}

CarItem::~CarItem()//tester si efface
{

}
qreal CarItem::x() {
    return positionVehicule;
}

void CarItem::setX(qreal x)
{
    positionVehicule = x;
}
