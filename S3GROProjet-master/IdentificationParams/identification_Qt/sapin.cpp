#include "sapin.h"

SapinItem::SapinItem(double angleSapin,double currentpos,double sapinLacher,double etat):
  sapin(new QGraphicsPixmapItem(QPixmap(":/image/sapin.png")))

{
    sapin->setPos(currentpos+15,365);

    sapin->setTransformOriginPoint(16.5,-28);
    sapin->setRotation(angleSapin);

    if(etat != 8)
    {
        addToGroup(sapin);
    }

    groundposition = scenePos().y()+395;

    /*connect(sapin,&yAnimation,[=](){

    groundposition = scenePos().y()+395;
    yAnimation = new QPropertyAnimation(this,"y",this);
    yAnimation->setStartValue(scenePos().y());
    yAnimation->setEndValue(groundposition);
    yAnimation->setEasingCurve(QEasingCurve::Linear);
    yAnimation->setDuration(1100);//compter cmb de temps prend d'aller a une position à l'autre 1100
    });
    if(sapinLacher == 1)
    {
            yAnimation->start();

    }
    if(sapinLacher == 0)
    {
       yAnimation->stop();
          connect(yAnimation,&QPropertyAnimation::finished,[=](){
               qDebug()<<"Animation Sapin Fini";
               scene()->removeItem(this);
               delete this;
          });
    }*/
}

    SapinItem::~SapinItem()//tester si efface
    {
        //qDebug()<< "Pendule est mort";
    }

    qreal SapinItem::getY() {
        return m_y;
    }

    void SapinItem::setY(qreal y)
    {
        //qDebug() << "Sapin position:"<< x;
        moveBy(0,y-m_y);
        m_y = y;
    }


