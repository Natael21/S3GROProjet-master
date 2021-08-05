#include "pendule.h"

#include <QDebug>

PenduleItem::PenduleItem(double anglePendule,double angleSapin,double currentpos,double sapinLacher,double etat):
  pendule(new QGraphicsPixmapItem(QPixmap(":/image/pendule.png"))),
  sapin(new QGraphicsPixmapItem(QPixmap(":/image/sapin.png")))

{
    pendule->setPos(currentpos-5,345);
    sapin->setPos(currentpos+15,365);

    sapin->setTransformOriginPoint(currentpos,-10);
    sapin->setRotation(angleSapin);
    QPointF offsetPendule = pendule->boundingRect().topRight();
   // QPointF offsetSapin= sapin->boundingRect().center();
    QTransform transPendule;

    transPendule.translate(offsetPendule .x(), offsetPendule .y());
    transPendule.rotate(anglePendule);
    transPendule.translate(-offsetPendule .x(), -offsetPendule .y());

    //QTransform transSapin;

   // transSapin.translate(offsetSapin.x(), offsetSapin.y());
    //transSapin.rotate(angleSapin);
  //  transSapin.translate(-offsetSapin.x(), -offsetSapin.y());


    pendule->setTransform(transPendule);
    //sapin->setTransform(transSapin);

    addToGroup(pendule);

    if(etat != 8)
    {
        addToGroup(sapin);
    }

    groundposition = scenePos().y()+395;

   /* connect(sapin,&yAnimation,[=](){

    groundposition = scenePos().y()+395;
    yAnimation = new QPropertyAnimation(this,"y",this);
    yAnimation->setStartValue(scenePos().y());
    yAnimation->setEndValue(groundposition);
    yAnimation->setEasingCurve(QEasingCurve::Linear);
    yAnimation->setDuration(1100);//compter cmb de temps prend d'aller a une position à l'autre 1100
    });
    if(sapinLacher == 1)
    {
            //yAnimation->start();

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

    PenduleItem::~PenduleItem()//tester si efface
    {
        //qDebug()<< "Pendule est mort";
    }

    qreal PenduleItem::getY() {
        return m_y;
    }

    void PenduleItem::setY(qreal y)
    {
        //qDebug() << "Sapin position:"<< x;
        moveBy(0,y-m_y);
        m_y = y;
    }

