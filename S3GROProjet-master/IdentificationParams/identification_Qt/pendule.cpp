#include "pendule.h"

#include <QDebug>

//double x2 = (tan(anglePendule*(3.1416/180)))*longeurPendule;
//double x1 = 40+lastposvoiture;
//double y1 = 346;
//double y2 = 350+longeurPendule;

PenduleItem::PenduleItem(double currentangle, double newangle,double currentpos, double newpos):
  pendule(new QGraphicsPixmapItem(QPixmap(":/image/pendule.png")))

{
    startanglePendule = currentangle;
    startpositionPendule = currentpos;
    //qDebug()<<"startpositionPendule"<< startpositionPendule;
    //qDebug()<<"newpositionPendule"<< newpositionPendule;
    pendule->setPos(currentpos-5,344);
    addToGroup(pendule);
    xAnimation = new QPropertyAnimation(this,"x",this);
    xAnimation->setStartValue(currentpos);//a changer
    xAnimation->setEndValue(newpos);//a changer
    xAnimation->setEasingCurve(QEasingCurve::Linear);
    xAnimation->setDuration(1100);//compter cmb de temps prend d'aller a une position à l'autre 1100

   connect(xAnimation,&QPropertyAnimation::finished,[=](){
        qDebug()<<"Animation position Pendule Fini";
        scene()->removeItem(this);
        delete this;
    });

    xAnimation->start();

    qAnimation = new QPropertyAnimation(this,"q",this);
    qAnimation->setStartValue(startanglePendule);//a changer
    qAnimation->setEndValue(newangle);//a changer
    qAnimation->setEasingCurve(QEasingCurve::InQuad);
    qAnimation->setDuration(1100);//compter cmb de temps prend d'aller a une position à l'autre 1100

    connect(qAnimation,&QPropertyAnimation::finished,[=](){
         qDebug()<<"Animation angle Pendule Fini";
         scene()->removeItem(this);
         delete this;
     });

     qAnimation->start();
}

PenduleItem::~PenduleItem()//tester si efface
{
    qDebug()<< "Pendule est mort";

}
qreal PenduleItem::q() {
    return m_q;
}

qreal PenduleItem::x()
{
    return m_x;
}

void PenduleItem::setQ(qreal q)
{
    m_q = q;
    qDebug() << "Pendule angle:"<< q;

    QPointF c = boundingRect().topRight();
    QTransform t;
    t.translate(c.x(), c.y());
    t.rotate(q);
    t.translate(-c.x(),-c.y());
    setTransform(t);
}

void PenduleItem::setX(qreal x)
{
    //qDebug() << "Pendule position:"<< x;
    moveBy(x-m_x,0);
    m_x = x;
}
