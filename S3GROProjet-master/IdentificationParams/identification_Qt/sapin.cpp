#include "sapin.h"

SapinItem::SapinItem(double angle,double currentpos,bool sapinlacher):
  sapin(new QGraphicsPixmapItem(QPixmap(":/image/sapin.png")))

{
    //x1 = sin(angle)*longeurPendule;
    //y1 = cos(angle)*longeurPendule;
    sapin->setPos(currentpos+15,365);

    QPointF offset = sapin->boundingRect().center();

    QTransform trans;

    trans.translate(offset.x(), offset.y());
    trans.rotate(angle);
    trans.translate(-offset.x(), -offset.y());
    sapin->setTransform(trans);

    addToGroup(sapin);

    groundposition = scenePos().y()+395;
    yAnimation = new QPropertyAnimation(this,"y",this);
    yAnimation->setStartValue(scenePos().y());//a changer
    yAnimation->setEndValue(groundposition);//a changer
    yAnimation->setEasingCurve(QEasingCurve::Linear);
    yAnimation->setDuration(1100);//compter cmb de temps prend d'aller a une position à l'autre 1100

    if(sapinlacher == 1)
    {
            yAnimation->start();

    }
    if(sapinlacher == 0)
    {
       yAnimation->stop();
    }

}

SapinItem::~SapinItem()
{

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

