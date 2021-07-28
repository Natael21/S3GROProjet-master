#include "flag.h"

FlagItem::FlagItem(double position):
    flag(new QGraphicsPixmapItem(QPixmap(":/image/flag.png")))
{
   flag->setPos(position-5,373);
   positionDepot = position;
   addToGroup(flag);
}

FlagItem::~FlagItem()
{

}

qreal FlagItem::x() {
       return positionDepot;
}

void FlagItem::setX(qreal x)
{
    m_x = x;
}

