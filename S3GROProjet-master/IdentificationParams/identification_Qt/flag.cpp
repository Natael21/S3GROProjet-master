#include "flag.h"

FlagItem::FlagItem():
    flag(new QGraphicsPixmapItem(QPixmap(":/image/flag.png")))
{
   flag->setPos(positionDepot-5,373);
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

