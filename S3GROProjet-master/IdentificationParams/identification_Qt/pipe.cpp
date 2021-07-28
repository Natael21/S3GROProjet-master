#include "pipe.h"

PipeItem::PipeItem(double position) :
    pipe(new QGraphicsPixmapItem(QPixmap(":/image/pipe.png")))
{
    positionObstacle = position;
    pipe->setPos(position,395);
    addToGroup(pipe);
}


PipeItem::~PipeItem()
{

}
qreal PipeItem::x() {
       return positionObstacle;
}

void PipeItem::setX(qreal x)
{
    positionObstacle = x;
}
