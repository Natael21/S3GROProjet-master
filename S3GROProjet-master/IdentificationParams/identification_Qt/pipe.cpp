#include "pipe.h"

PipeItem::PipeItem(double position,double hauteur) :
    pipe(new QGraphicsPixmapItem(QPixmap(":/image/pipe.png")))
{
    positionObstacle = position;
    pipe->setPos(position,395-hauteur);
    addToGroup(pipe);
}


PipeItem::~PipeItem()
{

}
