#ifndef PIPE_H
#define PIPE_H

#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QPropertyAnimation>
#include <QJsonObject>
#include <QJsonDocument>

class PipeItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT

public:
    explicit PipeItem(double position,double hauteur);
    ~PipeItem();

private:
    double positionObstacle;
    QGraphicsPixmapItem * pipe;
};

#endif // PIPE_H
