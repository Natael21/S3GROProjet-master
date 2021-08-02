#ifndef CAR_H
#define CAR_H

#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QGraphicsScene>
#include<QPropertyAnimation>

class CarItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal x READ getX WRITE setX)

public:
    explicit CarItem(double currentpos);
    ~CarItem();
    qreal getX();

signals:

public slots:
     void setX(qreal x);
private:
    qreal m_x;
    QGraphicsPixmapItem * vehicule;
//    QPropertyAnimation * xAnimation;

};

#endif // CAR_H
