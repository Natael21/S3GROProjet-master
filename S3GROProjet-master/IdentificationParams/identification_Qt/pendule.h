#ifndef PENDULE_H
#define PENDULE_H


#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QGraphicsScene>
#include<QPropertyAnimation>
#include<QtDebug>
#include<QtMath>
#include<QTransform>

class PenduleItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal y READ getY WRITE setY)


public:
    explicit PenduleItem(double anglePendule,double angleSapin,double currentpos,double sapinLacher,double etat);
    ~PenduleItem();
    qreal getY();


signals:

public slots:
     void setY(qreal y);

private:
    qreal m_y;
    qreal groundposition;

    QGraphicsPixmapItem * pendule;
    QGraphicsPixmapItem * sapin;
    QPropertyAnimation * yAnimation;

};

#endif // PENDULE_H
