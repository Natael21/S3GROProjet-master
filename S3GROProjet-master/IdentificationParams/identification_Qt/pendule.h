#ifndef PENDULE_H
#define PENDULE_H


#include <QObject>
#include<QGraphicsItemGroup>
#include<QGraphicsPixmapItem>
#include<QGraphicsScene>
#include<QPropertyAnimation>


class PenduleItem :public QObject, public QGraphicsItemGroup
{
    Q_OBJECT
    Q_PROPERTY(qreal q READ getQ WRITE setQ)
    Q_PROPERTY(qreal x READ getX WRITE setX)


public:
    explicit PenduleItem(double angle,double currentpos);
    ~PenduleItem();
    qreal getQ();
    qreal getX();

signals:

public slots:
    void setQ(qreal q);
    void setX(qreal x);

private:

    qreal m_q;
    qreal m_x;
    QGraphicsPixmapItem * pendule;
//    QPropertyAnimation * xAnimation;
//    QPropertyAnimation * qAnimation;

};

#endif // PENDULE_H
