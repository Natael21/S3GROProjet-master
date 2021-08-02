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
    Q_PROPERTY(qreal q READ q WRITE setQ)
    Q_PROPERTY(qreal x READ x WRITE setX)


public:
    explicit PenduleItem(double currentangle, double newangle,double currentpos, double newpos);
    ~PenduleItem();
    qreal q();
    qreal x();

signals:

public slots:
    void setQ(qreal q);
    void setX(qreal x);

private:
    double startanglePendule;
    double newanglePendule;
    double startpositionPendule;
    double newpositionPendule;
    qreal m_q;
    qreal m_x;
    QGraphicsPixmapItem * pendule;
    QPropertyAnimation * xAnimation;
    QPropertyAnimation * qAnimation;

};

#endif // PENDULE_H
