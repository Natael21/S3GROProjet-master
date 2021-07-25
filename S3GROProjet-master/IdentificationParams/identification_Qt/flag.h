#ifndef FLAG_H
#define FLAG_H

#include <QObject>

class flag : public QObject
{
    Q_OBJECT
public:
    explicit flag(QObject *parent = nullptr);

signals:

public slots:
};

#endif // FLAG_H