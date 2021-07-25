#ifndef PIPE_H
#define PIPE_H

#include <QObject>

class pipe : public QObject
{
    Q_OBJECT
public:
    explicit pipe(QObject *parent = nullptr);

signals:

public slots:
};

#endif // PIPE_H