#ifndef PPIPLUGINEX_GLOBAL_H
#define PPIPLUGINEX_GLOBAL_H

#include <QMetaEnum>
#include <QPoint>

#include <functional>

//base type
namespace ppl{
Q_NAMESPACE

enum DisplayRangeType
{
    SetupRange,
    SpokeRange,
};
Q_ENUM_NS(DisplayRangeType)

enum PointType
{
    RectCoord,
    PolarCoord,
};
Q_ENUM_NS(PointType)

enum BaseLayer
{
    PixelLayer,
    RealLayer,
};
Q_ENUM_NS(BaseLayer)

enum MotionType
{
    Relative,
    Absolute,

    MotionCount
};
Q_ENUM_NS(MotionType)

enum ReferencePoint
{
    ScreenPoint,
    OwnShip,
    FixedPoint,
};
Q_ENUM_NS(ReferencePoint)

enum UnitType
{
    UnitMetre,
    UnitNmile
};
Q_ENUM_NS(UnitType)

enum HeadingType{
    HeadingNone,
    HeadingShip,
    HeadingCourse,
    HeadingNorth,
};
Q_ENUM_NS(HeadingType)

struct DragPointF{
    QPointF begin;
    QPointF end;
    bool    move;
};

using PointProcessor = std::function<void(const QList<DragPointF>&)>;

}

#endif // PPIPLUGINEX_GLOBAL_H
