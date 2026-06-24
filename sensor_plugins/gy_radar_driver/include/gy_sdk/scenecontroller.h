#ifndef SCENECONTROLLER_H
#define SCENECONTROLLER_H

#include "ppiclientdef.h"
#include <QGraphicsScene>
#include <ExportDll.h>

namespace NaviRadar{
namespace Spoke{
struct SPOKE;
}
}
namespace NaviRadar{namespace Target{
struct tTrackedTarget;
}}

namespace NaviRadar{namespace PPI{
enum eAzimuthInterPolation : int;
}}

namespace NaviRadar{
enum eCOLOR_TYPE :int;
}

enum eElectronMouseOperateMode {
    eNullOperate,
    eAddDelete,
    eSelectModify,
};

enum ePositionIndicatorMode {
    ePickup,
    eDrawElectronArea,
};

class IBaseComponents;

class SceneControllerPrivate;
class DLL_API SceneController : public QGraphicsScene
{
    Q_OBJECT
    Q_DISABLE_COPY(SceneController)
    Q_DECLARE_PRIVATE(SceneController)
public:
    explicit SceneController(QWidget *viewport,QWidget *parent = nullptr);
    virtual ~SceneController();
    QPoint pos() const;
    const QImage& getImage() const;
Q_SIGNALS://Export SIGNALS
    void positionIndicatorEnableChanged(bool state);
    void positionIndicatorModeChanged(ePositionIndicatorMode mode);
    //radius
    void pixelRadiusChanged(qreal radius_px);
    void realRadiusChanged(qreal radius_me);

    void pickup_fps();
    void pickup_rpm();
    void pickup_guardZone(int id);
    void pickup_target(int id);
    void pickup_range();
    void pickup_unit();
    void pickup_polarPos(const QPointF& polarPos);
    void pickup_geoPos(const QPointF& geoPos);

    void EblVrmPositionChanged(int id,const QPointF& polarPos);
    void EblVrmCentreChanged(int id,const QPointF& polarPos);

    void GuardZoneRangeChanged(int id,qreal value_me);
    void GuardZoneDepthChanged(int id,qreal value_me);
    void GuardZoneAngleChanged(int id,qreal value_deg);
    void GuardZoneWidthChanged(int id,qreal value_deg);

    void CameraViewRangeChanged(int id,qreal value_me);
    void CameraViewDepthChanged(int id,qreal value_me);
    void CameraViewAngleChanged(int id,qreal value_deg);
    void CameraViewWidthChanged(int id,qreal value_deg);

    void ElectronFenceEnableChanged(int id, bool enable);
    void ElectronFencePositionsChanged(int id,const QList<QPointF>& positions);
    void ElectronSilentAreaEnableChanged(int id, bool enable);
    void ElectronSilentAreaPositionsChanged(int id,const QList<QPointF>& positions);

    void EccentricReseted();
//private signal
    void clearImageData(QImage* img);
public Q_SLOTS://Different part setting
    qreal getPixelRadius() const;
    qreal getVisualRange() const;//metres
    //Range and Unit Part
    void setDisplayRangeType(ppl::DisplayRangeType type);
    void setUnitRangeEnable(bool state);
    void setVisualRange(qreal range_metres);//with unit metres
    void setUnitType(ppl::UnitType type);
    ppl::UnitType getUnitType() const;
    qreal rangeUp();
    qreal rangeDown();
    //translate range of localType to given type
    qreal rangeTo(qreal range,ppl::UnitType toType) const;
    //translate range of givenType to local type
    qreal rangeFrom(qreal range,ppl::UnitType fromType) const;

    //Radar image quality Part
    void setRadarImageEnable(bool state);
    void setRadarImageInterpotionType(NaviRadar::PPI::eAzimuthInterPolation type);
    void setRadarImageThreshold(int threshold);
    void setRadarImageColorType(NaviRadar::eCOLOR_TYPE type);
    void clearRadarImage();
    //Coord Axes Part
    void setCoordAxesEnable(bool state);
    void setCoordAxesColor(const QColor &color);
    void setCoordAxesLine(int radiusCircle,int divideLine);
    //Tail Part
    void setTailTime(int time_secs);//if time < 0 means infinite
    void setTailFlush();
    //FPS Part
    void setFps(int fps);
    void setFpsShowing(bool state);
    //Rpm Part
    void setRpmShowing(bool state);
    //EblVrm Part
    void setEblVrmCount(int count);
    void setEblEnable(int id,bool state);
    void setVrmEnable(int id,bool state);
    void setEblVrmEnable(int id,bool state);
    void setEblPosition(int id,qreal angle);
    void setEblPosition(int id,const QPointF &screenPos);//screen pos
    void setVrmPosition(int id,qreal distance);
    void setVrmPosition(int id,const QPointF &screenPos);//screen pos
    void setEblVrmPosition(int id,qreal distance,qreal angle);
    void setEblVrmPosition(int id,const QPointF &screenPos);//screen pos
    void setEblVrmCentre(int id,const QPointF &screenPos);//screen pos
    void resetCentre(int id);
    void setEblVrmStyle(int id, const QColor &color, int width);
    //GuardZone Part
    void setGuardZoneCount(int count);
    void setGuardZoneEnable(int id,bool state);
    void setGuardZonePickupEnable(int id,bool state);
    void setGuardZone(int id, qreal range, qreal depth, qreal angle, qreal width);//metre and degree
    void setGuardZoneRange(int id,qreal value_me);
    void setGuardZoneDepth(int id,qreal value_me);
    void setGuardZoneAngle(int id,qreal value_deg);
    void setGuardZoneWidth(int id,qreal value_deg);
    void setGuardZoneDrag(int id, const QPointF &p1,const QPointF &p2);
    void getGuardZoneParam(const QPointF &p1,const QPointF &p2,qreal& range,qreal& depth,qreal& angle,qreal& width);
    void setGuardZoneStyle(int id,const QColor& color);
    //cameraview part
    void setCameraViewCount(int count);
    void setCameraViewEnable(bool state);
    void setCameraViewAngle(qreal value_deg);
    void setCameraViewDepth(qreal value_m);
    void setCameraViewStyle(const QColor& color);
    void setCameraView(qreal range, qreal depth, qreal angle, qreal width);
    //ElectronFence part
    void setElectronFenceCount(int count);
    void setElectronFenceActiveID(int id);
    void setElectronFenceMouseOperateMode(eElectronMouseOperateMode mode);
    void addElectronFenceMousePoint();
    void delElectronFenceLastPoint();
    void setElectronFencePointSelected();
    void setElectronFencePointModifyConfirmed();
    void setElectronFencePointModifyCanceled();
    bool setElectronFenceEnable(int id, bool state);
    bool getElectronFenceEnable(int id);
    void setElectronFencePickUpEnable(int id, bool state);
    void addElectronFencePosition(int id, qreal longitude, qreal latitude);
    void delElectronFenceLastPosition(int id);
    bool setElectronFencePosition(int id, int index, qreal longitude, qreal latitude);
    void setElectronFencePositions(int id, const QList<QPointF>& positions);
    void setElectronFenceClear(int id);
    QList<QPointF> ElectronFenceListPosition(int id);
    void setElectronFenceStyle(int id,const QColor& color);
    void setElectronFenceLineColor(int id,const QColor& color);
    //ElectronSilentArea
    void setElectronSilentAreaCount(int count);
    void setElectronSilentAreaActiveID(int id);
    void setElectronSilentAreaMouseOperateMode(eElectronMouseOperateMode mode);
    void addElectronSilentAreaMousePoint();
    void delElectronSilentAreaLastPoint();
    void setElectronSilentAreaPointSelected();
    void setElectronSilentAreaPointModifyConfirmed();
    void setElectronSilentAreaPointModifyCanceled();
    bool setElectronSilentAreaEnable(int id, bool state);
    bool getElectronSilentAreaEnable(int id);
    void setElectronSilentAreaPickUpEnable(int id, bool state);
    void addElectronSilentAreaPosition(int id, qreal longitude, qreal latitude);
    void delElectronSilentAreaLastPosition(int id);
    bool setElectronSilentAreaPosition(int id, int index, qreal longitude, qreal latitude);
    void setElectronSilentAreaPositions(int id, const QList<QPointF>& positions);
    void setElectronSilentAreaClear(int id);
    QList<QPointF> ElectronSilentAreaListPosition(int id);
    void setElectronSilentAreaStyle(int id,const QColor& color);
    void setElectronSilentAreaLineColor(int id,const QColor& color);

    //TrackedTarget Part
    void setTrackedTargetCount(int count);
    void setTrackedTargetFlush();
    void setTrackedTargetSelected(int id);
    void setTrackedTargetStyleOfState(quint32 state,const QImage& image,bool flashing);
    void setTrackedTargetStyleOfSelected(const QImage& image);
    QPointF getTrackedTargetPositionFromScreenPos(const QPointF& pos);//return polar(metre,degree)
    //Target Trail Part
    void setTrackedTargetTrailEnable(bool state);
    void setTrackedTargetTrailStyle(const QColor &color);
    void setTrackedTargetTrailFlush();
    void setTrackedTargetTrailMaxCount(int count);

    //Own Ship Part
    void setOwnShipEnable(bool state);
    /// @deprecated
    void setOwnShipStyle(const QColor &clr, bool filled,const QSizeF &size_metres, const QPointF &anchor_metres);
    void setOwnShipShowRange(qreal showRange_metres);
    void setOwnShipHeadingLineEnable(bool state);
    void setOwnShipBeamLineEnable(bool state);
    void setOwnShipSternLineEnable(bool state);
    void setOwnShipCourseLineEnable(bool state);
    void setOwnShipNorthLineEnable(bool state);
    void setOwnShipHeadingLineStyle(const QColor& color,int width,qreal lenth);// here @param lenth is scale base of visual radius
    void setOwnShipBeamLineStyle(const QColor& color,int width,qreal lenth);
    void setOwnShipSternLineStyle(const QColor& color,int width,qreal lenth);
    void setOwnShipCourseLineStyle(const QColor& color,int width,qreal lenth);
    void setOwnShipNorthLineStyle(const QColor& color,int width,qreal lenth);
    //Parallel Index Line Part
    void setParallelIndexLineCount(int count);
    void setParallelIndexLineEnable(int id,bool state);
    void setParallelIndexLine(int id, qreal dist_me, qreal angle);
    //return <distant,angle>
    QPointF getParallelIndexLineByDrag(const QPointF &p1,const QPointF &p2);
    void setParallelIndexLineStyle(const QPen& pen);
    //RadarState Text Part
    void setRadarState(quint32 state);
    void setRadarReplayState(bool state);
    void setRadarStateText(quint32 state,const QString& text);
    void setRadarStateStyle(const QColor &color);
    //Position indicator Part
    void setPositionIndicatorEnable(bool state);
    void setPositionIndicatorMode(ePositionIndicatorMode mode);
    void setPositionInfoEnable(bool state);
    void setPositionIndicatorCursorPress  (const QPointF &pos);
    void setPositionIndicatorCursorMove   (const QPointF &pos);
    void setPositionIndicatorCursorRelease(const QPointF &pos);
    void setPositionIndicatorStyle(const QColor &crossColor,const QColor &cursorColor);
    QPointF getPositionIndicator();

public Q_SLOTS://Different part non-visible setting
    //Eccentric part
    void setEccentricReset();
    void setEccentricMaxEdge(qreal range_percent);
    void setEccentricResetPosition(qreal range_percent,qreal angle_degree)
    { setEccentricResetRange(range_percent);setEccentricResetAngle(angle_degree); }
    void setEccentricResetRange(qreal range_percent);
    void setEccentricResetAngle(qreal angle_degree);
    void setEccentricPosition(qreal range_percent, qreal angle_degree);//effective only when MotionRelative
    void setEccentricPosition(const QPointF &pos);//the pos is pixel cartesian coord

    void setRequestOfPointCollector(ppl::PointProcessor caller, int maxPoint, bool singleShot = true, ppl::PointProcessor interCall = nullptr);

    //pickup
    void pickup(const QPointF &screenPos);
    void setPickupEnable(bool state);
    void setPickupWidget(QWidget* widget, const QObject *receiver, const char *member);
    void setPickupWidget(QWidget *widget, std::function<void()> functor);
    //Heading set
    void setHeadingType(ppl::HeadingType type);
    void setMotionType(ppl::MotionType type);
public://global interface
    //spoke
    void updateSpoke(const NaviRadar::Spoke::SPOKE *p);
    //target
    void updateTarget(const NaviRadar::Target::tTrackedTarget* p);
    //geo
    void updateGeoPosition(bool enable,qreal longitude,qreal latitude,qreal altitude);
    void updateGeoSpeed(qreal speed_kmh,qreal course);//based on north
    void updateGeoHeading(qreal angle);//based on north
    //ais TODO...
protected:
    //void wheelEvent(QGraphicsSceneWheelEvent *event) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void focusOutEvent(QFocusEvent *) override;
protected:
    QScopedPointer<SceneControllerPrivate> d_ptr;
    void addComponents(IBaseComponents *item);
    void initItem();//core method that init all items
};

#endif // SCENECONTROLLER_H
