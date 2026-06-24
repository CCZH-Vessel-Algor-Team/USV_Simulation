#pragma once

#include <NavTypes.h>
#include <NavRadarSpoke.h>
#include <ExportDll.h>

namespace NaviRadar
{

typedef uint32_t tColor, *pColor;
typedef const tColor *pLookUpTable;
enum eCOLOR_TYPE :int
{
    eDefaultColor,
    eBlueColor
};

pLookUpTable DLL_API GetLookUpTable(eCOLOR_TYPE colortype = eDefaultColor);

namespace PPI
{
class iTargetTrackingClientObserverPPI;
class PPIControllerImpl;

enum eAzimuthInterPolation :int
{
    enmAIPnone,
    enmAIPrepeat,
};
enum eRangeInterPolation :int//not used
{
    enmRIPnone,
    enmRIPpeak,
};

class DLL_API PPIController
{
public:
    PPIController();
    ~PPIController();
    void InitFrameBuffer(uint32_t nOfSamples, uint32_t nOfSpokes, const pLookUpTable plut, pColor pBuffer = nullptr);
    void SetColorLookUpTable(const pLookUpTable plut);

    void SetRangeResolution(uint32_t metresOfRadius);
    void SetAzimuthInterpolation(eAzimuthInterPolation value);
    eAzimuthInterPolation GetAzimuthInterpolation(void) const;
    void SetRangeInterpolation(eRangeInterPolation value);
    eRangeInterPolation GetRangeInterpolation(void) const;
    void SetTrailsTime(int32_t time_sec, int32_t radarRPM = 60);
    void Process(const Spoke::SPOKE *pspokes);
    void GetVersion(uint32_t& major,uint32_t& minor,uint32_t& build);
    void setThreshold(uint8_t threshold);
    void clearTrails();
    void setAzimuthShift(int shift_degree);
    //for tracker
//    void setRadarRange(int range_m);
//    bool setBoatSpeed( Target::tSpeedType speedType, uint32_t speed_dmps, Target::tDirectionType directionType, uint32_t direction_deg );
//    bool cancelByID(uint32_t ID);
//    bool cancelAll();
//    void queryTarget();
//    void queryAlarm();
//    void queryProperty();
//    bool acquireTarget( uint32_t id, uint32_t range_m, uint16_t bearing_deg, Target::tBearingType bearingType );
//    void setTargetObserver(iTargetTrackingClientObserverPPI* pTar);
//    void removeTargetObserver(iTargetTrackingClientObserverPPI* pTar);
private:
    PPIControllerImpl* m_pImpl;
};

}// namespace PPI
}// namespace naviRadar
