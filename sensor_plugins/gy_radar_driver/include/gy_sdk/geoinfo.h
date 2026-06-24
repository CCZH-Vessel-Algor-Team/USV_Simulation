#ifndef GEOINFO_H
#define GEOINFO_H

namespace NaviRadar {
namespace Geo {
#include <BytePackOn.h>
enum eDirectionType : uint16_t{
    eNorth,
    eSouth,
    eEast,
    eWest,
};
struct tGeoPosition
{
    bool    enable;
    double  longitude;
    double  latitude;
    double  altitude;
} BYTE_ALIGNED;

enum eSpeedSource : uint16_t{
    eSSourceInvalid,
    eSSourceGPS,
};
struct tGeoSpeed
{
    double          speed_kmh;
    double          track_deg;
    eSpeedSource    speedSource;
} BYTE_ALIGNED;

enum eHeadingZeroPoint : uint32_t
{
    eHeadingInvalid,
    eHeadingTrueNorth,
    eHeadingMagneticNorth,
};
struct tGeoHeading
{
    double              headingAngle_deg;
    eHeadingZeroPoint   headingType;
} BYTE_ALIGNED;
#include <BytePackOff.h>

class iGeoInfoObserver
{
public:
    virtual ~iGeoInfoObserver() {}
    virtual void UpdateGeoPosition(const tGeoPosition *pPosition) = 0;
    virtual void UpdateGeoSpeed(const tGeoSpeed *pSpeed) = 0;
    virtual void UpdateGeoHeading(const tGeoHeading *pHeading) = 0;
};


}//namespace Geo
}//namespace NaviRadar
#endif // GEOINFO_H
