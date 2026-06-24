#ifndef TARGETPROTOCOL_H
#define TARGETPROTOCOL_H

#include <stdint.h>
#include <string>

#include <BytePackOn.h>
namespace NaviRadar {
namespace Target{

typedef uint32_t tBearingType;
enum eBearingType
{
    eRelative   = 0,         ///< Relative to boat direction (bearing zero)
    eAbsolute   = 1,         ///< Relative to North (either magnetic or true)

    eTotalBearingTypes
};

typedef uint32_t tHeadingType;
enum eHeadingType
{
    eNoHeading     = 0,      ///< No heading available
    eMagneticNorth = 1,      ///< Relative to magnetic north
    eTrueNorth     = 2,      ///< Relative to true north

    eTotalHeadingTypes
};

typedef uint32_t tTargetType;
enum eTargetType
{
    eTargetTypeVessel = 0,
};

typedef uint32_t tSpeedType;
enum eSpeedType
{
    eNoSpeed         = 0,    ///< Unused
    eSpeedOverGround = 1,    ///< Speed Over Ground (SOG), eg. from GPS
    eWaterSpeed      = 2,    ///< Speed relative to the Water

    eTotalSpeedTypes
};

typedef uint32_t tDirectionType;
enum eDirectionType
{
    eNoDirection      = 0,   ///< Unused
    eCourseOverGround = 1,   ///< Course-Over-Ground (COG), eg. from GPS
    eHeadingMagnetic  = 2,   ///< Heading is relative to magnetic-north
    eHeadingTrue      = 3,   ///< Heading is relative to true-north

    eTotalDirectionTypes
};

typedef uint8_t tCaptureMode;
enum eCaptureMode : uint8_t {
    eManualMode,
    eAutoMode,
};

struct tTargetTrackingAlarmSetup      ///  Structure for conveying target-tracking alarm setup information
{                                     ///
    uint32_t  safeZoneDistance_m;     ///< Safe distance from boat, in metres
    uint32_t  safeZoneTime_sec;       ///< Safe amount of seconds before target gets within safeZoneDistance_m
} BYTE_ALIGNED;

struct tTargetTrackingProperties      ///  Structure for conveying target-tracking properties (eg. type of heading used)
{                                     ///
    tHeadingType    headingType;      ///< Compass heading type being used, one of eHeadingType enums
    tCaptureMode    captureMode;
    uint8_t         ttmConfigAbsolute;
    uint8_t         ttmCurrentAbsolute;
    uint8_t         tllEnable;
    uint8_t         tsmEnable;
    uint8_t         ttmtllMerge;
    uint8_t         ttmtllFixHead;
} BYTE_ALIGNED;

//---------------------------------------------------------------------------

typedef uint32_t tTargetState;
enum eTargetState              ///  Enumerations for Target-Tracking target states
{                              ///
    eAcquiringTarget = 0,      ///< Attempting to acquire target
    eSafeTarget      = 1,      ///< Target acquired and not on a collision course
    eDangerousTarget = 2,      ///< Target acquired and may be on a collision course
    eLostTarget      = 3,      ///< Target has been lost and needs to be cancelled an reacquired
    eAcquireFailure  = 4,      ///< Failed to acquire a target
    eLostingTarget   = 5,
    eOutOfRange      = 6,      ///< Target is now out of range
    eLostOutOfRange  = 7,      ///< Target lost due to staying out of range
    eInGuardZone     = 8,
    eFailAcquireMax  = 0x10,   ///< Acquire failed because no target IDs were free
    eFailAcquirePos  = 0x11,   ///< Acquire failed because the position was invalid

    eBadState        = 0xBAD15BAD,
};

typedef uint32_t tAcquireType;
enum eAcquireType
{
    eManualAcquired  = 0,      ///< Target was manually acquired
    eAutoAcquired    = 1,      ///< Not currently supported
};

typedef uint8_t tTargetSource;
enum eTargetSource
{
    TSource_Simulated = 0,
    TSource_Real      = 1,
};

struct tTargetInfo             ///  Structure for conveying target-tracking target information (eg. course and location)
{                              ///
    uint32_t  distance_m;      ///< Distance to target in metres
    uint32_t  bearing_ddeg;    ///< Target bearing in 10ths of a degree (deci-degrees)
    uint32_t  course_ddeg;     ///< Target course in 10ths of a degree (deci-degrees)
    uint32_t  speed_dmps;      ///< Target speed in 10ths of a metre per second (deci-metres/second)
} BYTE_ALIGNED;

struct tTrackedTarget                 ///  Structure for conveying all target-tracking target information
{                                     ///
    uint8_t       targetValid;        ///< 0x00 invalid, 0x01 valid
    tTargetSource targetSource;       ///< One of eTargetSource enum values (eg. simulated or real)
    tTargetType   targetType;         ///< One of eTargetType enum values
    uint32_t      targetID;           ///< Client assigned target-ID (provided by user in manual-acquire0
    int32_t       serverTargetID;     ///< Server assigned target-ID (negative when invalid acquire - eFailAcquire* states)
    tTargetState  targetState;        ///< One of eTargetState enum values
    tAcquireType  autoAcquired;       ///< One of eAcquireType enum values (manually or automatically acquired)

    tTargetInfo   infoRelative;       ///< Target details expressed relative to the boats speed and direction
    tTargetInfo   infoAbsolute;       ///< Target details expressed independant of the boat (relative to true north)
    uint8_t       infoAbsoluteValid;  ///< Whether 'infoAbsolute' is valid (0 absolute info invalid, 1 valid)

    int32_t       CPA_m;              ///< Closest point of approach expressed in meters
    int32_t       TCPA_sec;           ///< Time to the closest point of approach expressed in seconds
    uint8_t       towardsCPA;         ///< CPA direction (0 target moving away from CPA, 1 towards CPA)
    double        longitude;
    double        latitude;
    uint8_t       reserved[16];
} BYTE_ALIGNED;

typedef uint8_t tTrackingAreaType;
enum eTrackingAreaType {
    eCaptureArea,   // 捕获区
    eShieldArea,    // 屏蔽区
};

struct tTrackingAreaInfo {
     uint16_t id;
     uint8_t valid;
     uint8_t enable;
     uint16_t numOfPoints;
     bool operator<(const tTrackingAreaInfo& other) const {
         if (id != other.id) return id < other.id;
         if (valid != other.valid) return valid < other.valid;
         if (enable != other.enable) return enable < other.enable;
         return numOfPoints < other.numOfPoints;
     }
} BYTE_ALIGNED;

}//namespace Target
}//namespace NaviRadar
#include <BytePackOff.h>
#endif // TARGETPROTOCOL_H
