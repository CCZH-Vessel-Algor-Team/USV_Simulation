#ifndef NAVRADARPROTOCOL_H
#define NAVRADARPROTOCOL_H

#include "NavTypes.h"

namespace NaviRadar {

#include "BytePackOn.h"

///////////////////////////////////////////////////////////////////////////////
//  Guard Zone Alarm
///////////////////////////////////////////////////////////////////////////////

struct tGuardZoneCtrl
{
    uint16_t zone;
    uint32_t rangeStart_m;
    uint32_t rangeEnd_m;
    uint16_t azimuth_ddeg;
    uint16_t width_ddeg;
} BYTE_ALIGNED;

typedef uint8_t tGuardZoneSelect;

enum {
    cMaxGuardZones = 2,
    cMaxAdvancedGuardZones = 5
};

typedef uint32_t tGuardZoneAlarmType;
enum eGuardZoneAlarmType
{
    eGZAlarmEntry   = 0x0,
    eGZAlarmExit    = 0x1,
    eGZAlarmBoth    = 0x2,

    eTotalGuardZoneAlarmTypes
};

typedef uint32_t tAlarmState;
enum eAlarmState
{
    eAlarmActive    = 0x1,
    eAlarmInactive  = 0x2,
    eAlarmCancelled = 0x3,
};

struct tGuardZoneAlarm               ///  Sructure for conveying guard-zone alarm information
{                                    ///
    tGuardZoneSelect    zone;        ///< Number of the guard-zone that raised the alarm
    tGuardZoneAlarmType type;        ///< Type of alarm that has been raised
    tAlarmState         state;       ///< Current state of the raised alarm
} BYTE_ALIGNED;

///////////////////////////////////////////////////////////////////////////////
//  State
///////////////////////////////////////////////////////////////////////////////

typedef uint32_t tPowerTransmitState;
enum ePowerTransmitState {
    eOff        = 0,
    eStandby    = 1,
    eTransmit   = 2,
    eWarming    = 3,
    eDisconnect = 5,
};

struct tMode                         ///  Structure for conveying radar mode information (eg. scanner state)
{                                    ///
    uint32_t  state;                 ///< One of State::eScannerState
    uint32_t  ttState;               ///< Timed transmit mode setting
    uint32_t  warmupTime_sec;        ///< Number of seconds remaining for the scanner to warm-up
    uint32_t  ttCount_sec;           ///< Seconds remaining before timed-transmit will cause change between standby and transmit
} BYTE_ALIGNED;

///////////////////////////////////////////////////////////////////////////////
//  RadarError
///////////////////////////////////////////////////////////////////////////////

typedef uint32_t tRadarErrorType;
enum eRadarErrorType {
    eErrorHwSetting 		= 0x00000001,
    eErrorHwGetting 		= 0x00000002,
    eErrorHwIntrpts			= 0x00000003,
    eErrorHwIntrptFlagsLost = 0x00000004,
    eErrorMilAbnormal		= 0x00000005,
    eErrorHwErrors			= 0x00000006,

    eTotalErrorTypes,
};

struct tRadarError                  ///  Structure for conveying radar errors
{                                   ///
    tRadarErrorType type;           ///< One of eRadarErrorType enum values
    uint32_t param1;                ///< Information related to the error
    uint32_t param2;                ///< Further information related to the error
} BYTE_ALIGNED;

///////////////////////////////////////////////////////////////////////////////
//  Properties
///////////////////////////////////////////////////////////////////////////////

struct tRadarHwVersion {
    uint32_t hrfHardVersion;
    uint32_t hrfSoftVersion;
    uint32_t hrfSoftDate;
    uint32_t hrfSoftTime;
    uint32_t hdpHardVersion;
    uint32_t hdpSoftVersion;
    uint32_t hdpSoftDate;
    uint32_t hdpSoftTime;
} BYTE_ALIGNED;

struct tNetProperty {
    uint32_t  radarIP;
    uint8_t   dhcp;
    uint8_t   dedicated;
    uint8_t   dedNum;
} BYTE_ALIGNED;

struct tProperties                      ///  Structure for conveying radar propteries (eg. version numbers, on-time, etc)
{                                       ///  ip address use network endian  192.168.1.10    0x0A01A8C0
                                        ///  ip address use host endian     192.168.1.10    0xC0A8010A
    tNetProperty netProperty;           ///< host endian
    tNetProperty netPropertyDefault;    ///< host endian
    uint8_t   netMaskCIDR;              ///< Net mask CIDR
    uint8_t   reserved_1;               ///< Reserved
    uint32_t  powerCycles;              ///< Number of radar power cycles
    uint32_t  scannerTurns;				///< Number of antenna turns
    uint32_t  scannerId;                ///<
    uint32_t  scannerType;           	///< One of eScannerType enum values
    uint32_t  transmitTime_s;           ///< Total transmit time in seconds
    uint32_t  poweronTime_s;            ///< Total power on time in seconds
    uint32_t  maxRange_dm;              ///< Maximum radar range in 10ths of a metre
    uint16_t  scannerSwVersionMajor;    ///< Antenna software, major version number
    uint16_t  scannerSwVersionMinor;    ///< Antenna software, minor version number
    uint32_t  radarSwVersionMajor;      ///< Radar software, major version number
    uint32_t  radarSwVersionMinor;      ///< Radar software, minor version number
    tRadarHwVersion hwVersion;
    uint32_t  radarCtrlAddr;
    uint16_t  radarCtrlPort;
    uint32_t  radarSpokeAddr;
    uint16_t  radarSpokePort;
    uint32_t  radarStateAddr;
    uint16_t  radarStatePort;
    uint32_t  insDevIP;
    uint16_t  insDevPort;
    uint64_t  timestampUtc_ms;
    uint32_t  targetMsgAddr;
    uint16_t  targetMsgPort;
    uint8_t   reserved_2;
} BYTE_ALIGNED;

///////////////////////////////////////////////////////////////////////////////
//  Setup
///////////////////////////////////////////////////////////////////////////////

typedef uint32_t tUserGainManualAuto;
enum eUserGainManualAuto
{
    eUserGainManual       = 0,
    eUserGainAuto         = 1,
    eUserGainAutoHarbour  = eUserGainAuto,
    eUserGainAutoOffshore = 2,

    eTotalUserGains       = 3
};

enum eSetupGainControls
{
    eSetupGain,
    eSetupSea,
    eSetupRain,

    eTotalSetupGains
};

typedef uint8_t tStcCurveType;
enum eStcCurveType                ///  Enumeration of sea-conditions for STC curves
{
    eCalm,
    eModerate,
    eRough,

    eTotalStcCurves
};

struct tGainControl               ///  Structure for conveying gain-control mode and level information
{                                 ///
    uint32_t  type;               ///< One of eUserGainManualAuto enum values
    uint8_t   value;              ///< Manual gain settings (valid only if \c type is eUserGainManual)
} BYTE_ALIGNED;

struct tFTCControl
{                                 ///
    uint32_t  type;               ///< unused
    uint8_t   value;              ///< FTC level (0-255)
} BYTE_ALIGNED;

struct tGuardZone                 ///  Structure for conveying GuardZone zone setup information
{                                 ///
    uint32_t  orientation;        ///< Relative to vessel (0) or north/absolute (1)
    uint32_t  rangeStart_m;       ///< Close range from boat (metres)
    uint32_t  rangeEnd_m;         ///< Far range from boat (metres)
    uint16_t  azimuth_ddeg;       ///< Starting angle (10ths of a degree relative to \c orientation reference)
    uint16_t  width_ddeg;         ///< Width angle (10ths of a degree - deci degrees)
} BYTE_ALIGNED;

struct tGuardZoneAlarmSetup               ///  Structure for conveying GuardZone alarm setup information
{                                         ///
    tGuardZoneAlarmType alarmType;        ///< Alarm types
    uint8_t   enabled;                    ///< Alarm enabled state
} BYTE_ALIGNED;

struct tGuardZones                        ///  Structure for conveying all GuardZone setup information
{                                         ///
    uint8_t   sensitivity;                ///< Sensitivity (low 0-255 high)
    uint8_t   active[cMaxGuardZones];     ///< true if the corresponding guard-zone is enabled
    tGuardZone zone[cMaxGuardZones];
    tGuardZoneAlarmSetup alarmType[cMaxGuardZones];
} BYTE_ALIGNED;

struct tSetup                             ///  Structure for conveying radar setup information (eg. range, gains, etc)
{
    uint32_t  range_dm;                   ///< Currently selected range (in 10ths of a metre)
    uint8_t   spokeOutputEnable;          ///<
    uint8_t   spokeCompressed;            ///<
    tGainControl gain[eTotalSetupGains];  ///< Indexed by eSetupGainControls
    tFTCControl ftc;                      ///< Fast-Time-Constant level
    uint8_t   dynamicPeriod;
    uint8_t   dynamicBandwidth;
    uint16_t  hwRangeValidPct;            ///<
    uint8_t   maxFastScan;                ///<
    uint8_t   hwRangeIndex;               ///<
    uint32_t  interferenceReject;         ///< Interference-Reject level (off 0-3 high)
    uint32_t  targetStretch;              ///< Target-Stretch level
    uint32_t  targetBoost;                ///< Target-Boost, AKA target-emphasis (off 0-2 high)
    uint32_t  spokeRange_dm;              ///<
    uint32_t  hwRange_dm;                 ///<
    tGuardZones guardzones;               ///< Guard-Zone setup
} BYTE_ALIGNED;

struct tSetupExtended                     ///  Structure for conveying extra setup information (eg. newer controls, etc)
{                                         ///
    tStcCurveType stcCurveType;           ///< STC curve type for automatic sea clutter rejection (calm 0-2 rough)
    uint8_t   localIR;                    ///< Local interference-reject (off 0-3 high)
    uint8_t   fastScanMode;               ///< Faster scanner speed at shorter ranges
    tGainControl sidelobe;                ///< Sidelobe processing state
    uint16_t  rpmX10;                     ///< Scanner roation speed
    uint8_t   noiseReject;                ///< Noise rejection (coherent integration) (off 0-2 high)
    uint8_t   beamSharpening;             ///< Beam sharpening (off 0-3 high)
    uint8_t   sidelobeLevel;              ///< Sidelobe level(0-3)
    uint16_t  unused_1;                   ///<
    uint8_t   unused_2;                   ///< unused
} BYTE_ALIGNED;

struct tBlankSector                     ///  Structure for conveying StopTx sector information
{                                       ///
    uint8_t  state;                     ///< Non-zero if this stop-tx sector is active/enabled
    uint16_t  sectorStart_ddeg;         ///< Sector start (deci-degrees, or 10ths of a degree)
    uint16_t  sectorEnd_ddeg;           ///< Sector end (deci-degrees, or 10ths of a degree)
} BYTE_ALIGNED;

struct tBlankSectorCtrl
{
    uint32_t id;
    tBlankSector sector;
} BYTE_ALIGNED;

///////////////////////////////////////////////////////////////////////////////
//  Configuration
///////////////////////////////////////////////////////////////////////////////

enum {
    cMaxBlankSectors = 6                ///< Maximum number of stop sectors able to be supported
};

struct tConfiguration                   ///  Structure for conveying radar configuration information (eg. installation params)
{                                       ///
    uint32_t  unused_1;                 ///< unused
    uint16_t  zeroBearing_ddeg;         ///< Zero bearing offset (deci-degrees, or 10ths of a degree)
    uint16_t  unused_2;                 ///< unused
    uint32_t  antennaHeight_mm;         ///< Height of the antenna above the water (millimetres)
    int16_t   insAdjustTim_ms;          ///<
    int16_t   insHeadingModify_ddeg;    ///<
    uint8_t   insEnable;                ///< unused
    uint8_t   unused_4;                 ///< unused
    uint8_t   bitsPerSample;
    uint8_t   targetEnable;
    uint32_t  currHrfFreq_100KHz;       ///< Current Hrf Freq
    uint32_t  timedTransmitPeriod_s;    ///< Timed-transmit transmit period (seconds)
    uint32_t  timedStandbyPeriod_s;     ///< Timed-transmit standby period (seconds)
    tBlankSector blankSectors[cMaxBlankSectors]; ///< Indexed by stop-tx sector number
    uint16_t  unused_6;                 ///< reserved (was wasted blank sector data)
} BYTE_ALIGNED;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//  AdvancedSetup
///////////////////////////////////////////////////////////////////////////////

struct tTrackSectorCtrl
{
    uint32_t id;
    uint8_t  state;
    uint32_t rangeStart_m;
    uint32_t rangeEnd_m;
    uint16_t azimuth_ddeg;
    uint16_t width_ddeg;
} BYTE_ALIGNED;

struct tAdvancedGuardZones
{
    uint8_t   sensitivity;
    uint8_t   active[cMaxAdvancedGuardZones];
    tGuardZone zone[cMaxAdvancedGuardZones];
    tGuardZoneAlarmSetup alarmType[cMaxAdvancedGuardZones];
} BYTE_ALIGNED;

enum {
    cMaxTrackSectors = 6
};

struct tTrackSectors
{
    uint8_t  state[cMaxTrackSectors];
    uint32_t  rangeStart_m[cMaxTrackSectors];
    uint32_t  rangeEnd_m[cMaxTrackSectors];
    uint16_t  azimuth_ddeg[cMaxTrackSectors];
    uint16_t  width_ddeg[cMaxTrackSectors];
} BYTE_ALIGNED;

struct tAdvancedSetup
{
    tAdvancedGuardZones guardzones;
    tTrackSectors trackSectors;
} BYTE_ALIGNED;


///////////////////////////////////////////////////////////////////////////////
//  FactoryConfiguration
///////////////////////////////////////////////////////////////////////////////

enum eRadarInstance : uint8_t {
    eRadarA,
    eRadarB,
    eAllRadars,
};

enum {
    cRpmGrades = 5,
    cRangeGrades = 20,
};

typedef uint8_t tHwChannelType;
enum eHwChannelType : uint16_t {
    eProfessional,
    eIntelligence,
    eTest,
};

struct tHardwareChannel { ///Each range use different parameter
    tHwChannelType channelNum;//default eIntelligence
    uint16_t hwStc;//0-7
} BYTE_ALIGNED;

struct tNoiseEstimation {///Each noiseRejectLevel use different parameter
    uint16_t noiseSegment;
    double localNoiseLimit;
    double globalNoiseLimit;
} BYTE_ALIGNED;

struct tNoiseEstimationCtrl {
    uint8_t noiseRejectLevel;//0\1\2
    tNoiseEstimation noiseEstimation;
} BYTE_ALIGNED;

struct tADIntercept {
    uint16_t threshold_1;
    uint16_t threshold_2;
} BYTE_ALIGNED;

enum {
    cNoiseEstimaLevel = 3,
};

struct tFactoryConfiguration {
    uint8_t isSpokeEqualSetup;
    uint8_t unused_1;
    uint32_t hrfFreq_100KHz;
    tHardwareChannel hwChannels[cRangeGrades];
    tNoiseEstimation noiseEstimations[cNoiseEstimaLevel];
    uint32_t reserved_1[7];
    tADIntercept adIntercept;
} BYTE_ALIGNED;

enum {
    cStcCurveLen = 512,
};

struct tStcCurves {
    uint16_t stcCurves[cStcCurveLen];
    uint32_t range_dm;
    uint8_t stcType;
} BYTE_ALIGNED;

struct tStcConfiguration {
    uint16_t stcCurves[cStcCurveLen];
    uint32_t range_dm;
    uint32_t hwRange_dm;
    uint8_t stcType;
} BYTE_ALIGNED;

///////////////////////////////////////////////////////////////////////////////
//  DebugConfiguration
///////////////////////////////////////////////////////////////////////////////

typedef uint16_t tFilterType;
enum eFilterType : uint16_t {
    eNoSmooth,
    eMean1_2,
    eMean1_3,
    eMean1_5,
    eMean1_7,
    eMean1_9,
    eMean2_3,
    eMean2_5,
    eMean2_7,
    eMean2_9,

    eTotalFilterTypes,
};


typedef uint8_t tTimeSpanType;
enum eTimeSpanType {
    eNull               = 0,
    eProcessed          = 0x01,
    eInterpolated       = 0x02,
    eUdptransmitted     = 0x04,

    eTotalTimeSpanTypes = 16,
};

struct tSettingState {
    uint32_t powerA;
    uint32_t powerB;
    uint32_t transmitA;
    uint32_t transmitB;
    uint32_t rangeA;
    uint32_t rangeB;
    uint32_t rpm;
} BYTE_ALIGNED;

struct tRadarStatusReport {
    uint16_t intrptsPerMil;
    uint16_t intrptsPerSec;
    uint16_t intrptErrsPerSec;
    uint16_t softMilsLost;
    uint16_t originalMils;
    uint16_t secondCnt;
    uint16_t processPeriod[eTotalTimeSpanTypes];
    uint16_t maxTimeSpan;
    uint16_t storageCtrlMsgSize;
    tSettingState hwSetting;
} BYTE_ALIGNED;

typedef uint8_t tHwDataType;
enum eHwDataType : uint16_t {
    eFFT,
    eAD,
};

struct tSmoothTypes {
    tFilterType azmSmooth;
    tFilterType disSmooth;
    tFilterType detectedSmooth;
    tFilterType _reserved;
} BYTE_ALIGNED;

struct tProcess {
    tSmoothTypes smoothTypes;
    uint16_t reserved_1[4];
    tNoiseEstimation noiseEstimation;
} BYTE_ALIGNED;

struct tOriginalProcess {
    uint8_t originalDataEnable;
    tHwDataType hwDataType;
    uint16_t _reserved;
} BYTE_ALIGNED;

struct tProcessConfiguration {
    tSmoothTypes smoothTypes;
    tOriginalProcess originalProcess;
    tRadarStatusReport statusReport;
    tHardwareChannel currentHwChannel;
    uint16_t reserved_1[4];
    tNoiseEstimation currentNoiseEstimation;
} BYTE_ALIGNED;

struct tDebuggingPara {
    uint8_t highPassFilter;
    uint8_t fftWindowForm;
    uint8_t noiseThresMethod;
} BYTE_ALIGNED;

typedef uint8_t tInsFlag;
enum eInsFlag : uint8_t {
    eInsTime                = 0x01,
    eLatLng                 = 0x02,
    eCourse                 = 0x04,
    eSpeed                  = 0x08,
    eHeading                = 0x10,
    eAllValidFlag           = 0x1F,
};
struct tLngLat {
    double lat; //纬度latitude
    double lng; //经度longitude
} BYTE_ALIGNED;
struct tCartCoordPnt {
    double x;
    double y;
} BYTE_ALIGNED;
// 惯导数据信息
struct tBoatInsInfo {
    uint32_t insIP;
    uint16_t insPort;
    uint64_t insTim_ms; // 惯导时间: 惯导数据自身携带的时间
    uint64_t fixTim_ms; // 修正时间: 根据数据包收取的时间 修正后的设备时间
    uint64_t adjTim_ms; // 调整时间: 手动通过指令调整的时间
    tLngLat currLngLat;
    double course;  //航向
    double speed;   //航速m/s
    double heading; //艏向
    double turnRate;//艏向变化率 deg/min
    double reserved_1[4];
    uint32_t reserved_2;
    uint32_t insParserVersion;  //惯导解析库的版本

    tLngLat baseLngLat; //本船的基准经纬度
    tCartCoordPnt currCartCoord_m;
    tCartCoordPnt predictCartCoord_m;
    double kalmCourse;  //航向
    double kalmSpeed;   //航速m/s

    // 当前惯导信息的有效性:true时以上数据有效，false时以上数据无效
    bool currValid; // flag=0x0F时，currValid为true

    // 以下数据与currValid是否有效无关
    uint16_t packFreq;              // 每秒收取数据包的数量
    uint64_t packRecTim_ms;         // 数据包接收时的本地时间
    tInsFlag flag;                  // 惯导信息数据由多个包组成，需要一个标志用来标记一组惯导信息是否接收完整
    uint16_t invalidGroupCnt;       // 组包异常的计数
    uint16_t timeOutCnt;
    uint32_t timeOutSpan;
    // 每来一条惯导数据，都会生成一个tBoatInsInfo
    // 惯导数据分包接收未完成时，以下数据可用，flag的状态一致变化
    uint16_t groupFreq;               // 有效惯导数据的频率(可能需要接收多个包)
    // 惯导数据接收完成时（不一定完整flag<=0x0F),以下数据可用
    char packageInfo[256];
    // 惯导数据完整有效时，flag的值固定为0x0F
} BYTE_ALIGNED;

// spoke匹配的定位信息
struct tSpokeInsInfo {
    uint64_t adjTim_ms;
    tLngLat currLngLat;
    double course;
    double speed;
    double heading;
    tLngLat baseLngLat;
    tCartCoordPnt currCartCoord_m;
    bool valid;
} BYTE_ALIGNED;

//-----------------------------------------------------------------------------
#include "BytePackOff.h"

} //NaviRadar
#endif // NAVRADARPROTOCOL_H
