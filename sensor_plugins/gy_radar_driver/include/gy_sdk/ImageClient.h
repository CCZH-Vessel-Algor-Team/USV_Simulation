#ifndef IMAGECLIENT_H
#define IMAGECLIENT_H

#include "ExportDll.h"
#include "ImageClientObserver.h"

namespace NaviRadar {

class tImageClientImpl;

//-----------------------------------------------------------------------------
//! Class for controlling the state of the radar plus receiving spokes and state
//! information.
//-----------------------------------------------------------------------------
class DLL_API tImageClient
{
public:
    tImageClient();
    virtual ~tImageClient();

    //-----------------------------------------------------------------------------------
    //! Add an observer for radar state information
    //-----------------------------------------------------------------------------------
    bool AddStateObserver( iImageClientStateObserver * pObserver );

    //-----------------------------------------------------------------------------------
    //! Remove a previously added radar state observer
    //-----------------------------------------------------------------------------------
    bool RemoveStateObserver( iImageClientStateObserver * pObserver );

    //-----------------------------------------------------------------------------------
    //! Add an observer for radar images/spokes
    //-----------------------------------------------------------------------------------
    bool AddSpokeObserver( iImageClientSpokeObserver * pObserver );

    //-----------------------------------------------------------------------------------
    //! Remove a previously added observer for radar images/spokes
    //-----------------------------------------------------------------------------------
    bool RemoveSpokeObserver( iImageClientSpokeObserver * pObserver );

    bool connect(const char *pSerialNumber, uint16_t instance);

    bool disconnect();

    /////////////////////////////////////////////////////////////////////////////////////
    //  Commands
    /////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------------------------------
    //! Set the power state of the radar
    //! \param state The new state
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetPower( bool state );

    //-----------------------------------------------------------------------------------
    //! Set the transmission state of the radar. This will also start the scanner rotating
    //! \param state The new state
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetTransmit( bool state );

    //-----------------------------------------------------------------------------------
    //! Set the timed transmission state of the radar. This will cause the radar to
    //! alternately transmit for a period and then enter standby mode for a period of time.
    //! The transmit and standby periods must be set to valid values before this operation
    //! will have any effect (see \ref SetTimedTransmitSetup).
    //! \param state The new timed transmit state
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetTimedTransmit( bool state );

    //-----------------------------------------------------------------------------------
    //! Setup the conditions for timed radar transmission.
    //! \param transmitPeriod_sec The number of seconds to transmit for
    //! \param standbyPeriod_sec The number of seconds remain in standby for
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetTimedTransmitSetup( uint32_t transmitPeriod_sec, uint32_t standbyPeriod_sec );

    //-----------------------------------------------------------------------------------
    //! Set the new range in the radar
    //! \param range_m The new range expressed in meters
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetRange( uint32_t range_m );

    //-----------------------------------------------------------------------------------
    //! Send a client watch-dog kick, to make sure the radar doesn't stop sending spokes.
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool SendClientWatchdog();

    //-----------------------------------------------------------------------------------
    //! Set the gain control
    //! \param type The type of gain can be: eUserGainManual (in this situation the 'level'
    //!     value will be applied), or eUserGainAuto (the radar will work in automatic
    //!     gain mode)
    //! \param level The value to use in manual gain mode (0-255)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetGain( eUserGainManualAuto type, uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Set the sea clutter level
    //! \param type The type of sea clutter can be: eUserGainManual (in this situation
    //!     the 'level' value will be applied), eUserGainAutoHarbour (the radar will
    //!     work in auto mode optimised for Harbour situations), or eUserGainAutoOffshore
    //!     (the radar will work in auto mode optimised for offshore situations).
    //! \param level The value to use in eUserGainManual mode (0-255)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetSeaClutter( eUserGainManualAuto type, uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Set the type of STC curve that will be applied in automatic sea clutter rejection.
    //! \param type The curve type to use, one of the eStcCurveType enum values.
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetSTCCurveType( eStcCurveType type );

    //-----------------------------------------------------------------------------------
    //! Set the rain clutter rejection level
    //! \param level The manual gain value (0-255)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetRain( uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Set the sidelobe control
    //! \param type The type of gain can be: eUserGainManual (in this situation the 'level'
    //!     value will be applied), or eUserGainAuto (the radar will work in auto gain mode)
    //! \param level It is the manual gain value (0-255)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetSideLobe( eUserGainManualAuto type, uint8_t level );
    //-----------------------------------------------------------------------------------
    //! Set level of Manual Side Lobe
    //! \param level Amount of rejection to apply (none 0 to 3 high)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetSideLobe( uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Set the FTC level
    //! \param level It is the FTC value (0-255)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetFTC( uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Set the interference reject level
    //! \param level It is a value between 0 and 3 - 0 disabled, 3 highest rejection
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetInterferenceReject( uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Set level of local-interference reject
    //! \param level Amount of rejection to apply (none 0 to 3 high)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetLocalIR( uint8_t level );

    //---------------------------------------------------------------------------
    //! Set the Noise Reject level
    //! \param level Amount of noise rejection to apply (none 0 to 3 high)
    //! \return True if the operation was successful
    //---------------------------------------------------------------------------
    bool SetNoiseReject( uint8_t level );

    //---------------------------------------------------------------------------
    //! Set the Beam Sharpening level
    //! \param level Amount of beam-sharpening to apply (none 0-3 high)
    //! \return True if the operation was successful
    //---------------------------------------------------------------------------
    bool SetBeamSharpening( uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Selects between normal and fast scanner rotation speeds. Note that other controls,
    //! such as range and noise-reject, can restrict the maximum RPM available to lower
    //! than this setting.
    //! \param level Selects one of the RPM rates: 0 normal/24rpm, 1 36rpm, or 2 48rpm
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetFastScanMode( uint8_t level );

    //-----------------------------------------------------------------------------------
    //! Set target stretch/expansion in SW
    //! \param state If true, the target will appear larger, if false, with normal size
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetTargetStretch( bool state );

    //-----------------------------------------------------------------------------------
    //! Set target boost in hardware, changing physical parameters of the radar (pulse
    //! width or sweep length, for example).
    //! \param level The level of target boost desired. 0 = best target detail, 2 most
    //!     target emphasis.
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetTargetBoost( uint8_t level );


    /////////////////////////////////////////////////////////////////////////////////////
    //  Advanced Commands
    /////////////////////////////////////////////////////////////////////////////////////

    //---------------------------------------------------------------------------
    //! If the scanner supports this mode, the scanner can be configured not to
    //! transmit in a specific sector of rotation
    //! \param sectorID sector ID
    //! \param state if false, the sector will be in off state, if true will be
    //! in on state
    //! \param startBearing_degrees Start of the sector in degrees
    //! \param endBearing_degrees End of the sector in degrees
    //! \return True if the operation was successful
    //---------------------------------------------------------------------------
    bool SetSectorBlankingSetup( uint32_t sectorID, bool state, uint16_t startBearing_degrees, uint16_t endBearing_degrees );

    bool SetSectorTrackingSetup( uint32_t sectorID, bool state, uint32_t startRange_m, uint32_t endRange_m, uint16_t bearing_deg, uint16_t width_deg );

    /////////////////////////////////////////////////////////////////////////////////////
    //  Queries
    /////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------------------------------
    //! Query all info from a radar
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool QueryAll();

    //-----------------------------------------------------------------------------------
    //! Query mode info from a radar
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool QueryMode();

    //-----------------------------------------------------------------------------------
    //! Query setup info from a radar
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool QuerySetup();

    //-----------------------------------------------------------------------------------
    //! Query properties info from a radar
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool QueryProperties();

    //-----------------------------------------------------------------------------------
    //! Query configuration info from a radar
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool QueryConfiguration();

    /////////////////////////////////////////////////////////////////////////////////////
    //  Configuration
    /////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------------------------------
    //! Set the zero bearing offset
    //! \param bearing_ddeg The offset expressed in 10ths of a degree (deci-degrees)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetZeroBearingOffset( uint16_t bearing_ddeg );

    //-----------------------------------------------------------------------------------
    //! Set the antenna height
    //! \param antennaHeight_mm Height of the antenna, expressed in millimeters
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetAntennaHeight( uint32_t antennaHeight_mm );

    //-----------------------------------------------------------------------------------
    //! Sets the radar to use factory defaults
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetToFactoryDefaults();

    bool SetStcCurves(tStcCurves stc);

    bool StcCurvesStorage();

    bool StcCurvesReset();

    /////////////////////////////////////////////////////////////////////////////////////
    //  GuardZones
    /////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------------------------------
    //! Enable guard zone detection
    //! \param zone Guard zone index/number (0 to cMaxGuardZones-1)
    //! \param state If true, it will be enabled, false it will be disabled
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetGuardZoneEnable( uint8_t zone, bool state );

    //-----------------------------------------------------------------------------------
    //! Sets up the size and location information for a guard zone
    //! \param zone Guard zone index/number (0 to cMaxGuardZones-1)
    //! \param startRange_m Start of the sector, expressed in meters
    //! \param endRange_m End of the sector, expressed in meters
    //! \param bearing_deg Position of centre of the sector, expressed in degrees
    //! \param width_deg Width of the sector, expressed in degrees (use 360 degrees to
    //!     define a circular sector)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetGuardZoneSetup( uint8_t zone, uint32_t startRange_m, uint32_t endRange_m, uint16_t bearing_deg, uint16_t width_deg );

    //-----------------------------------------------------------------------------------
    //! Sets up the type of alarm for each guard zone
    //! \param zone Guard zone index/number (0 to cMaxGuardZones-1)
    //! \param type  The alarm type: eGZAlarmEntry (0) alarm only on targets entering the
    //!     zone, eGZAlarmExit (1) alarm only on targets exiting, eGZAlarmBoth (2) alarm
    //!     on targets both entering and exiting the zone
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetGuardZoneAlarmSetup( uint8_t zone, eGuardZoneAlarmType type );

    //-----------------------------------------------------------------------------------
    //! Cancels an alarm coming from guard zone system
    //! \param zone Guard zone index/number (0 to cMaxGuardZones-1)
    //! \param type Type of alarm we want to cancel: eGZAlarmEntry (0) or eGZAlarmExit (1)
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetGuardZoneAlarmCancel( uint8_t zone, eGuardZoneAlarmType type );

    //-----------------------------------------------------------------------------------
    //! Sets up the sensitivity of guard zones
    //! \param level The level of sensitivity. 0 = no sensitivity, 255 = max sensitivity
    //! \return True if the operation was successfully initiated
    //-----------------------------------------------------------------------------------
    bool SetGuardZoneSensitivity( uint8_t level );

    /////////////////////////////////////////////////////////////////////////////////////
    //  FactoryConfiguration
    /////////////////////////////////////////////////////////////////////////////////////
    bool SetIndeptManualGainEnable(bool enable);
    bool SetSpokeLengthType(bool isSpokeEqualSetup);
    bool SetHrfFreq(double freq_mhz);
    bool SetHardwareChannel(uint8_t rangeIndex, tHardwareChannel value);
    bool SetNoiseEstimation(uint8_t level, tNoiseEstimation noise);

    bool SetADIntercept(uint16_t threshold, uint16_t length);
    bool SetFastScanLimit(bool enable);
    /////////////////////////////////////////////////////////////////////////////////////
    //  DebugConfiguration
    /////////////////////////////////////////////////////////////////////////////////////
    bool SetProcess(tProcess process);
    bool SetOriginalProcess(tOriginalProcess originalProcess);

    bool EraseFactoryFlash();
    /////////////////////////////////////////////////////////////////////////////////////
    //  Implementation
    /////////////////////////////////////////////////////////////////////////////////////
    bool SetIpConfig(bool dhcp, uint32_t ip);
    bool SetNetMask(uint32_t mask);
    bool SetDedicatedNum(bool dedicated, uint8_t num);
    bool SetArpaEnable(bool enable);
    bool SetInsAdjustTim(int16_t tim_ms);
    bool SetInsHeadingModify(int16_t ddeg);
    bool SetInsEnable(bool enable);
    bool SetInsAddress(uint32_t ip, uint16_t port);
    bool SetFixedInsInfo(double lng, double lat, double heading);
    bool SetTimingInfo(uint64_t timstmp_ms);
    bool SetSpokeOutputEnable(bool enable);
    bool SetSpokeCompressEnable(bool enable);
    bool SetDynamicPeriod(uint8_t level);
    bool SetDynamicBandwidth(uint8_t level);
    bool SetRadarCtrlAddress(uint32_t ip, uint16_t port);
    bool SetRadarSpokeAddress(uint32_t ip, uint16_t port);
    bool SetRadarStateAddress(uint32_t ip, uint16_t port);
    bool SetTargetMsgAddress(uint32_t ip, uint16_t port);

    bool SetHighPassFilter(uint8_t type);
    bool SetFFTWindowForm(uint8_t type);
    bool SetNoiseThresholdMethod(uint8_t type);

    bool ManualAcquireTaget(int16_t id, uint32_t range_m, double bearing_deg, int bearingType);
    void ManualDataReceived(const char* address, const int port, const char* pBuff, const int len);

    bool ReplayImageData(int type, const char *pBuff, int len);

private:
    tImageClientImpl *m_pImpl;
};

} //NaviRadar
#endif // IMAGECLIENT_H
