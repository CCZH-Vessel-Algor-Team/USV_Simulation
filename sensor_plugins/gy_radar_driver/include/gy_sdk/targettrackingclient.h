#ifndef MARPACLIENT_H
#define MARPACLIENT_H

#include <NavTypes.h>
#include <ExportDll.h>
#include <vector>
#include <string>
#include "TargetProtocol.h"


class MultiRadarChecker;

namespace NaviRadar {
namespace Target {

class iTargetTrackingClientObserver;
class iTargetTrackingClientStateObserver;

class TargetTrackingClientPrivate;
class DLL_API TargetTrackingClient
{
public:
    TargetTrackingClient();
    ~TargetTrackingClient();

    void addStateObserver(iTargetTrackingClientStateObserver* p);
    void removeStateObserver(iTargetTrackingClientStateObserver* p);
    void addClientObserver(iTargetTrackingClientObserver* p);
    void removeClientObserver(iTargetTrackingClientObserver* p);

    bool connect(const char *pSerialNumber, uint16_t instance);
    bool disconnect();

    ///! Set Auto Capture
    bool setAutoCapture( bool enable );

    //! Acquire a target
    //! \param id The client id for a target.(id range 0~199)
    //! \param range_m The range where the radar should look to start tracking, in metres.
    //! \param bearing_deg The bearing where the radar should look to start tracking, in
    //!     degrees.
    //! \param bearingType If eAbsolute, the bearing will be interpreted as true north
    //!     bearing, if eRelative the bearing will be interpreted as relative to the heading
    //!     of the vessel.
    //! \return  True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool acquireTaget( uint32_t id, uint32_t range_m, uint16_t bearing_deg, eBearingType bearingType );

    ///! Draw Tracking Area
    ///! areaId range 0~9
    bool drawTrackingAreaPoint( uint32_t areaId, uint32_t range_m, uint16_t bearing_deg, eBearingType bearingType );

    ///! Delete Tracking Area Point
    bool deleteTrackingAreaPoint( uint32_t areaId );



    //-----------------------------------------------------------------------------------
    //! Cancel a tracked target. Also required to release server-ID's of targets that have
    //! been lost.
    //! \param serverID  The id of the target to be cancelled.
    //! \return  True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool cancel( uint32_t serverID );

    //-----------------------------------------------------------------------------------
    //! Cancel tracking of all targets
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool cancelAll();

    //-----------------------------------------------------------------------------------
    //! Sets the vessel speed and direction for use by target-tracking.
    //! \param speedType Type of speed supplied: eSpeedOverGround (1), eWaterSpeed (2) is
    //!     the water speed. Only one kind should be provided
    //! \param speed_dmps Speed magnitude expressed in 10ths of a metre per second (ie.
    //!     deci-metres per second)
    //! \param directionType Type of direction supplied: eCoureOverGround (1); eHeadingMagnetic
    //!     (2); or eHeadingTrue (3)
    //! \param direction_deg Direction of the vessel expressed in degrees.
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool setBoatSpeed( eSpeedType speedType, uint32_t speed_dmps, eDirectionType directionType, uint32_t direction_deg );

    bool setTTMAbsolute( bool absolute );

    /////////////////////////////////////////////////////////////////////////////////////
    //  Queries
    /////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------------------------------
    //! Query all info from TargetTracking module
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool queryAll();

    //-----------------------------------------------------------------------------------
    //! Query all targets info from TargetTracking module
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool queryAllTargets();

    //-----------------------------------------------------------------------------------
    //! Query setup info from TargetTracking module
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool queryAlarmSetup();

    //-----------------------------------------------------------------------------------
    //! Query properties info from TargetTracking module
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool queryProperties();


    /////////////////////////////////////////////////////////////////////////////////////
    //  Configuration
    /////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------------------------------
    //! Sets the danger zone distance. Used in connection with the danger zone time (see
    //! SetDangerTime) to provide early warning of a possible collision with a target.
    //! \param range_m Danger distance in metres
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool setDangerDistance( uint32_t range_m );

    //-----------------------------------------------------------------------------------
    //! Sets the danger zone time. Used in connection with the danger zone distance (see
    //! SetDangerDistance) to provide early warning of a possible collision with a target.
    //! \param time_sec Danger time in seconds
    //! \return True if the operation was successful
    //-----------------------------------------------------------------------------------
    bool setDangerTime( uint32_t time_sec );

    bool addNewTrackingAreaGeoPoint(eTrackingAreaType type, uint16_t id, double lng, double lat);
    bool modifyTrackingAreaGeoPoint(eTrackingAreaType type, uint16_t id, int geoIndex, double lng, double lat);
    bool setTrackingAreaEntireGeoPoints(eTrackingAreaType type, uint16_t id, const std::vector<std::pair<double,double>>& lnglats);
    bool setTrackingAreaEnable(eTrackingAreaType type, uint16_t id, bool enable);
    bool deleteTrackingAreaPreviousGeoPoint(eTrackingAreaType type, uint16_t id);
    bool queryAllTrackingAreas();

    void ManualDataReceived(const char* address, const int port, const char* pBuff, const int len);
    bool ReplayTargetData(int type, const char *pBuff, int len);

private:
    TargetTrackingClientPrivate*   d;
};

}//namespace Target
}//namespace NaviRadar
#endif // MARPACLIENT_H
