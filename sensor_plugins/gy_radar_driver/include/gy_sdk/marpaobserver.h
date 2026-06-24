#ifndef MARPAOBSERVER_H
#define MARPAOBSERVER_H

#include <map>
#include <vector>
#include "TargetProtocol.h"

namespace NaviRadar {
namespace Target {

class iTargetTrackingClientObserver
{
public:
    virtual ~iTargetTrackingClientObserver() {}

    virtual void UpdateTarget( const tTrackedTarget* pTarget ) = 0;
    virtual void UpdateTargetString( const std::string& str ) = 0;
    virtual void UpdateBuffToSave( int type, const char *pBuff, int len ) = 0;
};

class iTargetTrackingClientStateObserver
{
public:
    virtual ~iTargetTrackingClientStateObserver() {}

    virtual void UpdateAlarmSetup( const tTargetTrackingAlarmSetup* pAlarmSetup ) = 0;
    virtual void UpdateProperties( const tTargetTrackingProperties* pProperties ) = 0;
    virtual void ClearAll() = 0;
    virtual void UpdateCaptureAreaInfo(std::map<tTrackingAreaInfo, std::vector<std::pair<double,double>>> info) = 0;
    virtual void UpdateShieldAreaInfo(std::map<tTrackingAreaInfo, std::vector<std::pair<double,double>>> info) = 0;
};

}//namespace Target
}//namespace NaviRadar
#endif // MARPAOBSERVER_H
