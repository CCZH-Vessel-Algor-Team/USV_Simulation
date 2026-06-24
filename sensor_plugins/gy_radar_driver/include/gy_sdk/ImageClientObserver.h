#ifndef IMAGECLIENTOBSERVER_H
#define IMAGECLIENTOBSERVER_H

#include "NavRadarProtocol.h"
#include "NavRadarSpoke.h"

namespace NaviRadar {

//-----------------------------------------------------------------------------
//! \ref tImageClient callback interface for observing radar image data
//-----------------------------------------------------------------------------
class iImageClientSpokeObserver
{
public:
    virtual ~iImageClientSpokeObserver() {}
    virtual void UpdateSpoke( const Spoke::SPOKE *pSpoke ) = 0;
    virtual void UpdateOriginalSpoke( const Spoke::tOriginalSpoke *pSpoke ) = 0;
    virtual void UpdateBuffToSave( int type, const char *pBuff, int len ) = 0;
};
//-----------------------------------------------------------------------------
//! \ref tImageClient callback interface for observing changes to a radars
//! state and settings
//-----------------------------------------------------------------------------
class iImageClientStateObserver
{
public:
    virtual ~iImageClientStateObserver() {}
    virtual void UpdateMode( const tMode* pMode ) = 0;
    virtual void UpdateSetup( const tSetup* pSetup ) = 0;
    virtual void UpdateAdvancedSetup(const tAdvancedSetup* pAdvancedSetup) = 0;
    virtual void UpdateSetupExtended( const tSetupExtended* pSetupExtended ) = 0;
    virtual void UpdateProperties( const tProperties* pProperties ) = 0;
    virtual void UpdateConfiguration( const tConfiguration* pConfiguration ) = 0;
    virtual void UpdateGuardZoneAlarm( const tGuardZoneAlarm* pAlarm ) = 0;
    virtual void UpdateRadarError( const tRadarError* pError ) = 0;
//    virtual void UpdateAdvancedState( const tAdvancedSTCState* pState ) = 0;
    virtual void UpdateProcessCfg( const tProcessConfiguration* pProcessCfg ) = 0;
    virtual void UpdateFactoryCfg( const tFactoryConfiguration* pFactoryCfg ) = 0;
    virtual void UpdateStcCfg( const tStcConfiguration* pStcCfg ) = 0;
    virtual void UpdateDebuggingPara( const tDebuggingPara* pDebugPara ) = 0;
    virtual void UpdateBoatInsInfo( const tBoatInsInfo* pDebugPara ) = 0;
};
}

#endif // IMAGECLIENTOBSERVER_H

