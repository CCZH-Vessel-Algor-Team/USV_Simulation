#ifndef RADARSAVERANDREPLAYER_H
#define RADARSAVERANDREPLAYER_H

#include <ExportDll.h>
#include <string>

namespace NaviRadar {

enum eSaveReplayState {
    eIdle,
    eSaving,
    eReplaying,
};

class iSaveReplayObserver {
public:
    virtual ~iSaveReplayObserver(){}
    virtual void UpdateState(eSaveReplayState state) = 0;
    virtual void UpdateReplayData(double process, int type, const char* pBuff, int len) = 0;
};

class tRadarSaverAndReplayerPrivate;
class DLL_API tRadarSaverAndReplayer
{
public:
    tRadarSaverAndReplayer(iSaveReplayObserver *pObserver);
    virtual ~tRadarSaverAndReplayer();
public:
    eSaveReplayState State() const;
    // Save Radar Data
    bool StartSaving(const std::string& path);
    void SaveData(int type, const char* pBuff, int len);
    void EndSave();
    // Replay Radar Data
    bool StartReplaying(const std::string& path);
    bool Pause();
    bool Resume();
    bool Jump(double progressRate);// (0~1) 0起始位置，1结束位置
    void SetSpeed(double speedRate); // (0.1~10) 1正常速度，<1慢速，>1快速
    void EndReplay();
private:
    tRadarSaverAndReplayerPrivate* m_p;
    tRadarSaverAndReplayer(const tRadarSaverAndReplayer&) = delete;
    tRadarSaverAndReplayer& operator=(const tRadarSaverAndReplayer&) = delete;
};

}// namespace NaviRadar

#endif // RADARSAVERANDREPLAYER_H
