#ifndef MULTIRADARCLIENT_H
#define MULTIRADARCLIENT_H

#include <string>
#include <vector>
#include "ExportDll.h"

struct RadarAddress;

class iMultiRadarCheckerListener
{
public:
    virtual void OnLocalIpChanged(uint32_t ip) = 0;
};

class MultiRadarClientPrivate;
class DLL_API MultiRadarClient
{
public:
    static constexpr int sMaxSeialNumberSize = 24;
    static MultiRadarClient* getInstance();
    static void setInstance(MultiRadarClient* instance);
    MultiRadarClient();
    ~MultiRadarClient();
    void query();
    int  radarNum();
    void resetRadar();
    void queryForAdjust(const char* pSerialNumber);
    bool isRadarKeyValid(const char* pSerialNumber) const;
    int getRadarStreamCount(const char* pSerialNumber) const;
    const RadarAddress* getAddress(const char* pSerialNumber,int instance) const;
    std::vector<std::string> getRadar() const;
    int getRadar(char radars[][sMaxSeialNumberSize],int maxSize) const;
    void addListener(iMultiRadarCheckerListener* p);
    void removeListener(iMultiRadarCheckerListener* p);
    void setLocalIP(uint32_t addr);
private:
    MultiRadarClientPrivate*   d;
    MultiRadarClient(const MultiRadarClient&) = delete;
    MultiRadarClient& operator=(const MultiRadarClient&) = delete;
    static MultiRadarClient* m_instance;
};


#endif // MULTIRADARCLIENT_H
