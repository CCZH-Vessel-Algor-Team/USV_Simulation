#ifndef NAVRADARSPOKE_H
#define NAVRADARSPOKE_H

#include "NavTypes.h"

namespace NaviRadar {
namespace Spoke {

#include "BytePackOn.h"

//------------------------------------------------------------------------
// Spoke
//------------------------------------------------------------------------

#define SAMPLES_PER_SPOKE  1024

//------------------------------------------------------------------------------
//! Structure for conveying radar image data header information
//------------------------------------------------------------------------------
struct tagSPOKE_HEADER
{
    uint32_t spokeLength_bytes : 12;    //!< length of the whole spoke in bytes
    uint32_t : 4;                       //!< reserved
    uint32_t sequenceNumber : 12;       //!< spoke sequence number
    uint32_t : 4;                       //!< reserved

    uint32_t nOfSamples : 12;           //!< number of samples present in the spoke
    uint32_t bitsPerSample: 4;          //!< number of bits per sample, normally is set to 4
    uint32_t rangeCellSizeDiv2_mm : 16; //!< Distance divided by 2 represented by each range-cell. sample size is computed as: rangeCellSize_mm * 2*rangeCellsDiv2 / nOfSamples;

    uint32_t spokeAzimuth: 13;          //!< Azimuth of the spoke in the range 0-4095. Values greater than 4095 must be mapped to 4095. This represents a full circle 0-360 degrees
    uint32_t : 1;                       //!< reserved
    uint32_t bearingZeroError: 1;       //!< Set if there is malfunctioning bearing zero
    uint32_t replayFlag : 1;            //!< reserved
    uint32_t spokeCompass: 14;          //!< Heading of the boat when this spoke was sampled. It is represented in the 0-4095 range for 0-360degrees of heading
    uint32_t trueNorth : 1;             //!< The connected heading sensor is reporting true north (1) or magnetic north (0)
    uint32_t compassInvalid : 1;        //!< If this bit is 1, the compass information are invalid

    uint32_t rangeCells : 16;           //!< Number of range-cells represented by the data in this spoke
    uint32_t : 16;
    uint64_t hwTimestamp : 48;          //!< 1. hw data obtain time 2. pcap rec timestamp [pcap replay]
    uint64_t fftADFlag : 1;             //!< 0 fft 1 AD
    uint64_t : 15;
} BYTE_ALIGNED;
typedef tagSPOKE_HEADER SPOKE_HEADER, *PSPOKE_HEADER;
//------------------------------------------------------------------------------
//! Structure for conveying radar image data & header information (ie. spokes)
//------------------------------------------------------------------------------
struct SPOKE
{
    SPOKE_HEADER header;
    uint8_t data[ SAMPLES_PER_SPOKE/2 ];
} BYTE_ALIGNED;

struct OriginalSpoke {
    SPOKE_HEADER header;
    uint8_t data[ 4*512 ];
} BYTE_ALIGNED;

typedef struct OriginalSpoke tOriginalSpoke;

//------------------------------------------------------------------------------
//! Calculates the physical distance each sample in the spoke represents.
//! \param pSpoke  Non-null pointer to a valid spoke
//! \returns The number of milli-metres represented by each sample in the spoke
//------------------------------------------------------------------------------
inline uint32_t GetSampleRange_mm( const SPOKE_HEADER& pSpoke )
{
    return (uint32_t(pSpoke.rangeCellSizeDiv2_mm) * pSpoke.rangeCells) / (pSpoke.nOfSamples >> 1);
}

/// For backward compatability only, use GetSampleRange_mm instead
inline uint32_t GetPixelCellSize_mm( const SPOKE_HEADER& pSpoke ) { return GetSampleRange_mm( pSpoke ); }

//------------------------------------------------------------------------------
//! Calculates the total distance represented by all the samples in the spoke.
//! \param pSpoke  Non-null pointer to a valid spoke
//! \returns The total number of millimetres represented by the spoke
//------------------------------------------------------------------------------
inline uint32_t GetSpokeRange_mm( const SPOKE_HEADER& pSpoke )
{
    return (uint32_t(pSpoke.rangeCellSizeDiv2_mm) * 2 * pSpoke.rangeCells);
}

//-----------------------------------------------------------------------------
#include "BytePackOff.h"

} //Spoke
} //NaviRadar
#endif // NAVRADARSPOKE_H
