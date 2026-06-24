
#ifdef BYTE_PACK_ON
#   error __FILE__ ": Packing was on...."
#endif
#define BYTE_PACK_ON

#if defined _MSC_VER
#   pragma warning( disable:4103 )   // disable warning for alignment changed
#   pragma pack(push)
#   pragma pack(1)
#	define BYTE_ALIGNED
#elif defined __MINGW32__
#   pragma pack(1)
#   define BYTE_ALIGNED
#elif defined __GNUC__
#	define BYTE_ALIGNED __attribute__ ((aligned (1),packed))
#else
#   error __FILE__ ": Need to define a pack method for this compiler"

#endif



