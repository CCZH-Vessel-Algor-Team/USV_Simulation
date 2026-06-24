
#ifndef BYTE_PACK_ON
#   error __FILE__ ": Packing wasn't on...."
#endif
#undef BYTE_PACK_ON

#ifdef _MSC_VER
#   pragma warning( disable : 4103 )   // disable warning for alignement changed
#   pragma pack(pop)
#	undef BYTE_ALIGNED
#elif defined __MINGW32__
#   pragma pack()
#   undef BYTE_ALIGNED
#elif defined __GNUC__
#	undef BYTE_ALIGNED

#else
#   error __FILE__ ": Need to define a pack method for this compiler"
#endif
