#ifndef NAVTYPES_H
#define NAVTYPES_H

#ifdef __GNUC__
    //-------------------------------------------------------------------------
    // Definition of basic datatypes for GNU compilers
    //-------------------------------------------------------------------------
#   include <stdint.h>
#elif defined _MSC_VER
    // cstdint and stdint.h are available for Visual Studio 2010 and later.
#   ifdef __cplusplus
#       include <cstdint>
#   else
#       include <stdint.h>
#   endif
#endif


#endif // NAVTYPES_H
