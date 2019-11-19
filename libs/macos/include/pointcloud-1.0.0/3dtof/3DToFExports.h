
#ifndef TOFCAM_EXPORT_H
#define TOFCAM_EXPORT_H

#ifdef TOFCDK_STATIC_DEFINE
#  define TOFCAM_EXPORT
#  define TOFCDK_NO_EXPORT
#else
#  ifndef TOFCAM_EXPORT
#    ifdef tofcdk_EXPORTS
        /* We are building this library */
#      define TOFCAM_EXPORT 
#    else
        /* We are using this library */
#      define TOFCAM_EXPORT 
#    endif
#  endif

#  ifndef TOFCDK_NO_EXPORT
#    define TOFCDK_NO_EXPORT 
#  endif
#endif

#ifndef TOFCDK_DEPRECATED
#  define TOFCDK_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef TOFCDK_DEPRECATED_EXPORT
#  define TOFCDK_DEPRECATED_EXPORT TOFCAM_EXPORT TOFCDK_DEPRECATED
#endif

#ifndef TOFCDK_DEPRECATED_NO_EXPORT
#  define TOFCDK_DEPRECATED_NO_EXPORT TOFCDK_NO_EXPORT TOFCDK_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef TOFCDK_NO_DEPRECATED
#    define TOFCDK_NO_DEPRECATED
#  endif
#endif

#endif
