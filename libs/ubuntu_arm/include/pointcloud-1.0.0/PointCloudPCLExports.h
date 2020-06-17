
#ifndef POINTCLOUDPCL_EXPORT_H
#define POINTCLOUDPCL_EXPORT_H

#ifdef POINTCLOUDPCL_STATIC_DEFINE
#  define POINTCLOUDPCL_EXPORT
#  define POINTCLOUDPCL_NO_EXPORT
#else
#  ifndef POINTCLOUDPCL_EXPORT
#    ifdef pointcloudpcl_EXPORTS
        /* We are building this library */
#      define POINTCLOUDPCL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define POINTCLOUDPCL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef POINTCLOUDPCL_NO_EXPORT
#    define POINTCLOUDPCL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef POINTCLOUDPCL_DEPRECATED
#  define POINTCLOUDPCL_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef POINTCLOUDPCL_DEPRECATED_EXPORT
#  define POINTCLOUDPCL_DEPRECATED_EXPORT POINTCLOUDPCL_EXPORT POINTCLOUDPCL_DEPRECATED
#endif

#ifndef POINTCLOUDPCL_DEPRECATED_NO_EXPORT
#  define POINTCLOUDPCL_DEPRECATED_NO_EXPORT POINTCLOUDPCL_NO_EXPORT POINTCLOUDPCL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef POINTCLOUDPCL_NO_DEPRECATED
#    define POINTCLOUDPCL_NO_DEPRECATED
#  endif
#endif

#endif
