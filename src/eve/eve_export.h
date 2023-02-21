
#ifndef EVE_EXPORT_H
#define EVE_EXPORT_H

#ifdef EVE_STATIC_DEFINE
#define EVE_EXPORT
#define EVE_NO_EXPORT
#else
#ifndef EVE_EXPORT
#ifdef evedll_EXPORTS
/* We are building this library */
#define EVE_EXPORT __declspec(dllexport)
#else
/* We are using this library */
#define EVE_EXPORT __declspec(dllimport)
#endif
#endif

#ifndef EVE_NO_EXPORT
#define EVE_NO_EXPORT
#endif
#endif

#ifndef EVE_DEPRECATED
#define EVE_DEPRECATED __declspec(deprecated)
#endif

#ifndef EVE_DEPRECATED_EXPORT
#define EVE_DEPRECATED_EXPORT EVE_EXPORT EVE_DEPRECATED
#endif

#ifndef EVE_DEPRECATED_NO_EXPORT
#define EVE_DEPRECATED_NO_EXPORT EVE_NO_EXPORT EVE_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#ifndef EVE_NO_DEPRECATED
#define EVE_NO_DEPRECATED
#endif
#endif

#endif /* EVE_EXPORT_H */
