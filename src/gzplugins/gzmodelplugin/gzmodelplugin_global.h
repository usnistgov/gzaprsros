#ifndef GZMODELPLUGIN_GLOBAL_H
#define GZMODELPLUGIN_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(GZMODELPLUGIN_LIBRARY)
#  define GZMODELPLUGINSHARED_EXPORT Q_DECL_EXPORT
#else
#  define GZMODELPLUGINSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // GZMODELPLUGIN_GLOBAL_H
