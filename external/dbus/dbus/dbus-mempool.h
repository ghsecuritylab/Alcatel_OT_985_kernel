
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_MEMPOOL_H
#define DBUS_MEMPOOL_H

#include <dbus/dbus-internals.h>
#include <dbus/dbus-memory.h>
#include <dbus/dbus-types.h>

DBUS_BEGIN_DECLS

typedef struct DBusMemPool DBusMemPool;

DBusMemPool* _dbus_mem_pool_new     (int          element_size,
                                     dbus_bool_t  zero_elements);
void         _dbus_mem_pool_free    (DBusMemPool *pool);
void*        _dbus_mem_pool_alloc   (DBusMemPool *pool);
dbus_bool_t  _dbus_mem_pool_dealloc (DBusMemPool *pool,
                                     void        *element);

DBUS_END_DECLS

#endif /* DBUS_MEMPOOL_H */
