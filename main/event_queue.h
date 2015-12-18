#ifndef _EVENT_QUEUE_H
#define _EVENT_QUEUE_H _EVENT_QUEUE_H

#include <sys/queue.h>
#include <stdlib.h>

// for sysTime
#include "main.h"

#ifndef NULL
#define NULL (0L)
#endif /* NULL */

TAILQ_HEAD(event_list_head, event_entry);

struct event_entry {
  TAILQ_ENTRY(event_entry) link;

  unsigned long timestamp;
  unsigned char key;
};


extern void event_queue_init();
extern unsigned char event_queue_is_empty();
extern void event_queue_insert(struct event_entry* le);
extern struct event_entry* event_queue_pop_first();
extern struct event_entry* event_queue_new_event();
extern void event_queue_free_event(struct event_entry* le);
extern void event_queue_remove(struct event_entry* le);

#endif /* _EVENT_QUEUE_H */
