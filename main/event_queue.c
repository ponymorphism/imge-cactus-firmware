#include "event_queue.h"

struct event_list_head _lh;

unsigned char _isActive;

void event_queue_init() {
  TAILQ_INIT(&_lh);
}

unsigned char event_queue_is_empty() {
  return TAILQ_EMPTY(&_lh);
}

void event_queue_insert(struct event_entry* le) {
  if (event_queue_is_empty())
    TAILQ_INSERT_HEAD(&_lh, le, link);
  else
    TAILQ_INSERT_TAIL(&_lh, le, link);
  return;
}

struct event_entry* event_queue_pop_first() {
  struct event_entry* le = NULL;
  if (event_queue_is_empty())
    return NULL;

  le = TAILQ_FIRST(&_lh);
  TAILQ_REMOVE(&_lh, le, link);
  return le;
}

struct event_entry* event_queue_new_event() {
  return malloc(sizeof(struct event_entry));
}

extern void event_queue_free_event(struct event_entry* le) {
  if (le != NULL) {
    free(le);
  }
}

extern void event_queue_remove(struct event_entry* le) {
  if (le != NULL) {
    TAILQ_REMOVE(&_lh, le, link);
  }
}
