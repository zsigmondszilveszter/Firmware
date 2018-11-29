#pragma once

struct deque_entry_s {
	struct deque_entry_s *flink;
	struct deque_entry_s *blink;
};
typedef struct deque_entry_s deque_entry_t;

struct deque_queue_s {
	deque_entry_t *head;
	deque_entry_t *tail;
};
typedef struct deque_queue_s deque_queue_t;

void deque_addlast(deque_entry_t *node, deque_queue_t *queue);
void deque_rem(deque_entry_t *node, deque_queue_t *queue);
deque_entry_t *deque_remfirst(deque_queue_t *queue);
