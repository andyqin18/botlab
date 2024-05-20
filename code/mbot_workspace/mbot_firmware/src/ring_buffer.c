#include "ring_buffer.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

ring_buffer_t* ring_buffer_create(const uint16_t capacity) {
  ring_buffer_t* ring_buffer = (ring_buffer_t*)malloc(sizeof(ring_buffer_t));

  if (!ring_buffer) {
    fprintf(stderr, "ERROR (%s:%u): failed to allocate %lu bytes", __FILE__, __LINE__,
            sizeof(ring_buffer));
    return ring_buffer;
  }

  ring_buffer->capacity = capacity;
  ring_buffer->start = 0;
  ring_buffer->size = 0;
  ring_buffer->sum = 0.0f;
  ring_buffer->data = (float*)calloc(capacity, sizeof(float));

  if (!ring_buffer->data) {
    ring_buffer->capacity = 0;
    fprintf(stderr, "ERROR (%s:%u): failed to allocate %lu bytes", __FILE__, __LINE__,
            capacity * sizeof(float));
  }

  return ring_buffer;
}

void ring_buffer_destroy(ring_buffer_t* buf) {
  if (buf) {
    free(buf->data);
    free(buf);
  }
}

bool ring_buffer_pop(ring_buffer_t* buf, float* result_out) {
  if (!buf || buf->size == 0) {
    return false;
  }

  if (result_out) {
    *result_out = buf->data[buf->start];
  }
  buf->sum -= buf->data[buf->start];
  buf->start = (buf->start + 1) % buf->capacity;
  --buf->size;
  return true;
}

bool ring_buffer_push(ring_buffer_t* buf, const float val) {
  if (!buf || buf->size == buf->capacity && !ring_buffer_pop(buf, NULL)) {
    // capacity is 0, so we can't append anything!
    return false;
  }

  ++buf->size;
  buf->data[(buf->start + buf->size - 1) % buf->capacity] = val;
  buf->sum += val;

  return true;
}

bool ring_buffer_head(const ring_buffer_t* buf, float* head_out) {
  if (!buf || buf->size == 0) {
    return false;
  }

  *head_out = buf->data[buf->start];
  return true;
}

bool ring_buffer_tail(const ring_buffer_t* buf, float* tail_out) {
  if (!buf || buf->size == 0) {
    return false;
  }

  *tail_out = buf->data[(buf->start + buf->size - 1) % buf->capacity];
  return true;
}

bool ring_buffer_sum(const ring_buffer_t* buf, float* sum_out) {
  if (!buf || buf->size == 0) {
    return false;
  }

  *sum_out = buf->sum;
  return true;
}

bool ring_buffer_mean(const ring_buffer_t* buf, float* mean_out) {
  if (!buf || buf->size == 0) {
    return false;
  }

  *mean_out = buf->sum / (float)buf->size;
  return true;
}
