#pragma once

#include <stdbool.h>
#include <stdint.h>

/// @brief A bounded queue that automatically removes its oldest elements when
/// its capacity is exceeded.
typedef struct ring_buffer_t {
  /// @brief Maximum number of elements in the buffer.
  uint16_t capacity;

  /// @brief Number of elements currently in the buffer.
  uint16_t size;

  /// @brief Index of the first element in the buffer.
  uint16_t start;

  /// @brief Backing memory block.
  float* data;

  /// @brief Running of all elements in the buffer.
  float sum;
} ring_buffer_t;

/// @brief Create a ring buffer and allocate memory for the buffer.
/// @param capacity The maximum number of elements for the buffer.
/// @return A pointer to a ring buffer or NULL if the system does not have
/// enough memory.
ring_buffer_t* ring_buffer_create(const uint16_t capacity);

/// @brief Free all memory allocated to a ring buffer created with
/// ring_buffer_create.
/// @param buf The ring buffer to destroy.
void ring_buffer_destroy(ring_buffer_t* buf);

/// @brief Remove the first element of a ring buffer.
/// @param buf The ring buffer.
/// @param result_out If the return value is true, this contains the value of
/// the first element of the ring buffer prior to removal.
/// @return true if an element was removed, false otherwise.
bool ring_buffer_pop(ring_buffer_t* buf, float* result_out);

/// @brief Add a value to the end of a ring buffer, removing the first element
/// of the ring buffer if it is full.
/// @param buf The ring buffer.
/// @param val The value to add to the ring buffer.
/// @return true if the value was added, false otherwise.
bool ring_buffer_push(ring_buffer_t* buf, const float val);

/// @brief Query the first element of a ring buffer without removing it.
/// @param buf The ring buffer.
/// @param head_out The first element of the ring buffer if it exists.
/// @return true if the ring buffer is nonempty, false otherwise.
bool ring_buffer_head(const ring_buffer_t* buf, float* head_out);

/// @brief Query the last element of a ring buffer without removing it.
/// @param buf The ring buffer.
/// @param tail_out The last element of the ring buffer if it exists.
/// @return true if the ring buffer is nonempty, false otherwise.
bool ring_buffer_tail(const ring_buffer_t* buf, float* tail_out);

/// @brief Calculate the sum of the elements in a ring buffer.
/// @param buf The ring buffer.
/// @param sum_out If the return value is true, this contains the sum of the
/// elements in the ring buffer.
/// @return true if the sum is valid, false otherwise.
bool ring_buffer_sum(const ring_buffer_t* buf, float* sum_out);

/// @brief Calculate the mean of the elements in a ring buffer.
/// @param buf The ring buffer.
/// @param mean_out If the return value is true, this contains the mean of the
/// elements in the ring buffer.
/// @return true if the mean is valid, false otherwise.
bool ring_buffer_mean(const ring_buffer_t* buf, float* mean_out);
