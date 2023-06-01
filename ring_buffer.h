#include <algorithm> // for std::min

class CircularBuffer
{
public:
  CircularBuffer(size_t capacity);
  ~CircularBuffer();

  size_t size() const { return content; }
  size_t capacity() const { return buffer_size; }
  size_t filling() const {return content; }
  // Return number of bytes written.
  size_t write(const char *data, size_t bytes);
  // Return number of bytes read.
  size_t read(char *data, size_t bytes);

private:
  size_t tail, head, content, buffer_size;
  char *my_buffer;
};

CircularBuffer::CircularBuffer(size_t capacity)
  : tail(0)
  , head(0)
  , content(0)
  , buffer_size(capacity)
{
  my_buffer = new char[capacity];
}

CircularBuffer::~CircularBuffer()
{
  delete [] my_buffer;
}

size_t CircularBuffer::write(const char *source, size_t write_count)
{
  if (write_count == 0) return 0;

  size_t capacity = buffer_size;
  size_t bytes_to_write = std::min(write_count, capacity - content);

  // Write in a single step
  if (bytes_to_write <= capacity - head)
  {
    memcpy(my_buffer + head, source, bytes_to_write);
    head += bytes_to_write;
    if (head == capacity) head = 0;
  }
  // Write in two steps
  else
  {
    size_t size_1 = capacity - head;
    memcpy(my_buffer + head, source, size_1);
    size_t size_2 = bytes_to_write - size_1;
    memcpy(my_buffer, source + size_1, size_2);
    head = size_2;
  }

  content += bytes_to_write;
  return bytes_to_write;
}

size_t CircularBuffer::read(char *destination, size_t read_count)
{
  if (read_count == 0) return 0;

  size_t capacity = buffer_size;
  size_t bytes_to_read = std::min(read_count, content);

  // Read in a single step
  if (bytes_to_read <= capacity - tail)
  {
    memcpy(destination, my_buffer + tail, bytes_to_read);
    tail += bytes_to_read;
    if (tail == capacity) tail = 0;
  }
  // Read in two steps
  else
  {
    size_t size_1 = capacity - tail;
    memcpy(destination, my_buffer + tail, size_1);
    size_t size_2 = bytes_to_read - size_1;
    memcpy(destination + size_1, my_buffer, size_2);
    tail = size_2;
  }

  content -= bytes_to_read;
  return bytes_to_read;
}