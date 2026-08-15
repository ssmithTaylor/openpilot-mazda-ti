#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <atomic>

#define DEFAULT_SEGMENT_SIZE (10 * 1024 * 1024)
// 32, not the upstream 15. Slots are never released when a subscriber closes, so the cap has to
// exceed the number of subscribers a queue will ever see in one boot, ghosts included. This fork
// runs more than 15 live readers on carState and frogpilotPlan (stock daemons plus the FrogPilot
// process, classic_modeld, two UI SubMasters, and the TI MCP server). At the cap, the next
// registration evicts every reader, and each of them silently drops the messages in flight on
// its next receive -- which surfaced as 20Hz daemons flipping carState not-alive once or twice a
// second, cascading through valid=False into a permanent commIssue while driving. Sizing rule:
// N > max live readers, so a transient subscriber costs at most one round instead of a storm.
#define NUM_READERS 32
#define ALIGN(n) ((n + (8 - 1)) & -8)

#define UNUSED(x) (void)x
#define UNPACK64(higher, lower, input) do {uint64_t tmp = input; higher = tmp >> 32; lower = tmp & 0xFFFFFFFF;} while (0)
#define PACK64(output, higher, lower) output = ((uint64_t)higher << 32) | ((uint64_t)lower & 0xFFFFFFFF)

struct  msgq_header_t {
  uint64_t num_readers;
  uint64_t write_pointer;
  uint64_t write_uid;
  uint64_t read_pointers[NUM_READERS];
  uint64_t read_valids[NUM_READERS];
  uint64_t read_uids[NUM_READERS];
};

struct msgq_queue_t {
  std::atomic<uint64_t> *num_readers;
  std::atomic<uint64_t> *write_pointer;
  std::atomic<uint64_t> *write_uid;
  std::atomic<uint64_t> *read_pointers[NUM_READERS];
  std::atomic<uint64_t> *read_valids[NUM_READERS];
  std::atomic<uint64_t> *read_uids[NUM_READERS];
  char * mmap_p;
  char * data;
  size_t size;
  int reader_id;
  uint64_t read_uid_local;
  uint64_t write_uid_local;

  bool read_conflate;
  std::string endpoint;
};

struct msgq_msg_t {
  size_t size;
  char * data;
};

struct msgq_pollitem_t {
  msgq_queue_t *q;
  int revents;
};

void msgq_wait_for_subscriber(msgq_queue_t *q);
void msgq_reset_reader(msgq_queue_t *q);

int msgq_msg_init_size(msgq_msg_t *msg, size_t size);
int msgq_msg_init_data(msgq_msg_t *msg, char * data, size_t size);
int msgq_msg_close(msgq_msg_t *msg);

int msgq_new_queue(msgq_queue_t * q, const char * path, size_t size);
void msgq_close_queue(msgq_queue_t *q);
void msgq_init_publisher(msgq_queue_t * q);
void msgq_init_subscriber(msgq_queue_t * q);

int msgq_msg_send(msgq_msg_t *msg, msgq_queue_t *q);
int msgq_msg_recv(msgq_msg_t *msg, msgq_queue_t *q);
int msgq_msg_ready(msgq_queue_t * q);
int msgq_poll(msgq_pollitem_t * items, size_t nitems, int timeout);

bool msgq_all_readers_updated(msgq_queue_t *q);
