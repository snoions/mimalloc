/* cxl_example.c
 *
 * Two-arena cross-process CXL example:
 *   hwcc_arena — hardware-coherent memory; metadata and data co-located.
 *   swcc_arena — software-managed-coherent memory; metadata in HWcc memory,
 *                data blocks in SWcc memory.
 *
 * Build:
 *   gcc -O2 -o producer cxl_example.c -I../include \
 *       -Wl,-rpath,$(realpath ../build) -L../build -lmimalloc -lrt -pthread
 *   gcc -O2 -o consumer cxl_example.c -I../include \
 *       -Wl,-rpath,$(realpath ../build) -L../build -lmimalloc -lrt -pthread \
 *       -DCONSUMER
 *
 * Run:
 *   ./producer & sleep 0.1 && ./consumer
 */
 
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdatomic.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
 
#include <mimalloc.h>
#include <mimalloc/cxl.h>
 
/* -----------------------------------------------------------------------
   Fixed virtual addresses — must be identical in every process.
   ----------------------------------------------------------------------- */
#define HWCC_VA    ((void*)0x200000000000UL)
#define HWCC_SIZE  (4UL << 30)   /* 4 GiB HWcc region */
 
#define SWCC_VA    ((void*)0x300000000000UL)
#define SWCC_SIZE  (16UL << 30)  /* 16 GiB SWcc region */
 
/* POSIX shm names (stand-ins for real CXL DAX file descriptors) */
#define HWCC_SHM   "/cxl_hwcc"
#define SWCC_SHM   "/cxl_swcc"
 
/* -----------------------------------------------------------------------
   Control block: written by producer into the HWcc arena, read by consumer.
   Lives at a known offset (start of allocatable HWcc space, just past the
   mimalloc header).  In a real design use a proper channel.
   ----------------------------------------------------------------------- */
typedef struct {
  _Atomic(uint64_t) ready;
  uint64_t          hwcc_offset;   /* byte offset from HWCC_VA to hwcc buffer */
  uint64_t          swcc_offset;   /* byte offset from SWCC_VA to swcc buffer */
} cxl_ctrl_t;
 
/* -----------------------------------------------------------------------
   Helpers
   ----------------------------------------------------------------------- */
static void* shm_map(const char* name, size_t size, bool create, void* fixed_va) {
  int flags = create ? (O_RDWR | O_CREAT | O_TRUNC) : O_RDWR;
  int fd = shm_open(name, flags, 0600);
  if (fd < 0) { perror("shm_open"); exit(1); }
  if (create && ftruncate(fd, (off_t)size) < 0) { perror("ftruncate"); exit(1); }
  void* p = mmap(fixed_va, size, PROT_READ|PROT_WRITE,
                 MAP_SHARED|MAP_FIXED_NOREPLACE, fd, 0);
  if (p == MAP_FAILED || p != fixed_va) {
    fprintf(stderr, "mmap failed for %s at %p\n", name, fixed_va);
    exit(1);
  }
  close(fd);
  return p;
}
 
/* -----------------------------------------------------------------------
   Common init — MUST be called in the same order in every process.
   ----------------------------------------------------------------------- */
static void init_arenas(bool create,
                         mi_arena_id_t* hwcc_id,
                         mi_arena_id_t* swcc_id)
{
  void* hwcc = shm_map(HWCC_SHM, HWCC_SIZE, create, HWCC_VA);
  void* swcc = shm_map(SWCC_SHM, SWCC_SIZE, create, SWCC_VA);
 
  mi_option_set(mi_option_disallow_os_alloc, 1);
  mi_option_set(mi_option_verbose, 0);
 
  /* Arena 1: HWcc — metadata and data co-located in hwcc region. */
  if (!mi_manage_os_memory_shared(hwcc, HWCC_SIZE, hwcc_id)) {
    fprintf(stderr, "mi_manage_os_memory_shared (hwcc) failed\n"); exit(1);
  }
 
  /* Arena 2: SWcc — metadata carved from end of hwcc region, data in swcc.
   *
   * We carve the SWcc metadata from the END of the HWcc region so it doesn't
   * overlap with allocations from the HWcc arena.  In a real design you would
   * reserve a dedicated metadata stripe rather than borrowing from the end.
   *
   * Note: the HWcc arena is exclusive=false so mimalloc may use any part of
   * HWCC_SIZE for HWcc allocations.  To avoid collision, either:
   *   (a) use a separate dedicated HWcc region solely for SWcc metadata, or
   *   (b) pass exclusive=true to the HWcc arena and mmap a small separate
   *       HWcc window just for metadata.
   * This example uses approach (a) by borrowing the last meta_needed bytes
   * and relying on the application not filling the HWcc arena to capacity.
   */
  size_t meta_needed = mi_cxl_meta_region_size(SWCC_SIZE);
  void*  swcc_meta   = (uint8_t*)hwcc + HWCC_SIZE - meta_needed;
 
  if (!mi_manage_os_memory_shared_disjoint(
          swcc_meta, meta_needed,
          swcc, SWCC_SIZE,
          swcc_id))
  {
    fprintf(stderr, "mi_manage_os_memory_shared_disjoint (swcc) failed\n"); exit(1);
  }
 
  printf("[%s] hwcc_arena=%d  swcc_arena=%d  swcc_meta_needed=%zu\n",
         create ? "producer" : "consumer",
         *hwcc_id, *swcc_id, meta_needed);
}
 
/* -----------------------------------------------------------------------
   Producer
   ----------------------------------------------------------------------- */
#ifndef CONSUMER
int main(void) {
  mi_arena_id_t hwcc_id, swcc_id;
  init_arenas(true, &hwcc_id, &swcc_id);
 
  mi_heap_t* hwcc_heap = mi_heap_new_in_arena(hwcc_id);
  mi_heap_t* swcc_heap = mi_heap_new_in_arena(swcc_id);
 
  /* Allocate a small buffer in HWcc (coherent, low-latency) */
  char* hwcc_buf = mi_heap_malloc(hwcc_heap, 256);
  snprintf(hwcc_buf, 256, "HWcc data from producer pid=%d", getpid());
 
  /* Allocate a large buffer in SWcc (high-bandwidth, non-coherent) */
  const size_t big = 64 * 1024 * 1024; /* 64 MiB */
  char* swcc_buf = mi_heap_malloc(swcc_heap, big);
  memset(swcc_buf, 0xAB, big);
  printf("[producer] wrote 0xAB pattern to %zu MiB SWcc buffer\n", big >> 20);
 
  /* Publish pointers via a control block at the start of the HWcc arena's
   * allocatable region (just past the mimalloc header).                   */
  const mi_cxl_shm_header_t* hdr = mi_cxl_shm_header(HWCC_VA);
  cxl_ctrl_t* ctrl = (cxl_ctrl_t*)((uint8_t*)HWCC_VA + hdr->header_size);
  ctrl->hwcc_offset = (uint64_t)((uint8_t*)hwcc_buf - (uint8_t*)HWCC_VA);
  ctrl->swcc_offset = (uint64_t)((uint8_t*)swcc_buf - (uint8_t*)SWCC_VA);
  atomic_store_explicit(&ctrl->ready, 1ULL, memory_order_seq_cst);
 
  printf("[producer] published offsets hwcc=0x%llx swcc=0x%llx\n",
         (unsigned long long)ctrl->hwcc_offset,
         (unsigned long long)ctrl->swcc_offset);
 
  sleep(3);  /* keep alive while consumer runs */
 
  mi_heap_delete(hwcc_heap);
  mi_heap_delete(swcc_heap);
  shm_unlink(HWCC_SHM);
  shm_unlink(SWCC_SHM);
  printf("[producer] done\n");
  return 0;
}
 
/* -----------------------------------------------------------------------
   Consumer
   ----------------------------------------------------------------------- */
#else
int main(void) {
  mi_arena_id_t hwcc_id, swcc_id;
  init_arenas(false, &hwcc_id, &swcc_id);
 
  mi_heap_t* hwcc_heap = mi_heap_new_in_arena(hwcc_id);
  mi_heap_t* swcc_heap = mi_heap_new_in_arena(swcc_id);
 
  /* Wait for producer signal */
  const mi_cxl_shm_header_t* hdr = mi_cxl_shm_header(HWCC_VA);
  cxl_ctrl_t* ctrl = (cxl_ctrl_t*)((uint8_t*)HWCC_VA + hdr->header_size);
  printf("[consumer] waiting for producer...\n");
  while (atomic_load_explicit(&ctrl->ready, memory_order_seq_cst) == 0)
    usleep(1000);
 
  /* Reconstruct pointers from offsets */
  char* hwcc_buf = (char*)((uint8_t*)HWCC_VA + ctrl->hwcc_offset);
  char* swcc_buf = (char*)((uint8_t*)SWCC_VA + ctrl->swcc_offset);
 
  printf("[consumer] hwcc_buf: \"%s\"\n", hwcc_buf);
 
  /* Verify SWcc pattern */
  const size_t big = 64 * 1024 * 1024;
  bool ok = true;
  for (size_t i = 0; i < big; i++) {
    if ((unsigned char)swcc_buf[i] != 0xAB) { ok = false; break; }
  }
  printf("[consumer] swcc pattern check: %s\n", ok ? "PASS" : "FAIL");
 
  /* Free cross-process via xthread_free (v3 safe path) */
  mi_free(hwcc_buf);
  mi_free(swcc_buf);
  printf("[consumer] freed both buffers\n");
 
  mi_heap_delete(hwcc_heap);
  mi_heap_delete(swcc_heap);
  printf("[consumer] done\n");
  return 0;
}
#endif
