/* mimalloc/cxl.h
 *
 * Cross-process shared arenas for CXL memory pools.
 *
 * Two public functions are provided:
 *
 *   mi_manage_os_memory_shared()
 *       Co-located metadata and data — both live in the same mmap region.
 *       Use this for the HWcc arena where everything is in coherent memory.
 *
 *   mi_manage_os_memory_shared_disjoint()
 *       Disjoint metadata and data — metadata lives in one region (e.g.
 *       HWcc memory for coherence), data blocks live in another (e.g. SWcc
 *       memory).  Use this for the SWcc arena.
 *
 * -------------------------------------------------------------------------
 * Typical two-arena setup
 * -------------------------------------------------------------------------
 *
 *   // 1. Map HWcc region (hardware-coherent CXL memory).
 *   void* hwcc = mmap(HWCC_VA, HWCC_SIZE, PROT_READ|PROT_WRITE,
 *                     MAP_SHARED|MAP_FIXED_NOREPLACE, hwcc_fd, 0);
 *
 *   // 2. Map SWcc region (software-managed-coherent CXL memory).
 *   void* swcc = mmap(SWCC_VA, SWCC_SIZE, PROT_READ|PROT_WRITE,
 *                     MAP_SHARED|MAP_FIXED_NOREPLACE, swcc_fd, 0);
 *
 *   // 3. Register arenas — ORDER MUST BE IDENTICAL IN EVERY PROCESS.
 *
 *   mi_arena_id_t hwcc_id;
 *   mi_manage_os_memory_shared(hwcc, HWCC_SIZE, &hwcc_id);
 *
 *   mi_arena_id_t swcc_id;
 *   size_t meta_needed = mi_cxl_meta_region_size(SWCC_SIZE);
 *   mi_manage_os_memory_shared_disjoint(
 *       (uint8_t*)hwcc + HWCC_SIZE - meta_needed, meta_needed,
 *       swcc, SWCC_SIZE,
 *       &swcc_id);
 *
 *   // 4. Per thread: create heaps bound to each arena.
 *   mi_heap_t* hwcc_heap = mi_heap_new_in_arena(hwcc_id);
 *   mi_heap_t* swcc_heap = mi_heap_new_in_arena(swcc_id);
 *
 * -------------------------------------------------------------------------
 * Shared-memory layout — co-located
 * -------------------------------------------------------------------------
 *
 *   shm_base + 0                  mi_cxl_shm_header_t
 *   shm_base + sizeof(header)     mi_arena_t + bitmap arrays
 *   shm_base + header_size        allocation blocks
 *                                 (header_size rounded up to 64 MiB)
 *
 * -------------------------------------------------------------------------
 * Shared-memory layout — disjoint
 * -------------------------------------------------------------------------
 *
 *   meta_base + 0                 mi_cxl_shm_header_t
 *   meta_base + sizeof(header)    mi_arena_t + bitmap arrays
 *   (header_size NOT rounded)
 *
 *   data_base + 0                 allocation block 0
 *   data_base + N*64MiB           allocation block N
 *
 * -------------------------------------------------------------------------
 * CONSTRAINTS
 * -------------------------------------------------------------------------
 *
 *   Fixed addresses: every process must mmap each region at the SAME
 *     virtual address.  The header stores both VAs for validation on attach.
 *
 *   Slot ordering: registration calls must happen in the SAME ORDER in every
 *     process, before any allocation that would implicitly create an arena.
 *
 *   No crash recovery: a process crash while holding a mimalloc-internal
 *     lock can leave the arena inconsistent.  Use CXLAlloc for crash safety.
 *
 *   abandoned_visit_lock: initialised as a process-private mutex by the
 *     first process.  mi_abandoned_visit_blocks() is NOT safe cross-process.
 *
 *   Purging disabled: memory is never decommitted or reset.
 */
 
#pragma once
#ifndef MIMALLOC_CXL_H
#define MIMALLOC_CXL_H
 
#include <mimalloc.h>
#include <stdint.h>
#include <stddef.h>
 
#ifdef __cplusplus
extern "C" {
#endif
 
/* Magic values written into the header to signal initialisation state.
 * Atomic operations on the magic field are performed only inside arena-cxl.c
 * using mimalloc's internal atomic layer.  Callers treat this struct as
 * read-only after mi_manage_os_memory_shared*() returns.                    */
#define MI_CXL_SHM_MAGIC_INIT  UINT64_C(0x4D494D43584C0000)  /* initialising */
#define MI_CXL_SHM_MAGIC       UINT64_C(0x4D494D43584C0001)  /* ready        */
 
/* Shared-memory header.  Always at offset 0 of the metadata region.
 *
 * magic is read/written with acquire/release semantics inside arena-cxl.c.
 * Declared as volatile uint64_t here so that callers can poll it if needed,
 * but callers should normally just call mi_manage_os_memory_shared*() and
 * let the library handle synchronisation.                                   */
typedef struct mi_cxl_shm_header_s {
  volatile uint64_t magic;       /* last field written; signals readiness     */
  size_t   header_size;          /* bytes consumed in the meta region         */
  size_t   block_count;          /* number of 64 MiB allocation blocks        */
  size_t   field_count;          /* bitmap words per array                    */
  uintptr_t meta_va;             /* expected VA of meta_base (validation)     */
  uintptr_t data_va;             /* expected VA of data_base (validation)     */
  size_t   data_size;            /* size of the data region                   */
} mi_cxl_shm_header_t;
 
 
/* -------------------------------------------------------------------------
   mi_cxl_meta_region_size
   -------------------------------------------------------------------------
   Returns the number of bytes required in the metadata region for a data
   region of `data_size` bytes.  Use this to carve a slice from HWcc memory
   before calling mi_manage_os_memory_shared_disjoint().                     */
mi_decl_export size_t mi_cxl_meta_region_size(size_t data_size) mi_attr_noexcept;
 
 
/* -------------------------------------------------------------------------
   mi_manage_os_memory_shared
   -------------------------------------------------------------------------
   Register a fixed-address shared mmap as a mimalloc arena where metadata
   and allocation blocks are co-located in the same region.
   Use for the HWcc arena.                                                   */
mi_decl_export bool mi_manage_os_memory_shared(
    void*          shm_base,
    size_t         shm_size,
    mi_arena_id_t* arena_id) mi_attr_noexcept;
 
 
/* -------------------------------------------------------------------------
   mi_manage_os_memory_shared_disjoint
   -------------------------------------------------------------------------
   Register a cross-process arena where metadata and data are in separate
   regions:
     meta_base / meta_size  — holds header + bitmaps (e.g. HWcc memory)
     data_base / data_size  — holds allocation blocks (e.g. SWcc memory)
 
   meta_size must be >= mi_cxl_meta_region_size(data_size).                 */
mi_decl_export bool mi_manage_os_memory_shared_disjoint(
    void*          meta_base,
    size_t         meta_size,
    void*          data_base,
    size_t         data_size,
    mi_arena_id_t* arena_id) mi_attr_noexcept;
 
 
/* Convenience: read-only view of the header at the base of a meta region.
   Valid only after the corresponding mi_manage_os_memory_shared*() call.    */
static inline const mi_cxl_shm_header_t*
mi_cxl_shm_header(const void* meta_base) {
  return (const mi_cxl_shm_header_t*)meta_base;
}
 
#ifdef __cplusplus
}
#endif
 
#endif /* MIMALLOC_CXL_H */
