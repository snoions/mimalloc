/* arena-cxl.c
 *
 * Cross-process shared arenas for CXL memory pools.
 *
 * MUST be #include-d from the BOTTOM of src/arena.c (after all static
 * function definitions), not compiled as a separate translation unit.
 *
 * Integration:
 *   1. cp arena-cxl.c   mimalloc/src/
 *   2. cp cxl.h         mimalloc/include/mimalloc/
 *   3. In src/arena.c:
 *        a) Near the top, after other #includes:
 *             #include "mimalloc/cxl.h"
 *        b) At the very bottom of the file:
 *             #define MI_IN_ARENA_C
 *             #include "arena-cxl.c"
 *             #undef MI_IN_ARENA_C
 *   4. In include/mimalloc.h, after mi_manage_os_memory_ex:
 *        mi_decl_export size_t mi_cxl_meta_region_size(size_t data_size) mi_attr_noexcept;
 *        mi_decl_export bool   mi_manage_os_memory_shared(void* shm_base, size_t shm_size, mi_arena_id_t* arena_id) mi_attr_noexcept;
 *        mi_decl_export bool   mi_manage_os_memory_shared_disjoint(void* meta_base, size_t meta_size, void* data_base, size_t data_size, mi_arena_id_t* arena_id) mi_attr_noexcept;
 */
 
#ifndef MI_IN_ARENA_C
#error "arena-cxl.c must be #include-d from arena.c, not compiled directly"
#endif
 
/* ============================================================
   Internal layout helpers
   ============================================================
 
   Co-located layout (mi_manage_os_memory_shared):
 
     meta_base == shm_base:
       [0 .. sizeof(hdr))                  mi_cxl_shm_header_t
       [sizeof(hdr) .. sizeof(hdr)+meta)   mi_arena_t + bitmap overflow
                                           + blocks_dirty + blocks_abandoned
       [header_size .. header_size+blocks) allocation blocks
       header_size = align_up(sizeof(hdr)+meta, MI_ARENA_BLOCK_SIZE)
 
   Disjoint layout (mi_manage_os_memory_shared_disjoint):
 
     meta_base (e.g. a slice of HWcc memory):
       [0 .. sizeof(hdr))                  mi_cxl_shm_header_t
       [sizeof(hdr) .. sizeof(hdr)+meta)   mi_arena_t + bitmap overflow
                                           + blocks_dirty + blocks_abandoned
       header_size = sizeof(hdr) + meta    (NOT block-rounded)
 
     data_base (e.g. SWcc memory):
       [0 .. block_count * MI_ARENA_BLOCK_SIZE)   allocation blocks
*/
 
/* Bytes for the arena struct plus its bitmap storage.
 *
 * In v3.2.8, blocks_inuse is mi_bitmap_field_t blocks_inuse[1] embedded
 * at the END of mi_arena_t (verified: offsetof=184, sizeof=192).
 * arena->blocks_inuse[0] is already inside the struct; indices [1..field_count-1]
 * extend immediately past sizeof(mi_arena_t) as overflow words.
 *
 * blocks_committed, blocks_purge, blocks_dirty, blocks_abandoned are plain
 * mi_bitmap_field_t* pointers stored earlier in the struct; their arrays
 * follow the inuse overflow in the meta region.
 *
 * Total extra storage = (field_count-1) inuse overflow
 *                     + 4 * field_count for the four pointer-based arrays
 *                     = 5*field_count - 1 words
 */
static size_t mi_cxl_arena_meta_size(size_t field_count) {
  const size_t overflow = (field_count > 0) ? (field_count - 1) : 0;
  return sizeof(mi_arena_t)
       + overflow    * sizeof(mi_bitmap_field_t)  /* blocks_inuse overflow */
       + field_count * sizeof(mi_bitmap_field_t)  /* blocks_committed      */
       + field_count * sizeof(mi_bitmap_field_t)  /* blocks_purge          */
       + field_count * sizeof(mi_bitmap_field_t)  /* blocks_dirty          */
       + field_count * sizeof(mi_bitmap_field_t); /* blocks_abandoned      */
}
 
/* Total bytes consumed in the metadata region.
 * For co-located: rounded up to MI_ARENA_BLOCK_SIZE so blocks are aligned.
 * For disjoint:   not rounded (data_base alignment is the caller's concern). */
static size_t mi_cxl_header_size_colocated(size_t field_count) {
  return _mi_align_up(
      sizeof(mi_cxl_shm_header_t) + mi_cxl_arena_meta_size(field_count),
      MI_ARENA_BLOCK_SIZE);
}
 
static size_t mi_cxl_header_size_disjoint(size_t field_count) {
  return sizeof(mi_cxl_shm_header_t) + mi_cxl_arena_meta_size(field_count);
}
 
/* ============================================================
   Public sizing helper
   ============================================================ */
 
size_t mi_cxl_meta_region_size(size_t data_size) {
  /* Assume worst-case: data_base is 1 byte past a block boundary, so we
   * lose up to (MI_ARENA_BLOCK_SIZE - 1) bytes to alignment.  Callers
   * don't need to know their data_base when computing the meta size.       */
  const size_t worst_case = data_size > MI_ARENA_BLOCK_SIZE
                              ? data_size - MI_ARENA_BLOCK_SIZE : 0;
  const size_t block_count = worst_case / MI_ARENA_BLOCK_SIZE;
  const size_t field_count = (block_count > 0)
      ? _mi_divide_up(block_count, MI_BITMAP_FIELD_BITS) : 1;
  return _mi_align_up(mi_cxl_header_size_disjoint(field_count), 64);
}
 
/* ============================================================
   Co-located layout computation (two-pass, shared region)
   ============================================================ */
 
static bool mi_cxl_compute_layout_colocated(size_t  shm_size,
                                              size_t* out_block_count,
                                              size_t* out_field_count,
                                              size_t* out_hdr_size)
{
  /* Pass 1: upper-bound from total size */
  size_t block_count = shm_size / MI_ARENA_BLOCK_SIZE;
  if (block_count < 2) return false;
  size_t field_count = _mi_divide_up(block_count, MI_BITMAP_FIELD_BITS);
  size_t hdr_size    = mi_cxl_header_size_colocated(field_count);
  if (hdr_size >= shm_size) return false;
 
  /* Pass 2: recompute with actual data area */
  block_count = (shm_size - hdr_size) / MI_ARENA_BLOCK_SIZE;
  if (block_count == 0) return false;
  field_count = _mi_divide_up(block_count, MI_BITMAP_FIELD_BITS);
  hdr_size    = mi_cxl_header_size_colocated(field_count);
  if (hdr_size >= shm_size) return false;
 
  *out_block_count = block_count;
  *out_field_count = field_count;
  *out_hdr_size    = hdr_size;
  return true;
}
 
/* ============================================================
   Disjoint layout computation (single-pass, separate regions)
   ============================================================ */
 
static bool mi_cxl_compute_layout_disjoint(size_t  meta_size,
                                             size_t  data_size,
                                             uintptr_t data_base_addr,
                                             size_t* out_block_count,
                                             size_t* out_field_count,
                                             size_t* out_hdr_size)
{
  /* Skip any partial block at the base of the data region so that
   * arena->start is always block-aligned.                                   */
  const uintptr_t aligned = _mi_align_up(data_base_addr, MI_ARENA_BLOCK_SIZE);
  if (aligned >= data_base_addr + data_size) return false; /* no room at all */
  const size_t effective_size = data_size - (aligned - data_base_addr);
 
  const size_t block_count = effective_size / MI_ARENA_BLOCK_SIZE;
  if (block_count == 0) return false;
  const size_t field_count = _mi_divide_up(block_count, MI_BITMAP_FIELD_BITS);
  const size_t hdr_size    = mi_cxl_header_size_disjoint(field_count);
  if (hdr_size > meta_size) return false;
 
  *out_block_count = block_count;
  *out_field_count = field_count;
  *out_hdr_size    = hdr_size;
  return true;
}
 
/* ============================================================
   Process-local arena table registration
   ============================================================ */
 
static bool mi_cxl_arena_register(mi_arena_t*    arena,
                                   size_t         required_slot,
                                   mi_arena_id_t* arena_id)
{
  if (mi_unlikely(required_slot >= MI_MAX_ARENAS)) {
    _mi_error_message(ENOMEM,
      "cxl-arena: required slot %zu >= MI_MAX_ARENAS (%d)\n"
      "  Call mi_manage_os_memory_shared*() before any other arena creation.\n",
      required_slot, MI_MAX_ARENAS);
    return false;
  }
 
  if (mi_unlikely(
        mi_atomic_load_ptr_acquire(mi_arena_t, &mi_arenas[required_slot]) != NULL))
  {
    _mi_error_message(EINVAL,
      "cxl-arena: slot %zu is already occupied.\n"
      "  Call mi_manage_os_memory_shared*() before any other arena creation.\n",
      required_slot);
    return false;
  }
 
  size_t current = mi_atomic_load_relaxed(&mi_arena_count);
  while (current <= required_slot) {
    if (mi_atomic_cas_strong_acq_rel(&mi_arena_count, &current, required_slot + 1))
      break;
  }
 
  mi_atomic_store_ptr_release(mi_arena_t, &mi_arenas[required_slot], arena);
  *arena_id = mi_arena_id_create(required_slot);
  return true;
}
 
/* ============================================================
   Shared internal implementation
   ============================================================
   meta_base / meta_size : region that holds the header + arena + bitmaps
   data_base / data_size : region that holds the allocation blocks
   block_count, field_count, hdr_size: pre-computed by the caller
*/
 
static bool mi_cxl_init_shared(void*  meta_base, size_t meta_size,
                                void*  data_base, size_t data_size,
                                size_t block_count,
                                size_t field_count,
                                size_t hdr_size,
                                mi_arena_id_t* arena_id)
{
  (void)meta_size; /* checked by caller; not needed further */
 
  uint8_t* const base     = (uint8_t*)meta_base;
  mi_cxl_shm_header_t* const shm_hdr = (mi_cxl_shm_header_t*)base;
  mi_arena_t*          const arena   =
      (mi_arena_t*)(base + sizeof(mi_cxl_shm_header_t));
 
  /* Bitmap layout immediately after the struct:
   *
   *   [sizeof(mi_arena_t)]
   *   blocks_inuse[1..field_count-1]   overflow words  (field_count-1 words)
   *   blocks_committed[0..field_count-1]               (field_count words)
   *   blocks_purge    [0..field_count-1]               (field_count words)
   *   blocks_dirty    [0..field_count-1]               (field_count words)
   *   blocks_abandoned[0..field_count-1]               (field_count words)
   *
   * blocks_inuse[0] is already embedded in the struct at offset 184.
   * Do NOT assign arena->blocks_inuse — it is an array type, not a pointer.
   */
  const size_t inuse_overflow = (field_count > 0) ? (field_count - 1) : 0;
  mi_bitmap_field_t* const after_struct =
      (mi_bitmap_field_t*)((uint8_t*)arena + sizeof(mi_arena_t));
  mi_bitmap_field_t* const p_committed = after_struct + inuse_overflow;
  mi_bitmap_field_t* const p_purge     = p_committed + field_count;
  mi_bitmap_field_t* const p_dirty     = p_purge     + field_count;
  mi_bitmap_field_t* const p_abandoned = p_dirty     + field_count;
 
  /* Align the data start up to a full arena block boundary.
   * data_base may not be block-aligned if the mapped region has metadata
   * (e.g. system metadata) at its base.  We skip the partial first block
   * rather than letting mimalloc write segment headers into live metadata.
   * block_count was already computed from the aligned view by
   * mi_cxl_compute_layout_disjoint, so no adjustment is needed there.     */
  uint8_t* const data_start =
      (uint8_t*)_mi_align_up((uintptr_t)data_base, MI_ARENA_BLOCK_SIZE);
 
  /* ================================================================
     Election: CAS on magic to elect exactly one first-initialiser.
     ================================================================ */
  uint64_t expected_zero = 0;
  const bool i_am_first  =
      mi_atomic_cas_strong_acq_rel(&shm_hdr->magic,
                                   &expected_zero,
                                   MI_CXL_SHM_MAGIC_INIT);
 
  if (i_am_first) {
    /* ------------------------------------------------------------------
       FIRST PROCESS: zero header, populate arena struct, publish.
       ------------------------------------------------------------------ */
    _mi_memzero(base, hdr_size);
 
    mi_atomic_store_ptr_release(uint8_t, &arena->start, data_start);
 
    arena->block_count = block_count;
    arena->field_count = field_count;
    arena->meta_size   = mi_cxl_arena_meta_size(field_count);
    arena->meta_memid  = _mi_memid_create(MI_MEM_STATIC);
    arena->numa_node   = -1;
    arena->exclusive   = true;
    arena->is_large    = true;  /* pinned; disables commit/decommit paths */
 
    mi_lock_init(&arena->abandoned_visit_lock);
    mi_atomic_store_relaxed(&arena->search_idx, (size_t)0);
    mi_atomic_store_relaxed(&arena->purge_expire, (mi_msecs_t)INT64_MAX);
 
    /* blocks_inuse: do not assign — embedded array, already at &arena->blocks_inuse[0].
     * Overflow words [1..field_count-1] are zeroed by the _mi_memzero above. */
    arena->blocks_committed = p_committed;
    arena->blocks_purge     = p_purge;
    arena->blocks_dirty     = p_dirty;
    arena->blocks_abandoned = p_abandoned;
 
    arena->memid                     = _mi_memid_create(MI_MEM_STATIC);
    arena->memid.initially_committed = true;
    arena->memid.initially_zero      = true;
    arena->memid.is_pinned           = true;
 
    const size_t my_slot = mi_atomic_load_relaxed(&mi_arena_count);
    arena->id = mi_arena_id_create(my_slot);
 
    // and claim leftover blocks if needed (so we never allocate there)
    ptrdiff_t post = (field_count * MI_BITMAP_FIELD_BITS) - block_count;
    mi_assert_internal(post >= 0);
    if (post > 0) {
      // don't use leftover bits at the end
      mi_bitmap_index_t postidx = mi_bitmap_index_create(field_count - 1, MI_BITMAP_FIELD_BITS - post);
      _mi_bitmap_claim(arena->blocks_inuse, field_count, post, postidx, NULL);
    }

    if (!mi_cxl_arena_register(arena, my_slot, arena_id)) {
      mi_atomic_store_release(&shm_hdr->magic, UINT64_C(0));
      return false;
    }
 
    /* Fill header fields — all must be written before publishing magic. */
    shm_hdr->header_size = hdr_size;
    shm_hdr->block_count = block_count;
    shm_hdr->field_count = field_count;
    shm_hdr->meta_va     = (uintptr_t)meta_base;
    shm_hdr->data_va     = (uintptr_t)data_base;
    shm_hdr->data_size   = data_size;
 
    /* PUBLISH */
    mi_atomic_store_release(&shm_hdr->magic, MI_CXL_SHM_MAGIC);
 
  } else {
    /* ------------------------------------------------------------------
       SUBSEQUENT PROCESS: wait for init, validate, attach.
       ------------------------------------------------------------------ */
    uint64_t m;
    do {
      m = mi_atomic_load_acquire(&shm_hdr->magic);
    } while (m == MI_CXL_SHM_MAGIC_INIT);
 
    if (mi_unlikely(m != MI_CXL_SHM_MAGIC)) {
      _mi_error_message(EINVAL,
        "cxl-arena: unexpected magic 0x%llx — region uninitialised or corrupt\n",
        (unsigned long long)m);
      return false;
    }
 
    /* Validate that both processes agree on the mapped VAs and data size.
     * A mismatch means MAP_FIXED landed at a different address in one of
     * the processes, which would cause silent pointer corruption.          */
    if (mi_unlikely((uintptr_t)meta_base != shm_hdr->meta_va)) {
      _mi_error_message(EINVAL,
        "cxl-arena: meta_base VA mismatch: header=0x%zx caller=0x%zx\n"
        "  All processes must mmap the meta region at the same address.\n",
        (size_t)shm_hdr->meta_va, (size_t)(uintptr_t)meta_base);
      return false;
    }
    if (mi_unlikely((uintptr_t)data_base != shm_hdr->data_va)) {
      _mi_error_message(EINVAL,
        "cxl-arena: data_base VA mismatch: header=0x%zx caller=0x%zx\n"
        "  All processes must mmap the data region at the same address.\n",
        (size_t)shm_hdr->data_va, (size_t)(uintptr_t)data_base);
      return false;
    }
    if (mi_unlikely(data_size != shm_hdr->data_size)) {
      _mi_error_message(EINVAL,
        "cxl-arena: data_size mismatch: header=%zu caller=%zu\n",
        shm_hdr->data_size, data_size);
      return false;
    }
 
    const size_t required_slot = mi_arena_id_index(arena->id);
    if (!mi_cxl_arena_register(arena, required_slot, arena_id)) return false;
  }
 
  return true;
}
 
/* ============================================================
   Public entry points
   ============================================================ */
 
/* Co-located: metadata and data in the same shared region. */
bool mi_manage_os_memory_shared(void*          shm_base,
                                 size_t         shm_size,
                                 mi_arena_id_t* arena_id)
{
  if (mi_unlikely(shm_base == NULL || shm_size == 0 || arena_id == NULL)) {
    _mi_error_message(EINVAL, "cxl-arena: invalid arguments\n");
    return false;
  }
 
  size_t block_count, field_count, hdr_size;
  if (!mi_cxl_compute_layout_colocated(shm_size,
                                        &block_count, &field_count, &hdr_size))
  {
    const size_t min_fc  = _mi_divide_up(2, MI_BITMAP_FIELD_BITS);
    const size_t min_sz  = mi_cxl_header_size_colocated(min_fc)
                         + 2 * MI_ARENA_BLOCK_SIZE;
    _mi_error_message(EINVAL,
      "cxl-arena: shm_size %zu is too small (minimum ~%zu bytes)\n",
      shm_size, min_sz);
    return false;
  }
 
  /* For co-located: data starts immediately after the (block-aligned) header. */
  void* const data_base = (uint8_t*)shm_base + hdr_size;
  const size_t data_size = block_count * MI_ARENA_BLOCK_SIZE;
 
  return mi_cxl_init_shared(shm_base, shm_size,
                              data_base, data_size,
                              block_count, field_count, hdr_size,
                              arena_id);
}
 
/* Disjoint: metadata in meta_base (e.g. HWcc), data in data_base (e.g. SWcc). */
bool mi_manage_os_memory_shared_disjoint(void*          meta_base,
                                          size_t         meta_size,
                                          void*          data_base,
                                          size_t         data_size,
                                          mi_arena_id_t* arena_id)
{
  if (mi_unlikely(meta_base == NULL || meta_size == 0 ||
                  data_base == NULL || data_size == 0 || arena_id == NULL))
  {
    _mi_error_message(EINVAL, "cxl-arena: invalid arguments\n");
    return false;
  }
 
  size_t block_count, field_count, hdr_size;
  if (!mi_cxl_compute_layout_disjoint(meta_size, data_size,
                                       (uintptr_t)data_base,
                                       &block_count, &field_count, &hdr_size))
  {
    _mi_error_message(EINVAL,
      "cxl-arena: meta_size %zu too small for data_size %zu\n"
      "  Need at least %zu bytes; call mi_cxl_meta_region_size(data_size).\n",
      meta_size, data_size,
      mi_cxl_meta_region_size(data_size));
    return false;
  }
 
  return mi_cxl_init_shared(meta_base, meta_size,
                              data_base, data_size,
                              block_count, field_count, hdr_size,
                              arena_id);
}
