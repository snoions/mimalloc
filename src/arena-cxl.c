/* arena-cxl.c
 *
 * Cross-process shared arenas for CXL memory pools.
 *
 * MUST be #include-d from the BOTTOM of src/arena.c (after all static
 * function definitions), not compiled as a separate translation unit.

 * ============================================================
 * Shared-memory layout (both co-located and disjoint)
 * ============================================================
 *
 * The meta region contains:
 *
 *   [0]                     mi_cxl_shm_header_t
 *   [sizeof(hdr)]           mi_arena_t
 *   [sizeof(hdr)
 *    + sizeof(mi_arena_t)]  bitmap arrays, laid out exactly as in
 *                           mi_manage_os_memory_ex2:
 *
 *     &arena->blocks_inuse[0]            blocks_inuse  [0..fields-1]
 *     &arena->blocks_inuse[fields]       blocks_dirty  [0..fields-1]
 *     &arena->blocks_inuse[2*fields]     blocks_abandoned [0..fields-1]
 *     blocks_committed = NULL   (pinned memory — always committed)
 *     blocks_purge     = NULL   (pinned memory — never purged)
 *
 * Total bitmap storage = 3 * fields * sizeof(mi_bitmap_field_t)
 * (blocks_inuse[0] is embedded in mi_arena_t; [1..fields-1] are overflow)
 *
 * For co-located layout, the meta region is rounded up to MI_ARENA_BLOCK_SIZE
 * so that allocation blocks start on a block-aligned boundary.
 * For disjoint layout, the meta region is not rounded (data_base alignment
 * is the caller's concern).
 */

#ifndef MI_IN_ARENA_C
#error "arena-cxl.c must be #include-d from arena.c, not compiled directly"
#endif

/* ============================================================
   Layout helpers — canonical pattern from mi_manage_os_memory_ex2
   ============================================================ */

/* Bytes for the arena struct plus its 3 bitmap arrays (pinned memory).
 *
 * Matches: asize = sizeof(mi_arena_t) + (bitmaps * fields * sizeof(field))
 * where bitmaps = 3 for pinned (inuse, dirty, abandoned).
 * blocks_committed and blocks_purge are NULL for pinned CXL memory.
 *
 * blocks_inuse[0] is embedded in mi_arena_t; [1..fields-1] are overflow
 * words immediately following the struct — this is the same flexible-array
 * pattern used throughout arena.c.
 */
static size_t mi_cxl_arena_meta_size(size_t fields) {
  return sizeof(mi_arena_t) + 3 * fields * sizeof(mi_bitmap_field_t);
}

/* Total bytes consumed in the meta region.
 * Co-located: rounded up to MI_ARENA_BLOCK_SIZE (blocks must be aligned).
 * Disjoint:   not rounded (data_base alignment is caller's responsibility). */
static size_t mi_cxl_header_size_colocated(size_t fields) {
  return _mi_align_up(sizeof(mi_cxl_shm_header_t) + mi_cxl_arena_meta_size(fields),
                      MI_ARENA_BLOCK_SIZE);
}

static size_t mi_cxl_header_size_disjoint(size_t fields) {
  return sizeof(mi_cxl_shm_header_t) + mi_cxl_arena_meta_size(fields);
}

/* ============================================================
   Public sizing helper
   ============================================================ */

size_t mi_cxl_meta_region_size(size_t data_size) {
  /* Worst-case: data_base is 1 byte past a MI_SEGMENT_ALIGN boundary,
   * losing up to (MI_SEGMENT_ALIGN - 1) bytes to alignment.               */
  const size_t worst = (data_size > MI_SEGMENT_ALIGN)
                         ? data_size - MI_SEGMENT_ALIGN : 0;
  const size_t bcount = worst / MI_ARENA_BLOCK_SIZE;
  const size_t fields = (bcount > 0) ? _mi_divide_up(bcount, MI_BITMAP_FIELD_BITS) : 1;
  /* Align to cache line so carved slices don't false-share. */
  return _mi_align_up(mi_cxl_header_size_disjoint(fields), 64);
}

/* ============================================================
   Layout computation
   ============================================================ */

/* Align data_base up to MI_SEGMENT_ALIGN (matching mi_manage_os_memory_ex2
 * which aligns start to MI_SEGMENT_ALIGN, not MI_ARENA_BLOCK_SIZE).        */
static uintptr_t mi_cxl_align_data(uintptr_t data_base_addr) {
  return _mi_align_up(data_base_addr, MI_SEGMENT_ALIGN);
}

static bool mi_cxl_compute_layout_colocated(size_t  shm_size,
                                              size_t* out_bcount,
                                              size_t* out_fields,
                                              size_t* out_hdr_size)
{
  /* Pass 1: upper bound */
  size_t bcount = shm_size / MI_ARENA_BLOCK_SIZE;
  if (bcount < 2) return false;
  size_t fields   = _mi_divide_up(bcount, MI_BITMAP_FIELD_BITS);
  size_t hdr_size = mi_cxl_header_size_colocated(fields);
  if (hdr_size >= shm_size) return false;

  /* Pass 2: recompute with actual data size */
  bcount   = (shm_size - hdr_size) / MI_ARENA_BLOCK_SIZE;
  if (bcount == 0) return false;
  fields   = _mi_divide_up(bcount, MI_BITMAP_FIELD_BITS);
  hdr_size = mi_cxl_header_size_colocated(fields);
  if (hdr_size >= shm_size) return false;

  *out_bcount   = bcount;
  *out_fields   = fields;
  *out_hdr_size = hdr_size;
  return true;
}

static bool mi_cxl_compute_layout_disjoint(size_t    meta_size,
                                             size_t    data_size,
                                             uintptr_t data_base_addr,
                                             size_t*   out_bcount,
                                             size_t*   out_fields,
                                             size_t*   out_hdr_size)
{
  const uintptr_t aligned = mi_cxl_align_data(data_base_addr);
  if (aligned >= data_base_addr + data_size) return false;
  const size_t effective = data_size - (aligned - data_base_addr);

  const size_t bcount   = effective / MI_ARENA_BLOCK_SIZE;
  if (bcount == 0) return false;
  const size_t fields   = _mi_divide_up(bcount, MI_BITMAP_FIELD_BITS);
  const size_t hdr_size = mi_cxl_header_size_disjoint(fields);
  if (hdr_size > meta_size) return false;

  *out_bcount   = bcount;
  *out_fields   = fields;
  *out_hdr_size = hdr_size;
  return true;
}

/* ============================================================
   Shared internal implementation
   ============================================================ */

static bool mi_cxl_init_shared(void*  meta_base, size_t meta_size,
                                void*  data_base, size_t data_size,
                                size_t bcount, size_t fields, size_t hdr_size,
                                mi_arena_id_t* arena_id)
{
  (void)meta_size;

  uint8_t*             const base    = (uint8_t*)meta_base;
  mi_cxl_shm_header_t* const shm_hdr = (mi_cxl_shm_header_t*)base;
  mi_arena_t*          const arena   =
      (mi_arena_t*)(base + sizeof(mi_cxl_shm_header_t));

  /* Data start: align up to MI_SEGMENT_ALIGN, matching mi_manage_os_memory_ex2. */
  uint8_t* const data_start =
      (uint8_t*)mi_cxl_align_data((uintptr_t)data_base);

  /* ================================================================
     Election: CAS elects exactly one first-initialiser.
     ================================================================ */
  uint64_t expected_zero = 0;
  const bool i_am_first  =
      mi_atomic_cas_strong_acq_rel(&shm_hdr->magic,
                                   &expected_zero,
                                   MI_CXL_SHM_MAGIC_INIT);

  if (i_am_first) {
    /* ------------------------------------------------------------------
       FIRST PROCESS: matches mi_manage_os_memory_ex2 field-for-field,
       then uses mi_arena_add() exactly as the canonical path does.
       ------------------------------------------------------------------ */
    _mi_memzero(base, hdr_size);

    arena->id      = _mi_arena_id_none();
    arena->memid   = _mi_memid_create(MI_MEM_STATIC);
    arena->memid.initially_committed = true;
    arena->memid.initially_zero      = true;
    arena->memid.is_pinned           = true;

    arena->exclusive    = false;
    arena->is_large     = true;
    arena->numa_node    = -1;
    arena->block_count  = bcount;
    arena->field_count  = fields;
    arena->meta_size    = mi_cxl_arena_meta_size(fields);
    arena->meta_memid   = _mi_memid_create(MI_MEM_STATIC);

    arena->start        = data_start;
    arena->purge_expire = 0;
    arena->search_idx   = 0;
    mi_lock_init(&arena->abandoned_visit_lock);

    arena->blocks_dirty     = &arena->blocks_inuse[fields];
    arena->blocks_abandoned = &arena->blocks_inuse[2 * fields];
    arena->blocks_committed = NULL;  /* pinned: always committed, no tracking */
    arena->blocks_purge     = NULL;  /* pinned: never purged                  */
    ptrdiff_t post = (ptrdiff_t)(fields * MI_BITMAP_FIELD_BITS) - (ptrdiff_t)bcount;
    mi_assert_internal(post >= 0);
    if (post > 0) {
      mi_bitmap_index_t postidx = mi_bitmap_index_create(fields - 1,
                                      MI_BITMAP_FIELD_BITS - (size_t)post);
      _mi_bitmap_claim(arena->blocks_inuse, fields, (size_t)post, postidx, NULL);
    }

    /* Use mi_arena_add — same function the canonical path calls.
     * It atomically claims a slot, sets arena->id, updates stats,
     * and stores the pointer into mi_arenas[].                            */
    if (!mi_arena_add(arena, arena_id, &_mi_stats_main)) {
      mi_atomic_store_release(&shm_hdr->magic, UINT64_C(0));
      return false;
    }
    /* arena->id is now set by mi_arena_add; publish it before magic so
     * attaching processes can read it after their acquire load.           */

    shm_hdr->header_size        = hdr_size;
    shm_hdr->block_count        = bcount;
    shm_hdr->field_count        = fields;
    shm_hdr->meta_va            = (uintptr_t)meta_base;
    shm_hdr->data_va            = (uintptr_t)data_base;
    shm_hdr->data_size          = data_size;
    shm_hdr->attached_processes = 1;

    mi_atomic_store_release(&shm_hdr->magic, MI_CXL_SHM_MAGIC);  /* PUBLISH */

  } else {
    /* ------------------------------------------------------------------
       SUBSEQUENT PROCESS: wait, validate, attach.
       ------------------------------------------------------------------ */
    uint64_t m;
    do { m = mi_atomic_load_acquire(&shm_hdr->magic); }
    while (m == MI_CXL_SHM_MAGIC_INIT);

    if (mi_unlikely(m != MI_CXL_SHM_MAGIC)) {
      _mi_error_message(EINVAL,
        "cxl-arena: magic 0x%llx unexpected — uninitialised or corrupt\n",
        (unsigned long long)m);
      return false;
    }
    if (mi_unlikely((uintptr_t)meta_base != shm_hdr->meta_va)) {
      _mi_error_message(EINVAL,
        "cxl-arena: meta_base VA mismatch header=0x%zx caller=0x%zx\n",
        (size_t)shm_hdr->meta_va, (size_t)(uintptr_t)meta_base);
      return false;
    }
    if (mi_unlikely((uintptr_t)data_base != shm_hdr->data_va)) {
      _mi_error_message(EINVAL,
        "cxl-arena: data_base VA mismatch header=0x%zx caller=0x%zx\n",
        (size_t)shm_hdr->data_va, (size_t)(uintptr_t)data_base);
      return false;
    }
    if (mi_unlikely(data_size != shm_hdr->data_size)) {
      _mi_error_message(EINVAL,
        "cxl-arena: data_size mismatch header=%zu caller=%zu\n",
        shm_hdr->data_size, data_size);
      return false;
    }

    /* Reference count. */
    const size_t prev_count = mi_atomic_add_acq_rel(
        (_Atomic(size_t)*)&shm_hdr->attached_processes, (size_t)1);

    if (prev_count == 0) {
      _mi_memzero(&arena->blocks_inuse[0],
                  3 * fields * sizeof(mi_bitmap_field_t));
      arena->blocks_dirty     = &arena->blocks_inuse[fields];
      arena->blocks_abandoned = &arena->blocks_inuse[2 * fields];
      arena->blocks_committed = NULL;
      arena->blocks_purge     = NULL;
      ptrdiff_t post = (ptrdiff_t)(fields * MI_BITMAP_FIELD_BITS) - (ptrdiff_t)bcount;
      mi_assert_internal(post >= 0);
      if (post > 0) {
        mi_bitmap_index_t postidx = mi_bitmap_index_create(fields - 1,
                                        MI_BITMAP_FIELD_BITS - (size_t)post);
        _mi_bitmap_claim(arena->blocks_inuse, fields, (size_t)post, postidx, NULL);
      }
    } else {
      /* Clear stale abandoned bits to prevent reclaim of cross-process
       * segments whose page->heap points into another process's DRAM.    */
      for (size_t i = 0; i < arena->field_count; i++) {
        mi_bitmap_field_t aband =
            mi_atomic_load_acquire(&arena->blocks_abandoned[i]);
        while (aband != 0) {
          if (mi_atomic_cas_strong_acq_rel(&arena->blocks_abandoned[i],
                                            &aband, (mi_bitmap_field_t)0)) {
            mi_bitmap_field_t cur =
                mi_atomic_load_relaxed(&arena->blocks_inuse[i]);
            while (!mi_atomic_cas_strong_acq_rel(&arena->blocks_inuse[i],
                                                  &cur, cur & ~aband))
              { /* cur refreshed */ }
            break;
          }
        }
      }
    }

    /* Install the arena at the slot the first process chose.
     * arena->id was written before magic was published so it is
     * visible after our acquire load above.                              */
    const size_t slot = (size_t)mi_arena_id_index(arena->id);
    if (mi_unlikely(slot >= MI_MAX_ARENAS)) {
      _mi_error_message(EINVAL, "cxl-arena: invalid slot %zu in shared header\n",
                        slot);
      mi_atomic_add_acq_rel((_Atomic(size_t)*)&shm_hdr->attached_processes,
                          (size_t)-1);
      return false;
    }
    /* Advance mi_arena_count to cover this slot if needed. */
    size_t cur = mi_atomic_load_relaxed(&mi_arena_count);
    while (cur <= slot) {
      if (mi_atomic_cas_strong_acq_rel(&mi_arena_count, &cur, slot + 1)) break;
    }
    mi_atomic_store_ptr_release(mi_arena_t, &mi_arenas[slot], arena);
    if (arena_id != NULL) { *arena_id = arena->id; }
    /* Mirror what mi_arena_add does for the first process: count this arena
     * in the local process stats so mi_stats_print shows it.               */
    _mi_stat_counter_increase(&_mi_stats_main.arena_count, 1);
  }

  return true;
}

/* ============================================================
   mi_cxl_detach
   ============================================================
   Decrement the attached_processes reference count.
   Call before exit/munmap for every arena this process registered:

     mi_heap_collect(heap, true);
     mi_heap_delete(heap);
     mi_cxl_detach(meta_base);
     munmap(...);  // only after ALL mi_cxl_detach calls
*/
void mi_cxl_detach(void* meta_base) {
  if (mi_unlikely(meta_base == NULL)) return;
  mi_cxl_shm_header_t* const h = (mi_cxl_shm_header_t*)meta_base;
  if (mi_atomic_load_acquire(&h->magic) != MI_CXL_SHM_MAGIC) return;
  size_t cur = mi_atomic_load_relaxed((_Atomic(size_t)*)&h->attached_processes);
  while (cur > 0) {
    if (mi_atomic_cas_strong_acq_rel((_Atomic(size_t)*)&h->attached_processes,
                                      &cur, cur - 1))
      break;
  }
}

/* ============================================================
   mi_cxl_arena_stats / mi_cxl_arena_stats_print
   ============================================================ */

/* Count set bits across an entire bitmap array using popcount.
 * Reads with acquire to see the latest values from all processes.          */
static size_t mi_cxl_bitmap_popcount(const mi_bitmap_field_t* bm,
                                      size_t field_count)
{
  size_t total = 0;
  for (size_t i = 0; i < field_count; i++) {
    mi_bitmap_field_t w = mi_atomic_load_acquire(
        (const mi_bitmap_field_t*)&bm[i]);
    total += (size_t)__builtin_popcountll((unsigned long long)w);
  }
  return total;
}

void mi_cxl_arena_stats(const void* meta_base, mi_cxl_arena_stats_t* out) {
  if (mi_unlikely(meta_base == NULL || out == NULL)) return;

  const mi_cxl_shm_header_t* const h =
      (const mi_cxl_shm_header_t*)meta_base;

  /* Validate magic before touching arena. */
  if (mi_unlikely(mi_atomic_load_acquire(
          (const _Atomic(uint64_t)*)&h->magic) != MI_CXL_SHM_MAGIC)) {
    _mi_memzero(out, sizeof(*out));
    return;
  }

  const mi_arena_t* const arena =
      (const mi_arena_t*)((const uint8_t*)meta_base + sizeof(mi_cxl_shm_header_t));
  const size_t fields      = arena->field_count;
  const size_t block_count = arena->block_count;

  /* Read all bitmaps.  blocks_inuse spans [0..fields-1] starting at
   * &arena->blocks_inuse[0] — the same layout set by mi_cxl_arena_setup_bitmaps. */
  const size_t inuse     = mi_cxl_bitmap_popcount(&arena->blocks_inuse[0], fields);
  const size_t abandoned = (arena->blocks_abandoned != NULL)
      ? mi_cxl_bitmap_popcount(arena->blocks_abandoned, fields) : 0;
  const size_t dirty     = (arena->blocks_dirty != NULL)
      ? mi_cxl_bitmap_popcount(arena->blocks_dirty, fields) : 0;

  /* blocks_inuse includes phantom guard bits for the partial last word.
   * Subtract them to get the real inuse count.                             */
  const size_t phantom = fields * MI_BITMAP_FIELD_BITS - block_count;
  const size_t real_inuse = (inuse >= phantom) ? (inuse - phantom) : 0;
  const size_t free_blocks = (real_inuse <= block_count)
      ? (block_count - real_inuse) : 0;

  out->block_size         = MI_ARENA_BLOCK_SIZE;
  out->block_count        = block_count;
  out->blocks_inuse       = real_inuse;
  out->blocks_abandoned   = abandoned;
  out->blocks_dirty       = dirty;
  out->blocks_free        = free_blocks;
  out->attached_processes = mi_atomic_load_acquire(
      (const _Atomic(size_t)*)&h->attached_processes);
  out->bytes_inuse        = real_inuse  * MI_ARENA_BLOCK_SIZE;
  out->bytes_free         = free_blocks * MI_ARENA_BLOCK_SIZE;
}

void mi_cxl_arena_stats_print(const mi_cxl_arena_stats_t* s, const char* label) {
  if (s == NULL) return;
  if (label == NULL) label = "cxl";
  _mi_verbose_message(
    "%-8s arena: block_size=%zuKiB  total=%zu  inuse=%zu  free=%zu  "
    "abandoned=%zu  dirty=%zu  attached=%zu\n"
    "%-8s        bytes_inuse=%zuMiB  bytes_free=%zuMiB\n",
    label,
    s->block_size / 1024,
    s->block_count,
    s->blocks_inuse,
    s->blocks_free,
    s->blocks_abandoned,
    s->blocks_dirty,
    s->attached_processes,
    label,
    s->bytes_inuse  / (1024 * 1024),
    s->bytes_free   / (1024 * 1024));
}

/* ============================================================
   Public entry points
   ============================================================ */

bool mi_manage_os_memory_shared(void* shm_base, size_t shm_size,
                                 mi_arena_id_t* arena_id)
{
  if (mi_unlikely(shm_base == NULL || shm_size == 0 || arena_id == NULL)) {
    _mi_error_message(EINVAL, "cxl-arena: invalid arguments\n");
    return false;
  }
  size_t bcount, fields, hdr_size;
  if (!mi_cxl_compute_layout_colocated(shm_size, &bcount, &fields, &hdr_size)) {
    _mi_error_message(EINVAL,
      "cxl-arena: shm_size %zu too small (need at least 2 blocks + header)\n",
      shm_size);
    return false;
  }
  void* const  data_base = (uint8_t*)shm_base + hdr_size;
  const size_t data_size = bcount * MI_ARENA_BLOCK_SIZE;
  return mi_cxl_init_shared(shm_base, shm_size, data_base, data_size,
                              bcount, fields, hdr_size, arena_id);
}

bool mi_manage_os_memory_shared_disjoint(void* meta_base, size_t meta_size,
                                          void* data_base, size_t data_size,
                                          mi_arena_id_t* arena_id)
{
  if (mi_unlikely(meta_base == NULL || meta_size == 0 ||
                  data_base == NULL || data_size == 0 || arena_id == NULL)) {
    _mi_error_message(EINVAL, "cxl-arena: invalid arguments\n");
    return false;
  }
  size_t bcount, fields, hdr_size;
  if (!mi_cxl_compute_layout_disjoint(meta_size, data_size,
                                       (uintptr_t)data_base,
                                       &bcount, &fields, &hdr_size)) {
    _mi_error_message(EINVAL,
      "cxl-arena: meta_size %zu too small for data_size %zu "
      "(need %zu bytes — call mi_cxl_meta_region_size)\n",
      meta_size, data_size, mi_cxl_meta_region_size(data_size));
    return false;
  }
  return mi_cxl_init_shared(meta_base, meta_size, data_base, data_size,
                              bcount, fields, hdr_size, arena_id);
}
