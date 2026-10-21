use std::collections::BTreeMap;

const PAGE_BITS: u32 = 8;
const PAGE_SIZE: usize = 256;
const PAGE_SIZE_U64: u64 = 256;
const PAGE_MASK: u32 = 255;

/// Work a paged sparse index performs below the caller's one-slot-per-access view.
///
/// A page-map descent is one ordered page-map lookup, counted as one regardless of tree depth.
/// Slot accesses are the in-page slot reads and writes. Page allocation and release are the
/// fixed-size page boxes created by a first key in a range and dropped after its last key leaves.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PageWork {
    pub page_descents: u64,
    pub slot_accesses: u64,
    pub pages_allocated: u64,
    pub pages_released: u64,
}

impl PageWork {
    pub fn accumulate(&mut self, other: Self) {
        self.page_descents = self.page_descents.saturating_add(other.page_descents);
        self.slot_accesses = self.slot_accesses.saturating_add(other.slot_accesses);
        self.pages_allocated = self.pages_allocated.saturating_add(other.pages_allocated);
        self.pages_released = self.pages_released.saturating_add(other.pages_released);
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Page {
    slots: Box<[Option<usize>; PAGE_SIZE]>,
    occupied: usize,
}

impl Page {
    fn new() -> Self {
        Self {
            slots: Box::new([None; PAGE_SIZE]),
            occupied: 0,
        }
    }
}

/// Sparse index that allocates fixed-size pages only for key ranges that are actually used.
///
/// Dense ECS storage can therefore keep direct slot addressing within each page without allocating
/// every slot below the largest entity id. Page lookup is deterministic and ordered; unused pages
/// are released after their final entry is removed.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct PagedSparseIndex {
    pages: BTreeMap<u32, Page>,
}

impl PagedSparseIndex {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            pages: BTreeMap::new(),
        }
    }

    /// Uncounted lookup for hot query paths whose callers count lookups themselves.
    #[must_use]
    pub fn get(&self, key: u32) -> Option<usize> {
        let (page_id, slot) = split_key(key);
        self.pages.get(&page_id)?.slots[slot]
    }

    /// Looks the key up and records the page-map descent and, when the page exists, the slot read.
    pub fn get_counted(&self, key: u32, work: &mut PageWork) -> Option<usize> {
        let (page_id, slot) = split_key(key);
        work.page_descents += 1;
        let page = self.pages.get(&page_id)?;
        work.slot_accesses += 1;
        page.slots[slot]
    }

    /// Associates the key with the value, returning the previous value when one existed.
    pub fn set(&mut self, key: u32, value: usize, work: &mut PageWork) -> Option<usize> {
        let (page_id, slot) = split_key(key);
        work.page_descents += 1;
        work.slot_accesses += 1;
        let page = self.pages.entry(page_id).or_insert_with(|| {
            work.pages_allocated += 1;
            Page::new()
        });
        let previous = page.slots[slot].replace(value);
        if previous.is_none() {
            page.occupied += 1;
        }
        previous
    }

    /// Removes the key, returning its previous value. Releasing an emptied page costs a second
    /// page-map descent.
    pub fn remove(&mut self, key: u32, work: &mut PageWork) -> Option<usize> {
        let (page_id, slot) = split_key(key);
        work.page_descents += 1;
        let (value, empty) = {
            let page = self.pages.get_mut(&page_id)?;
            work.slot_accesses += 1;
            let value = page.slots[slot].take()?;
            page.occupied -= 1;
            (value, page.occupied == 0)
        };
        if empty {
            work.page_descents += 1;
            work.pages_released += 1;
            self.pages.remove(&page_id);
        }
        Some(value)
    }

    /// Number of logical sparse slots retained by allocated pages.
    #[must_use]
    pub fn allocated_slots(&self) -> u64 {
        u64::try_from(self.pages.len())
            .unwrap_or(u64::MAX)
            .saturating_mul(PAGE_SIZE_U64)
    }
}

fn split_key(key: u32) -> (u32, usize) {
    (
        key >> PAGE_BITS,
        usize::try_from(key & PAGE_MASK).unwrap_or_default(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distant_keys_allocate_only_their_pages() {
        let mut index = PagedSparseIndex::new();
        let mut work = PageWork::default();
        assert_eq!(index.set(2, 7, &mut work), None);
        assert_eq!(index.set(u32::MAX, 11, &mut work), None);

        assert_eq!(index.get(2), Some(7));
        assert_eq!(index.get(u32::MAX), Some(11));
        assert_eq!(index.get(3), None);
        assert_eq!(index.allocated_slots(), 2 * PAGE_SIZE_U64);
        assert_eq!(work.pages_allocated, 2);
    }

    #[test]
    fn updating_existing_key_does_not_change_footprint() {
        let mut index = PagedSparseIndex::new();
        let mut work = PageWork::default();
        assert_eq!(index.set(42, 1, &mut work), None);
        assert_eq!(index.set(42, 9, &mut work), Some(1));

        assert_eq!(index.get(42), Some(9));
        assert_eq!(index.allocated_slots(), PAGE_SIZE_U64);
        assert_eq!(work.pages_allocated, 1);
    }

    #[test]
    fn empty_page_is_released_after_last_removal() {
        let mut index = PagedSparseIndex::new();
        let mut work = PageWork::default();
        index.set(1, 3, &mut work);
        index.set(2, 4, &mut work);

        let mut removal = PageWork::default();
        assert_eq!(index.remove(1, &mut removal), Some(3));
        assert_eq!(index.allocated_slots(), PAGE_SIZE_U64);
        assert_eq!(index.remove(2, &mut removal), Some(4));
        assert_eq!(index.allocated_slots(), 0);
        assert_eq!(
            removal,
            PageWork {
                page_descents: 3,
                slot_accesses: 2,
                pages_allocated: 0,
                pages_released: 1,
            }
        );
    }

    #[test]
    fn counted_lookup_of_missing_page_descends_without_slot_access() {
        let index = PagedSparseIndex::new();
        let mut work = PageWork::default();
        assert_eq!(index.get_counted(7, &mut work), None);
        assert_eq!(
            work,
            PageWork {
                page_descents: 1,
                ..PageWork::default()
            }
        );
    }
}
