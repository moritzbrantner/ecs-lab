use std::collections::BTreeMap;

const PAGE_BITS: u32 = 8;
const PAGE_SIZE: usize = 256;
const PAGE_SIZE_U64: u64 = 256;
const PAGE_MASK: u32 = 255;

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

    #[must_use]
    pub fn get(&self, key: u32) -> Option<usize> {
        let (page_id, slot) = split_key(key);
        self.pages.get(&page_id)?.slots[slot]
    }

    /// Associates the key with the value, returning the previous value when one existed.
    pub fn set(&mut self, key: u32, value: usize) -> Option<usize> {
        let (page_id, slot) = split_key(key);
        let page = self.pages.entry(page_id).or_insert_with(Page::new);
        let previous = page.slots[slot].replace(value);
        if previous.is_none() {
            page.occupied += 1;
        }
        previous
    }

    /// Removes the key, returning its previous value.
    pub fn remove(&mut self, key: u32) -> Option<usize> {
        let (page_id, slot) = split_key(key);
        let (value, empty) = {
            let page = self.pages.get_mut(&page_id)?;
            let value = page.slots[slot].take()?;
            page.occupied -= 1;
            (value, page.occupied == 0)
        };
        if empty {
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
        assert_eq!(index.set(2, 7), None);
        assert_eq!(index.set(u32::MAX, 11), None);

        assert_eq!(index.get(2), Some(7));
        assert_eq!(index.get(u32::MAX), Some(11));
        assert_eq!(index.get(3), None);
        assert_eq!(index.allocated_slots(), 2 * PAGE_SIZE_U64);
    }

    #[test]
    fn updating_existing_key_does_not_change_footprint() {
        let mut index = PagedSparseIndex::new();
        assert_eq!(index.set(42, 1), None);
        assert_eq!(index.set(42, 9), Some(1));

        assert_eq!(index.get(42), Some(9));
        assert_eq!(index.allocated_slots(), PAGE_SIZE_U64);
    }

    #[test]
    fn empty_page_is_released_after_last_removal() {
        let mut index = PagedSparseIndex::new();
        index.set(1, 3);
        index.set(2, 4);

        assert_eq!(index.remove(1), Some(3));
        assert_eq!(index.allocated_slots(), PAGE_SIZE_U64);
        assert_eq!(index.remove(2), Some(4));
        assert_eq!(index.allocated_slots(), 0);
    }
}
