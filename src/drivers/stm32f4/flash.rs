//! Internal Flash controller for the STM32F4 family
use crate::{
    hal::flash::ReadWrite,
    stm32pac::FLASH,
    utilities::{
        memory::{self, IterableByOverlaps},
    },
};
use core::ops::{Add, Sub};
use nb::block;

pub struct McuFlash {
    flash: FLASH,
}

#[derive(Copy, Clone, Debug)]
pub enum Error {
    MemoryNotReachable,
    MisalignedAccess,
}

#[derive(Default, Copy, Clone, Debug, PartialOrd, PartialEq, Ord, Eq)]
pub struct Address(pub u32);

impl Add<usize> for Address {
    type Output = Self;
    fn add(self, rhs: usize) -> Address { Address(self.0 + rhs as u32) }
}

impl Sub<usize> for Address {
    type Output = Self;
    fn sub(self, rhs: usize) -> Address { Address(self.0.saturating_sub(rhs as u32)) }
}

impl Sub<Address> for Address {
    type Output = usize;
    fn sub(self, rhs: Address) -> usize { self.0.saturating_sub(rhs.0) as usize }
}

impl Into<usize> for Address {
    fn into(self) -> usize { self.0 as usize }
}

#[derive(Copy, Clone, Debug)]
struct Range(Address, Address);

/// Different address blocks as defined in [Table 5](../../../../../../../documentation/hardware/stm32f412_reference.pdf#page=58)
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Block {
    /// Main memory, but reserved for immutable data (e.g. a bootloader image)
    Reserved,
    /// Main memory, where the application is written
    Main,
    SystemMemory,
    OneTimeProgrammable,
    OptionBytes,
}

/// A memory map sector, with an associated block and an address range
#[derive(Copy, Clone, Debug, PartialEq)]
#[non_exhaustive]
struct Sector {
    block: Block,
    location: Address,
    size: usize,
}

#[non_exhaustive]
pub struct MemoryMap {
    sectors: [Sector; SECTOR_NUMBER],
}

const UNLOCK_KEYS: [u32; 2] = [0x45670123, 0xCDEF89AB];

#[cfg(feature = "stm32f412")]
const SECTOR_NUMBER: usize = 15;

#[cfg(feature = "stm32f446")]
const SECTOR_NUMBER: usize = 11;

#[cfg(feature = "stm32f412")]
const MEMORY_MAP: MemoryMap = MemoryMap {
    sectors: [
        Sector::new(Block::Reserved, Address(0x0800_0000), KB!(16)),
        Sector::new(Block::Reserved, Address(0x0800_4000), KB!(16)),
        Sector::new(Block::Reserved, Address(0x0800_8000), KB!(16)),
        Sector::new(Block::Reserved, Address(0x0800_C000), KB!(16)),
        Sector::new(Block::Main, Address(0x0801_0000), KB!(64)),
        Sector::new(Block::Main, Address(0x0802_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x0804_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x0806_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x0808_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x080A_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x080C_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x080E_0000), KB!(128)),
        Sector::new(Block::SystemMemory, Address(0x1FFF_0000), KB!(32)),
        Sector::new(Block::OneTimeProgrammable, Address(0x1FFF_7800), 528),
        Sector::new(Block::OptionBytes, Address(0x1FFF_C000), 16),
    ],
};

#[cfg(feature = "stm32f446")]
const MEMORY_MAP: MemoryMap = MemoryMap {
    sectors: [
        Sector::new(Block::Reserved, Address(0x0800_0000), KB!(16)),
        Sector::new(Block::Reserved, Address(0x0800_4000), KB!(16)),
        Sector::new(Block::Reserved, Address(0x0800_8000), KB!(16)),
        Sector::new(Block::Reserved, Address(0x0800_C000), KB!(16)),
        Sector::new(Block::Main, Address(0x0801_0000), KB!(64)),
        Sector::new(Block::Main, Address(0x0802_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x0804_0000), KB!(128)),
        Sector::new(Block::Main, Address(0x0806_0000), KB!(128)),
        Sector::new(Block::SystemMemory, Address(0x1FFF_0000), KB!(30)),
        Sector::new(Block::OneTimeProgrammable, Address(0x1FFF_7800), 528),
        Sector::new(Block::OptionBytes, Address(0x1FFF_C000), 16),
    ],
};

impl MemoryMap {
    // Verifies that the memory map is consecutive and well formed
    fn is_sound(&self) -> bool {
        let main_sectors = self.sectors.iter().filter(|s| s.is_in_main_memory_area());
        let mut consecutive_pairs = main_sectors.clone().zip(main_sectors.skip(1));
        let consecutive = consecutive_pairs.all(|(a, b)| a.end() == b.start());
        let ranges_valid =
            self.sectors.iter().map(|s| Range(s.start(), s.end())).all(Range::is_valid);
        consecutive && ranges_valid
    }

    fn sectors() -> impl Iterator<Item = Sector> { MEMORY_MAP.sectors.iter().cloned() }
    pub const fn writable_start() -> Address {
        let mut i = 0;
        loop {
            if MEMORY_MAP.sectors[i].is_writable() {
                break MEMORY_MAP.sectors[i].start();
            }
            i += 1;
        }
    }
    pub const fn writable_end() -> Address {
        let mut i = 0;
        loop {
            // Reach the writable area.
            if MEMORY_MAP.sectors[i].is_writable() {
                break;
            }
            i += 1;
        }

        loop {
            // Reach the end of the writable area
            if !MEMORY_MAP.sectors[i + 1].is_writable() {
                break MEMORY_MAP.sectors[i].end();
            }
            i += 1;
        }
    }
}

impl Range {
    /// Sectors spanned by this range of addresses
    fn span(self) -> &'static [Sector] {
        let first = MEMORY_MAP
            .sectors
            .iter()
            .enumerate()
            .find_map(|(i, sector)| self.overlaps(sector).then_some(i));
        let last = MEMORY_MAP
            .sectors
            .iter()
            .enumerate()
            .rev()
            .find_map(|(i, sector)| self.overlaps(sector).then_some(i));
        match (first, last) {
            (Some(first), Some(last)) if (last >= first) => &MEMORY_MAP.sectors[first..(last + 1)],
            _ => &MEMORY_MAP.sectors[0..1],
        }
    }

    const fn is_valid(self) -> bool {
        let Range(Address(start), Address(end)) = self;
        let after_map = start >= MEMORY_MAP.sectors[SECTOR_NUMBER - 1].end().0;
        let before_map = end < MEMORY_MAP.sectors[0].end().0;
        let monotonic = end >= start;
        monotonic && !before_map && !after_map
    }

    fn overlaps(self, sector: &Sector) -> bool {
        (self.0 <= sector.start()) && (self.1 > sector.end())
            || (self.0 < sector.end()) && (self.1 >= sector.end())
            || (self.0 >= sector.start() && self.0 < sector.end())
            || (self.1 < sector.end() && self.1 >= sector.start())
    }

    /// Verify that all sectors spanned by this range are writable
    fn is_writable(self) -> bool { self.span().iter().all(Sector::is_writable) }
}

impl memory::Region<Address> for Sector {
    fn contains(&self, address: Address) -> bool {
        (self.start() <= address) && (self.end() > address)
    }
}

impl Sector {
    const fn start(&self) -> Address { self.location }
    const fn end(&self) -> Address { Address(self.start().0 + self.size as u32) }
    const fn new(block: Block, location: Address, size: usize) -> Self {
        Sector { block, location, size }
    }
    fn number(&self) -> Option<u8> {
        MEMORY_MAP.sectors.iter().enumerate().find_map(|(index, sector)| {
            (sector.is_in_main_memory_area() && self == sector).then_some(index as u8)
        })
    }
    const fn is_writable(&self) -> bool { self.block as u8 == Block::Main as u8 }
    const fn is_in_main_memory_area(&self) -> bool {
        self.block as u8 == Block::Main as u8 || self.block as u8 == Block::Reserved as u8
    }
}

impl McuFlash {
    pub fn new(flash: FLASH) -> Result<Self, Error> {
        assert!(MEMORY_MAP.is_sound());
        Ok(Self { flash })
    }

    /// Parallelism for 3v3 voltage from [table 7](../../../../../../../../documentation/hardware/stm32f412_reference.pdf#page=63)
    /// (Word access parallelism)
    fn unlock(&mut self) -> nb::Result<(), Error> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }
        // NOTE(Safety): Unsafe block to use the 'bits' convenience function.
        // Applies to all blocks in this file unless specified otherwise
        self.flash.keyr.write(|w| unsafe { w.bits(UNLOCK_KEYS[0]) });
        self.flash.keyr.write(|w| unsafe { w.bits(UNLOCK_KEYS[1]) });
        self.flash.cr.modify(|_, w| unsafe { w.psize().bits(0b10) });
        Ok(())
    }

    fn lock(&mut self) -> nb::Result<(), Error> {
        if self.is_busy() {
            Err(nb::Error::WouldBlock)
        } else {
            self.flash.cr.modify(|_, w| w.lock().set_bit());
            Ok(())
        }
    }

    fn erase(&mut self, sector: &Sector) -> nb::Result<(), Error> {
        let number = sector.number().ok_or(nb::Error::Other(Error::MemoryNotReachable))?;
        self.unlock()?;
        self.flash
            .cr
            .modify(|_, w| unsafe { w.ser().set_bit().snb().bits(number).strt().set_bit() });
        block!(self.lock())?;
        Ok(())
    }

    fn is_busy(&self) -> bool { self.flash.sr.read().bsy().bit_is_set() }

    fn write_bytes(
        &mut self,
        bytes: &[u8],
        sector: &Sector,
        address: Address,
    ) -> nb::Result<(), Error> {
        if (address < sector.start()) || (address + bytes.len() > sector.end()) {
            return Err(nb::Error::Other(Error::MisalignedAccess));
        }

        let words = bytes.chunks(4).map(|bytes| {
            u32::from_le_bytes([
                bytes.get(0).cloned().unwrap_or(0),
                bytes.get(1).cloned().unwrap_or(0),
                bytes.get(2).cloned().unwrap_or(0),
                bytes.get(3).cloned().unwrap_or(0),
            ])
        });

        block!(self.unlock())?;
        self.flash.cr.modify(|_, w| w.pg().set_bit());
        let base_address = address.0 as *mut u32;
        for (index, word) in words.enumerate() {
            // NOTE(Safety): Writing to a memory-mapped flash
            // directly is naturally unsafe. We have to trust that
            // the memory map is correct, and that these dereferences
            // won't cause a hardfault or overlap with our firmware.
            unsafe {
                *(base_address.add(index)) = word;
            }
        }
        block!(self.lock())?;
        Ok(())
    }
}

impl ReadWrite for McuFlash {
    type Error = Error;
    type Address = Address;

    fn range(&self) -> (Address, Address) {
        (MemoryMap::writable_start(), MemoryMap::writable_end())
    }

    // NOTE: This only erases the sections of the MCU flash that are writable
    // from the application's perspective. Not the reserved sector, system bytes, etc.
    fn erase(&mut self) -> nb::Result<(), Self::Error> {
        for sector in MEMORY_MAP.sectors.iter().filter(|s| s.is_writable()) {
            self.erase(sector)?;
        }
        Ok(())
    }

    fn write(&mut self, address: Address, bytes: &[u8]) -> nb::Result<(), Self::Error> {
        ///////////////////////////////////////////////////////////////////////////////////////////
        // WORKAROUND: This function pushes a 128KiB buffer onto the stack!! This makes chips    //
        // with 128KiB of RAM die, which isn't good. A workaround is to enforce writes starting  //
        // at sector boundaries, meaning this scratch buffer doesn't need to exist. While this   //
        // fixes the issue, other changes need to be made to suit the change. Eg. enforcing at a //
        // type level, updating loadstone_front, etc.                                            //
        ///////////////////////////////////////////////////////////////////////////////////////////

        let address_is_sector_boundary = MEMORY_MAP.sectors.iter().any(|s| s.location == address);
        if !address_is_sector_boundary {
            return Err(nb::Error::Other(Error::MisalignedAccess));
        }

        let range = Range(address, Address(address.0 + bytes.len() as u32));
        if !range.is_writable() {
            return Err(nb::Error::Other(Error::MemoryNotReachable));
        }

        // Early yield if busy
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        for (block, sector, _address) in MemoryMap::sectors().overlaps(bytes, address) {
            block!(self.erase(&sector))?;
            block!(self.write_bytes(block, &sector, sector.location))?;
        }

        Ok(())
    }

    fn read(&mut self, address: Address, bytes: &mut [u8]) -> nb::Result<(), Self::Error> {
        let range = Range(address, Address(address.0 + bytes.len() as u32));
        if !range.is_writable() {
            Err(nb::Error::Other(Error::MemoryNotReachable))
        } else {
            let base = address.0 as *const u8;
            for (index, byte) in bytes.iter_mut().enumerate() {
                // NOTE(Safety) we are reading directly from raw memory locations,
                // which is inherently unsafe.
                *byte = unsafe { *(base.add(index)) };
            }
            Ok(())
        }
    }

    fn write_from_blocks<I: Iterator<Item = [u8; N]>, const N: usize>(
        &mut self,
        address: Self::Address,
        blocks: I,
    ) -> Result<(), Self::Error> {
        const TRANSFER_SIZE: usize = KB!(4);
        assert!(TRANSFER_SIZE % N == 0);
        let mut transfer_array = [0x00u8; TRANSFER_SIZE];
        let mut memory_index = 0usize;

        for block in blocks {
            let slice = &mut transfer_array
                [(memory_index % TRANSFER_SIZE)..((memory_index % TRANSFER_SIZE) + N)];
            slice.clone_from_slice(&block);
            memory_index += N;

            if memory_index % TRANSFER_SIZE == 0 {
                nb::block!(self.write(address + (memory_index - TRANSFER_SIZE), &transfer_array))?;
                transfer_array.iter_mut().for_each(|b| *b = 0x00u8);
            }
        }
        let remainder = &transfer_array[0..(memory_index % TRANSFER_SIZE)];
        nb::block!(self.write(address + (memory_index - remainder.len()), &remainder))?;
        Ok(())
    }

    fn label() -> &'static str { "stm32f4 flash (Internal)" }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn ranges_overlap_sectors_correctly() {
        let sector = Sector::new(Block::Reserved, Address(10), 10usize);
        assert!(Range(Address(10), Address(20)).overlaps(&sector));
        assert!(Range(Address(5), Address(15)).overlaps(&sector));
        assert!(Range(Address(15), Address(25)).overlaps(&sector));
        assert!(Range(Address(5), Address(25)).overlaps(&sector));
        assert!(Range(Address(12), Address(18)).overlaps(&sector));

        assert!(!Range(Address(0), Address(5)).overlaps(&sector));
        assert!(!Range(Address(20), Address(25)).overlaps(&sector));
    }

    #[test]
    fn ranges_span_the_correct_sectors() {
        let range = Range(Address(0x0801_1234), Address(0x0804_5678));
        let expected_sectors = &MEMORY_MAP.sectors[4..7];

        assert_eq!(expected_sectors, range.span());
    }

    #[test]
    fn map_shows_correct_writable_range() {
        let (start, end) = (MemoryMap::writable_start(), MemoryMap::writable_end());
        assert_eq!(start, MEMORY_MAP.sectors[4].start());
        assert_eq!(end, MEMORY_MAP.sectors[11].end());
    }

    #[test]
    fn ranges_are_correctly_marked_writable() {
        let (start, size) = (Address(0x0801_0008), 48usize);
        let range = Range(start, Address(start.0 + size as u32));
        assert!(range.is_writable());
    }
}
