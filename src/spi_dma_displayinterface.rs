//! DMA SPI interface for display drivers

use core::cell::RefCell;
use core::ptr::addr_of_mut;

use byte_slice_cast::AsByteSlice;
use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use esp_hal::dma::{DmaChannel, DmaDescriptor, DmaTxBuf, SpiPeripheral};
use esp_hal::gpio::DummyPin;
//use esp_hal::gpio::NoPin;
use esp_hal::spi::master::InstanceDma;
use esp_hal::spi::master::SpiDmaTransfer;
use esp_hal::spi::FullDuplexMode;
use esp_hal::Blocking;

const DMA_BUFFER_SIZE: usize = 4096;
type SpiDma<'d, T, C> =
    esp_hal::spi::master::SpiDma<'d, T, C, esp_hal::spi::FullDuplexMode, esp_hal::Blocking>;

/// SPI display interface.
///
/// This combines the SPI peripheral and a data/command as well as a chip-select pin
pub struct SPIInterface<'d, DC, CS, T, C>
where
    DC: embedded_hal::digital::v2::OutputPin,
    CS: embedded_hal::digital::v2::OutputPin,
    T: InstanceDma,
    C: DmaChannel,
    C::P: SpiPeripheral,
{
    avg_data_len_hint: usize,
    spi: RefCell<Option<SpiDma<'d, T, C>>>,
    transfer: RefCell<Option<SpiDmaTransfer<'d, T, C, FullDuplexMode, Blocking, DmaTxBuf>>>,
    dc: DC,
    cs: Option<CS>,
}

#[allow(unused)]
impl<'d, DC, CS, T, C> SPIInterface<'d, DC, CS, T, C>
where
    DC: embedded_hal::digital::v2::OutputPin,
    CS: embedded_hal::digital::v2::OutputPin,
    T: InstanceDma,
    C: DmaChannel,
    C::P: SpiPeripheral,
{
    pub fn new(avg_data_len_hint: usize, spi: SpiDma<'d, T, C>, dc: DC, cs: CS) -> Self {
        Self {
            avg_data_len_hint,
            spi: RefCell::new(Some(spi)),
            transfer: RefCell::new(None),
            dc,
            cs: Some(cs),
        }
    }

    fn send_u8(&mut self, words: DataFormat<'_>) -> Result<(), DisplayError>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
    {
        if let Some(transfer) = self.transfer.get_mut().take() {
            let (reclaimed_spi, buffer) = transfer.wait();
            self.spi.replace(Some(reclaimed_spi));
        }

        match words {
            DataFormat::U8(slice) => {
                use byte_slice_cast::*;

                let mut send_buffer = dma_buffer1();
                send_buffer.as_mut_slice()[..slice.len()].copy_from_slice(slice.as_byte_slice());
                send_buffer.set_length(slice.len());

                self.single_transfer(send_buffer);
            }
            DataFormat::U16(slice) => {
                use byte_slice_cast::*;

                let mut send_buffer = dma_buffer1();
                send_buffer.as_mut_slice()[..slice.len() * 2]
                    .copy_from_slice(slice.as_byte_slice());
                send_buffer.set_length(slice.len() * 2);

                self.single_transfer(send_buffer);
            }
            DataFormat::U16LE(slice) => {
                use byte_slice_cast::*;
                for v in slice.as_mut() {
                    *v = v.to_le();
                }

                let mut send_buffer = dma_buffer1();
                send_buffer.as_mut_slice()[..slice.len() * 2]
                    .copy_from_slice(slice.as_byte_slice());
                send_buffer.set_length(slice.len() * 2);

                self.single_transfer(send_buffer);
            }
            DataFormat::U16BE(slice) => {
                use byte_slice_cast::*;
                for v in slice.as_mut() {
                    *v = v.to_be();
                }

                let mut send_buffer = dma_buffer1();
                send_buffer.as_mut_slice()[..slice.len() * 2]
                    .copy_from_slice(slice.as_byte_slice());
                send_buffer.set_length(slice.len() * 2);

                self.single_transfer(send_buffer);
            }
            DataFormat::U8Iter(iter) => {
                self.iter_transfer(iter, |v| v.to_be_bytes());
            }
            DataFormat::U16LEIter(iter) => {
                self.iter_transfer(iter, |v| v.to_le_bytes());
            }
            DataFormat::U16BEIter(iter) => {
                self.iter_transfer(iter, |v| v.to_be_bytes());
            }
            _ => {
                return Err(DisplayError::DataFormatNotImplemented);
            }
        }
        Ok(())
    }

    fn single_transfer(&mut self, send_buffer: DmaTxBuf) {
        let transfer = self
            .spi
            .get_mut()
            .take()
            .unwrap()
            .dma_write(send_buffer)
            .unwrap();
        let (reclaimed_spi, _) = transfer.wait();
        self.spi.replace(Some(reclaimed_spi));
    }

    fn iter_transfer<WORD>(
        &mut self,
        iter: &mut dyn Iterator<Item = WORD>,
        convert: fn(WORD) -> <WORD as num_traits::ToBytes>::Bytes,
    ) where
        WORD: num_traits::int::PrimInt + num_traits::ToBytes,
    {
        let mut desired_chunk_sized =
            self.avg_data_len_hint - ((self.avg_data_len_hint / DMA_BUFFER_SIZE) * DMA_BUFFER_SIZE);
        let mut spi = Some(self.spi.get_mut().take().unwrap());
        let mut current_buffer = 0;
        let mut transfer: Option<SpiDmaTransfer<'d, T, C, FullDuplexMode, Blocking, DmaTxBuf>> =
            None;
        loop {
            let mut buffer = if current_buffer == 0 {
                dma_buffer1()
            } else {
                dma_buffer2()
            };
            let mut idx = 0;
            loop {
                let b = iter.next();

                match b {
                    Some(b) => {
                        let b = convert(b);
                        let b = b.as_byte_slice();
                        buffer.as_mut_slice()[idx + 0] = b[0];
                        if b.len() == 2 {
                            buffer.as_mut_slice()[idx + 1] = b[1];
                        }
                        idx += b.len();
                    }
                    None => break,
                }

                if idx >= usize::min(desired_chunk_sized, DMA_BUFFER_SIZE) {
                    break;
                }
            }
            desired_chunk_sized = DMA_BUFFER_SIZE;

            if let Some(transfer) = transfer {
                if idx > 0 {
                    let (reclaimed_spi, relaimed_buffer) = transfer.wait();
                    spi = Some(reclaimed_spi);
                } else {
                    // last transaction inflight
                    self.transfer.replace(Some(transfer));
                }
            }

            if idx > 0 {
                buffer.set_length(idx);

                let spi_instance = Option::take(&mut spi).unwrap();
                transfer = Some(spi_instance.dma_write(buffer).unwrap());

                current_buffer = (current_buffer + 1) % 2;
            } else {
                break;
            }
        }
    }
}

// pub fn new_no_cs<'d, DC, T, C>(
//     avg_data_len_hint: usize,
//     spi: SpiDma<'d, T, C>,
//     dc: DC,
// ) -> SPIInterface<'d, DC, NoPin, T, C>
// where
//     DC: embedded_hal::digital::v2::OutputPin,
//     T: InstanceDma,
//     C: DmaChannel,
//     C::P: SpiPeripheral,
// {
//     SPIInterface {
//         avg_data_len_hint,
//         spi: RefCell::new(Some(spi)),
//         transfer: RefCell::new(None),
//         dc,
//         cs: Some(NoPin),
//     }
// }

pub fn new_no_cs<'d, DC, T, C>(
    avg_data_len_hint: usize,
    spi: SpiDma<'d, T, C>,
    dc: DC,
) -> SPIInterface<'d, DC, DummyPin, T, C>
where
    DC: embedded_hal::digital::v2::OutputPin,
    T: InstanceDma,
    C: DmaChannel,
    C::P: SpiPeripheral,
{
    SPIInterface {
        avg_data_len_hint,
        spi: RefCell::new(Some(spi)),
        transfer: RefCell::new(None),
        dc,
        cs: Some(DummyPin::new()),
    }
}


impl<'d, DC, CS, T, C> WriteOnlyDataCommand for SPIInterface<'d, DC, CS, T, C>
where
    DC: embedded_hal::digital::v2::OutputPin,
    CS: embedded_hal::digital::v2::OutputPin,
    T: InstanceDma,
    C: DmaChannel,
    C::P: SpiPeripheral,
{
    fn send_commands(&mut self, cmds: DataFormat<'_>) -> Result<(), DisplayError> {
        // Assert chip select pin
        if let Some(cs) = self.cs.as_mut() {
            cs.set_low().ok();
        }

        // 1 = data, 0 = command
        self.dc.set_low().ok();

        // Send words over SPI
        let res = self.send_u8(cmds);

        // Deassert chip select pin
        if let Some(cs) = self.cs.as_mut() {
            cs.set_high().ok();
        }
        res
    }

    fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
        // Assert chip select pin
        if let Some(cs) = self.cs.as_mut() {
            cs.set_low().ok();
        }

        // 1 = data, 0 = command
        self.dc.set_high().ok();

        // Send words over SPI
        let res = self.send_u8(buf);

        // Deassert chip select pin
        if let Some(cs) = self.cs.as_mut() {
            cs.set_high().ok();
        }

        res
    }
}

fn dma_buffer1() -> DmaTxBuf {
    static mut BUFFER: [u8; DMA_BUFFER_SIZE] = [0u8; DMA_BUFFER_SIZE];
    let tx_buffer = unsafe { &mut *addr_of_mut!(BUFFER) };
    static mut TX_DESCRIPTORS: [DmaDescriptor; 8 * 3] = [DmaDescriptor::EMPTY; 8 * 3];
    let tx_descriptors = unsafe { &mut *addr_of_mut!(TX_DESCRIPTORS) };

    DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap()
}

fn dma_buffer2() -> DmaTxBuf {
    static mut BUFFER: [u8; DMA_BUFFER_SIZE] = [0u8; DMA_BUFFER_SIZE];
    let tx_buffer = unsafe { &mut *addr_of_mut!(BUFFER) };
    static mut TX_DESCRIPTORS: [DmaDescriptor; 8 * 3] = [DmaDescriptor::EMPTY; 8 * 3];
    let tx_descriptors = unsafe { &mut *addr_of_mut!(TX_DESCRIPTORS) };

    DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap()
}
