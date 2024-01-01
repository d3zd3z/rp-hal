//! This is the IRQ controller.  It should have doc comments.

use core::{marker::PhantomData, ops::Deref};
use embedded_hal::blocking::i2c::{Read, Write, WriteIter, WriteIterRead, WriteRead};
use fugit::HertzU32;

#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::i2c as eh1;
use rtic::Mutex;
use rtic_sync::channel::{Receiver, Sender};

use super::{i2c_reserved_addr, Controller, Error, ValidPinScl, ValidPinSda, I2C};
use crate::{
    pac::{i2c0::RegisterBlock as Block, RESETS},
    resets::SubsystemReset,
};

/// Information returned from interrupts.
pub enum ControllerIrqInfo {
    /// An abort condition happened.
    Abort(u32),
    /// The stop condition happened.
    Stop,
}

/// The Receiver to wait for interrupts.
pub type ControllerIrqWait<'a, const N: usize> = Receiver<'a, ControllerIrqInfo, N>;

impl<T, Sda, Scl> I2C<T, (Sda, Scl), Controller>
where
    T: SubsystemReset + Deref<Target = Block>,
    Sda: ValidPinSda<T>,
    Scl: ValidPinScl<T>,
{
    /// Configures the I2C peripheral to work in controller mode
    pub fn new_controller(
        i2c: T,
        sda_pin: Sda,
        scl_pin: Scl,
        freq: HertzU32,
        resets: &mut RESETS,
        system_clock: HertzU32,
    ) -> Self {
        let freq = freq.to_Hz();
        assert!(freq <= 1_000_000);
        assert!(freq > 0);

        i2c.reset_bring_down(resets);
        i2c.reset_bring_up(resets);

        i2c.ic_enable.write(|w| w.enable().disabled());

        // select controller mode & speed
        i2c.ic_con.modify(|_, w| {
            w.speed().fast();
            w.master_mode().enabled();
            w.ic_slave_disable().slave_disabled();
            w.ic_restart_en().enabled();
            w.tx_empty_ctrl().enabled()
        });

        // Clear FIFO threshold
        i2c.ic_tx_tl.write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl.write(|w| unsafe { w.rx_tl().bits(0) });

        // Clear all interrupts.
        i2c.ic_clr_intr.read();
        i2c.ic_intr_mask.write(|w| unsafe { w.bits(0) });

        let freq_in = system_clock.to_Hz();

        // There are some subtleties to I2C timing which we are completely ignoring here
        // See: https://github.com/raspberrypi/pico-sdk/blob/bfcbefafc5d2a210551a4d9d80b4303d4ae0adf7/src/rp2_common/hardware_i2c/i2c.c#L69
        let period = (freq_in + freq / 2) / freq;
        let lcnt = period * 3 / 5; // spend 3/5 (60%) of the period low
        let hcnt = period - lcnt; // and 2/5 (40%) of the period high

        // Check for out-of-range divisors:
        assert!(hcnt <= 0xffff);
        assert!(lcnt <= 0xffff);
        assert!(hcnt >= 8);
        assert!(lcnt >= 8);

        // Per I2C-bus specification a device in standard or fast mode must
        // internally provide a hold time of at least 300ns for the SDA signal to
        // bridge the undefined region of the falling edge of SCL. A smaller hold
        // time of 120ns is used for fast mode plus.
        let sda_tx_hold_count = if freq < 1000000 {
            // sda_tx_hold_count = freq_in [cycles/s] * 300ns * (1s / 1e9ns)
            // Reduce 300/1e9 to 3/1e7 to avoid numbers that don't fit in uint.
            // Add 1 to avoid division truncation.
            ((freq_in * 3) / 10000000) + 1
        } else {
            // fast mode plus requires a clk_in > 32MHz
            assert!(freq_in >= 32_000_000);

            // sda_tx_hold_count = freq_in [cycles/s] * 120ns * (1s / 1e9ns)
            // Reduce 120/1e9 to 3/25e6 to avoid numbers that don't fit in uint.
            // Add 1 to avoid division truncation.
            ((freq_in * 3) / 25000000) + 1
        };
        assert!(sda_tx_hold_count <= lcnt - 2);

        unsafe {
            i2c.ic_fs_scl_hcnt
                .write(|w| w.ic_fs_scl_hcnt().bits(hcnt as u16));
            i2c.ic_fs_scl_lcnt
                .write(|w| w.ic_fs_scl_lcnt().bits(lcnt as u16));
            i2c.ic_fs_spklen.write(|w| {
                w.ic_fs_spklen()
                    .bits(if lcnt < 16 { 1 } else { (lcnt / 16) as u8 })
            });
            i2c.ic_sda_hold
                .modify(|_r, w| w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16));
        }

        // Enable I2C block
        i2c.ic_enable.write(|w| w.enable().enabled());

        Self {
            i2c,
            pins: (sda_pin, scl_pin),
            mode: PhantomData,
        }
    }
}
impl<T: Deref<Target = Block>, PINS> I2C<T, PINS, Controller> {
    fn validate(
        addr: u16,
        opt_tx_empty: Option<bool>,
        opt_rx_empty: Option<bool>,
    ) -> Result<(), Error> {
        // validate tx parameters if present
        if opt_tx_empty.unwrap_or(false) {
            return Err(Error::InvalidWriteBufferLength);
        }

        // validate rx parameters if present
        if opt_rx_empty.unwrap_or(false) {
            return Err(Error::InvalidReadBufferLength);
        }

        // validate address
        if addr >= 0x80 {
            Err(Error::AddressOutOfRange(addr))
        } else if i2c_reserved_addr(addr) {
            Err(Error::AddressReserved(addr))
        } else {
            Ok(())
        }
    }

    fn setup(&mut self, addr: u16) {
        self.i2c.ic_enable.write(|w| w.enable().disabled());
        self.i2c.ic_tar.write(|w| unsafe { w.ic_tar().bits(addr) });
        self.i2c.ic_enable.write(|w| w.enable().enabled());
    }

    fn read_and_clear_abort_reason(&mut self) -> Option<u32> {
        let abort_reason = self.i2c.ic_tx_abrt_source.read().bits();
        if abort_reason != 0 {
            // Note clearing the abort flag also clears the reason, and
            // this instance of flag is clear-on-read! Note also the
            // IC_CLR_TX_ABRT register always reads as 0.
            self.i2c.ic_clr_tx_abrt.read();
            Some(abort_reason)
        } else {
            None
        }
    }

    fn read_internal(
        &mut self,
        buffer: &mut [u8],
        force_restart: bool,
        do_stop: bool,
    ) -> Result<(), Error> {
        let lastindex = buffer.len() - 1;
        for (i, byte) in buffer.iter_mut().enumerate() {
            let first = i == 0;
            let last = i == lastindex;

            // wait until there is space in the FIFO to write the next byte
            while self.tx_fifo_full() {}

            self.i2c.ic_data_cmd.write(|w| {
                if force_restart && first {
                    w.restart().enable();
                } else {
                    w.restart().disable();
                }

                if do_stop && last {
                    w.stop().enable();
                } else {
                    w.stop().disable();
                }

                w.cmd().read()
            });

            while self.i2c.ic_rxflr.read().bits() == 0 {
                if let Some(abort_reason) = self.read_and_clear_abort_reason() {
                    return Err(Error::Abort(abort_reason));
                }
            }

            *byte = self.i2c.ic_data_cmd.read().dat().bits();
        }

        Ok(())
    }

    /// Issue the given bytes as a write to the specified target.  This will
    /// only block if the transmit fifo fills up, and does not wait upon completion.
    /// The `do_stop` flag indicates that the last written value should initiate
    /// a I2C bus stop.  This will stop early if an abort is detected, but does
    /// not handle that error in any way.
    fn write_internal(&mut self, bytes: &[u8], do_stop: bool) -> Result<(), Error> {
        let mut pos = 0;
        while pos < bytes.len() {
            pos += self.write_internal_partial(&bytes[pos..], do_stop)?;

            // If the transmit fifo is full, need to wait.
            if pos < bytes.len() {
                todo!();
            }
        }
        Ok(())
    }

    fn write_internal_partial(&mut self, bytes: &[u8], do_stop: bool) -> Result<usize, Error> {
        for (i, byte) in bytes.iter().enumerate() {
            let last = i == bytes.len() - 1;

            // If the transmit fifo is full, we need to wait until it has room.
            // TODO: This can wait for an appropriate interrupt for a threshold.
            // Which will require changing the meaning of the TX_EMPTY_CTRL flag.
            // TODO: Change this to always write, and detect the TX_OVER
            // interrupt. It doesn't seem like we can change the threshold.  I
            // wonder if this was intended to be used by DMA.
            if self.i2c.ic_status.read().tfnf().is_full() {
                return Ok(i);
            }

            self.i2c.ic_data_cmd.write(|w| {
                if do_stop && last {
                    w.stop().enable();
                } else {
                    w.stop().disable();
                }
                unsafe { w.dat().bits(*byte) }
            });

            // If, at any time, we see an abort, stop all of this.
            if self.i2c.ic_raw_intr_stat.read().tx_abrt().is_active() {
                break;
            }
        }
        Ok(bytes.len())
    }


    // /// Rtic async version of above.
    // async fn write_internal_async<M>(me: &M, bytes: &[u8], do_stop: bool) -> Result<(), Error>
    // {
    //     unimplemented!()
    // }

    /// Wait for the I2c bus to stop.  Can  return indicating an abort happened.
    /// Blocking version.
    fn wait_stop(&mut self) -> Result<(), Error> {
        let mut abort = None;
        loop {
            let reg = self.i2c.ic_raw_intr_stat.read();
            // If we get an abort, read it.  But we still wait for a stop.
            if reg.tx_abrt().is_active() {
                abort = Some(self.i2c.ic_tx_abrt_source.read().bits());
                self.i2c.ic_clr_tx_abrt.read();
            }
            if reg.stop_det().is_active() {
                self.i2c.ic_clr_stop_det.read();
                break;
            }
        };

        if let Some(abort) = abort {
            return Err(Error::Abort(abort));
        }

        Ok(())
    }

    async fn wait_stop_async<'a, const N: usize>(irq: &mut ControllerIrqWait<'a, N>) -> Result<(), Error> {
        let mut abort = None;

        loop {
            match irq.recv().await {
                Ok(ControllerIrqInfo::Abort(count)) => abort = Some(count),
                Ok(ControllerIrqInfo::Stop) => break,
                Err(_) => return Err(Error::IrqError),
            }
        }

        if let Some(abort) = abort {
            return Err(Error::Abort(abort));
        }

        Ok(())
    }

    /// The actual interrupt handler.  Any events received should be sent to the
    /// given channel.
    pub fn handle_irq<'a, const N: usize>(&mut self, chan: &mut Sender<'a, ControllerIrqInfo, N>) {
        let reg = self.i2c.ic_intr_stat.read();
        // defmt::info!("i2c irq: {:x}", reg.bits());
        if reg.r_tx_abrt().is_active() {
            let abort = self.i2c.ic_tx_abrt_source.read().bits();
            self.i2c.ic_clr_tx_abrt.read();
            let _ = chan.try_send(ControllerIrqInfo::Abort(abort));
        }
        if reg.r_stop_det().is_active() {
            self.i2c.ic_clr_stop_det.read();
            let _ = chan.try_send(ControllerIrqInfo::Stop);
        }
    }

    /// Writes bytes to slave with address `address`
    ///
    /// # I2C Events (contract)
    ///
    /// Same as the `write` method
    pub fn write_iter<B>(&mut self, address: u8, bytes: B) -> Result<(), Error>
    where
        B: IntoIterator<Item = u8>,
    {
        if true {
            return Err(Error::Todo);
        }

        let mut peekable = bytes.into_iter().peekable();
        let addr: u16 = address.into();
        Self::validate(addr, Some(peekable.peek().is_none()), None)?;
        self.setup(addr);

        while let Some(tx) = peekable.next() {
            self.write_internal(&[tx], peekable.peek().is_none())?
        }
        Ok(())
    }

    /// Writes bytes to slave with address `address` and then reads enough bytes to fill `buffer` *in a
    /// single transaction*
    ///
    /// # I2C Events (contract)
    ///
    /// Same as the `write_read` method
    pub fn write_iter_read<B>(
        &mut self,
        address: u8,
        bytes: B,
        buffer: &mut [u8],
    ) -> Result<(), Error>
    where
        B: IntoIterator<Item = u8>,
    {
        if true {
            return Err(Error::Todo);
        }

        let mut peekable = bytes.into_iter().peekable();
        let addr: u16 = address.into();
        Self::validate(addr, Some(peekable.peek().is_none()), None)?;
        self.setup(addr);

        for tx in peekable {
            self.write_internal(&[tx], false)?
        }
        self.read_internal(buffer, true, true)
    }

    /// Execute the provided operations on the I2C bus (iterator version).
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an acknowledge for the last byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0 to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    #[cfg(feature = "eh1_0_alpha")]
    pub fn transaction_iter<'a, O>(&mut self, address: u8, operations: O) -> Result<(), Error>
    where
        O: IntoIterator<Item = eh1::Operation<'a>>,
    {
        if true {
            return Err(Error::Todo);
        }
        let addr: u16 = address.into();
        self.setup(addr);
        let mut peekable = operations.into_iter().peekable();
        while let Some(operation) = peekable.next() {
            let last = peekable.peek().is_none();
            match operation {
                eh1::Operation::Read(buf) => self.read_internal(buf, false, last)?,
                eh1::Operation::Write(buf) => self.write_internal(buf, last)?,
            }
        }
        Ok(())
    }

    /// Async (rtic) write method.
    pub async fn write_async<'a, const N: usize, M: Mutex<T=Self>>(i2c: &mut M, irq: &mut ControllerIrqWait<'a, N>, addr: u8, tx: &[u8]) -> Result<(), Error> {
        let addr: u16 = addr.into();
        Self::validate(addr, Some(tx.is_empty()), None)?;
        i2c.lock(|i2c| i2c.setup(addr));

        i2c.lock(|i2c| {
            i2c.i2c.ic_intr_mask.write(|w| {
                unsafe { w.bits(0); }
                w.m_stop_det().set_bit();
                w.m_tx_abrt().set_bit();
                w
            });
            // i2c.i2c.ic_intr_mask.modify(|_, w| {
            //     w.m_stop_det().set_bit();
            //     w.m_tx_abrt().set_bit()
            // });
            // defmt::info!("irq mask: 0x{:x}", i2c.i2c.ic_intr_mask.read().bits());
        });

        let mut pos = 0;
        while pos < tx.len() {
            pos += i2c.lock(|i2c| i2c.write_internal_partial(&tx[pos..], true))?;

            if pos < tx.len() {
                // TODO: async block for some kind of interrupt.
                todo!();
            }
        }

        let result = Self::wait_stop_async(irq).await?;

        i2c.lock(|i2c| {
            i2c.i2c.ic_intr_mask.write(|w| unsafe { w.bits(0) });
            // i2c.i2c.ic_intr_mask.modify(|_, w| {
            //     w.m_stop_det().set_bit();
            //     w.m_tx_abrt().set_bit()
            // });
        });

        Ok(result)
    }
}

impl<T: Deref<Target = Block>, PINS> Read for I2C<T, PINS, Controller> {
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        let addr: u16 = addr.into();

        Self::validate(addr, None, Some(buffer.is_empty()))?;

        self.setup(addr);
        self.read_internal(buffer, true, true)
    }
}
impl<T: Deref<Target = Block>, PINS> WriteRead for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write_read(&mut self, addr: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), Error> {
        if true {
            return Err(Error::Todo);
        }
        let addr: u16 = addr.into();

        Self::validate(addr, Some(tx.is_empty()), Some(rx.is_empty()))?;
        self.setup(addr);

        self.write_internal(tx, false)?;
        self.read_internal(rx, true, true)
    }
}

impl<T: Deref<Target = Block>, PINS> Write for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write(&mut self, addr: u8, tx: &[u8]) -> Result<(), Error> {
        let addr: u16 = addr.into();
        Self::validate(addr, Some(tx.is_empty()), None)?;
        self.setup(addr);

        self.write_internal(tx, true)?;
        self.wait_stop()
    }
}

impl<T: Deref<Target = Block>, PINS> WriteIter for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write<B>(&mut self, address: u8, bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        self.write_iter(address, bytes)
    }
}

impl<T: Deref<Target = Block>, PINS> WriteIterRead for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write_iter_read<B>(
        &mut self,
        address: u8,
        bytes: B,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        self.write_iter_read(address, bytes, buffer)
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<T: Deref<Target = Block>, PINS> eh1::ErrorType for I2C<T, PINS, Controller> {
    type Error = Error;
}

#[cfg(feature = "eh1_0_alpha")]
impl<T: Deref<Target = Block>, PINS> eh1::I2c for I2C<T, PINS, Controller> {
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        Write::write(self, addr, bytes)
    }

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        Read::read(self, addr, buffer)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [eh1::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let addr: u16 = address.into();
        self.setup(addr);
        for i in 0..operations.len() {
            let last = i == operations.len() - 1;
            match &mut operations[i] {
                eh1::Operation::Read(buf) => self.read_internal(buf, false, last)?,
                eh1::Operation::Write(buf) => self.write_internal(buf, last)?,
            }
        }
        Ok(())
    }
}
