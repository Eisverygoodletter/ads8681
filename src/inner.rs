//! Actual implementation details. This crate uses the bisync library to
//! generate both sync and async implementations from one async implementation.
use super::{bisync, only_async, only_sync};
use crate::{
    Ads8681, Ads8681SpiInterface, Ads8681WaitMode, Alarm, CommandBits, DataOutCtl, DeviceAddress,
    NineBitAddress, OutputDataWord, RangeSel, RstPwrCtrl, SdiMode, SdoCtl, registers,
};

/// Implementations for [`crate::Ads8681WaitMode`] which are sync/async
/// dependent.
#[bisync]
#[allow(async_fn_in_trait)]
pub trait WaitModeFeatures {
    /// An error that occurs while accessing the RVS pin
    type WaitError;
    /// Wait until the RVS pin is high. This is determined by the internal
    /// [`crate::Ads8681WaitMode`]. Returns true if an spi operation delay
    /// should be issued to the driver.
    async fn wait_for_rvs(&mut self) -> Result<bool, Self::WaitError>;
}
#[only_sync]
impl<P: embedded_hal::digital::InputPin> WaitModeFeatures for Ads8681WaitMode<P> {
    type WaitError = <P as embedded_hal::digital::ErrorType>::Error;
    fn wait_for_rvs(&mut self) -> Result<bool, Self::WaitError> {
        match self {
            // todo: implement t_conv delay
            Ads8681WaitMode::AsyncBlocking(_) => unreachable!(),
            Ads8681WaitMode::SyncBlocking(pin) => loop {
                if pin.is_high()? {
                    break Ok(false);
                }
            },
        }
    }
}
#[only_async]
impl<P: embedded_hal_async::digital::Wait + embedded_hal::digital::InputPin> WaitModeFeatures
    for Ads8681WaitMode<P>
{
    type WaitError = <P as embedded_hal::digital::ErrorType>::Error;
    async fn wait_for_rvs(&mut self) -> Result<bool, Self::WaitError> {
        match self {
            Ads8681WaitMode::AsyncBlocking(pin) => {
                pin.wait_for_high().await?;
                Ok(false)
            }
            Ads8681WaitMode::SyncBlocking(pin) => loop {
                if pin.is_high()? {
                    break Ok(false);
                }
            },
        }
    }
}

/// Interface for input and output commands from the Ads8681.
#[bisync]
#[allow(async_fn_in_trait)]
pub trait CommandInterface {
    /// The error type from the underlying driver.
    type Error;
    /// Wait until the RVS pin is high. This is determined by the internal
    /// [`crate::Ads8681WaitMode`]. Returns true if an spi operation delay
    /// should be issued to the driver.
    async fn wait_for_rvs(&mut self) -> Result<(), Self::Error>;
    /// Send a NOOP spi message.
    async fn noop(&mut self) -> Result<OutputDataWord, Self::Error>;
    /// Clear an hword at the given address. All bits marked 1 in the
    /// clear_bit_mask are cleared (reset to 0) with other bits unchanged.
    async fn clear_hword(
        &mut self,
        addr: NineBitAddress,
        clear_bit_mask: u16,
    ) -> Result<OutputDataWord, Self::Error>;
    /// Read an hword from the given address.
    async fn read_hword(&mut self, addr: NineBitAddress) -> Result<u16, Self::Error>;
    /// Read a u8 from the given address.
    async fn read(&mut self, addr: NineBitAddress) -> Result<u8, Self::Error>;
    /// Write an hword to the given address.
    async fn write_hword(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error>;
    /// Write the most significant byte of the given value to the given register.
    /// The least significant byte is ignored.
    async fn write_hword_ms(
        &mut self,
        addr: NineBitAddress,
        ls_ignored: u16,
    ) -> Result<OutputDataWord, Self::Error>;
    /// Write the least significant byte of the given value to the given register.
    /// The most significant byte is ignored.
    async fn write_hword_ls(
        &mut self,
        addr: NineBitAddress,
        ms_ignored: u16,
    ) -> Result<OutputDataWord, Self::Error>;
    /// Set all bits with the mask marked as 1 as 1, leaving other bits unchanged.
    async fn set_hword(
        &mut self,
        addr: NineBitAddress,
        write_bit_mask: u16,
    ) -> Result<OutputDataWord, Self::Error>;
}

#[only_sync]
use embedded_hal as eh;
#[only_async]
use embedded_hal_async as eh;

#[bisync]
impl<T, P> CommandInterface for crate::Ads8681SpiInterface<T, P>
where
    T: eh::spi::SpiDevice,
    Ads8681WaitMode<P>: WaitModeFeatures,
{
    type Error = Ads8681Error<
        <T as eh::spi::ErrorType>::Error,
        <Ads8681WaitMode<P> as WaitModeFeatures>::WaitError,
    >;
    async fn wait_for_rvs(&mut self) -> Result<(), Self::Error> {
        if self
            .1
            .wait_for_rvs()
            .await
            .map_err(Ads8681Error::RvsInduced)?
        {
            // refer to datasheet 6.5 Electrical Characteristics t_conv.
            const MAX_ADS8681_CONVERSION_TIME_NS: u32 = 665;
            self.0
                .transaction(&mut [eh::spi::Operation::DelayNs(MAX_ADS8681_CONVERSION_TIME_NS)])
                .await
                .map_err(Ads8681Error::SpiInduced)?;
        }
        Ok(())
    }
    /// TODO: See if the ADS8681 clocks out conversion data from the previous
    /// frame when writing the noop.
    async fn noop(&mut self) -> Result<OutputDataWord, Self::Error> {
        let mut buf = [0, 0, 0, 0];
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok(OutputDataWord(buf))
    }
    async fn clear_hword(
        &mut self,
        addr: NineBitAddress,
        clear_bit_mask: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::ClearHword, clear_bit_mask);
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok(OutputDataWord(buf))
    }
    async fn read_hword(&mut self, addr: NineBitAddress) -> Result<u16, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::ReadHword, 0);
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok((buf[0] as u16) << 8 | (buf[1] as u16))
    }
    async fn read(&mut self, addr: NineBitAddress) -> Result<u8, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::Read, 0);
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok(buf[0])
    }
    async fn write_hword(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::WriteHword, value);
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok(OutputDataWord(buf))
    }
    async fn write_hword_ms(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::WriteHwordMs, value);
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok(OutputDataWord(buf))
    }
    async fn write_hword_ls(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::WriteHwordLs, value);
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok(OutputDataWord(buf))
    }
    async fn set_hword(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::SetHword, value);
        self.0.write(&buf).await.map_err(Ads8681Error::SpiInduced)?;
        self.wait_for_rvs().await?;
        self.0
            .read(&mut buf)
            .await
            .map_err(Ads8681Error::SpiInduced)?;
        Ok(OutputDataWord(buf))
    }
}

/// An error from the [`Ads8681`] driver.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum Ads8681Error<SPIERROR, PINERROR> {
    /// Error that occurs while operating the spi interface.
    SpiInduced(SPIERROR),
    /// Error that occurs while accessing the RVS pin. This variant can
    /// be ignored if you are not using an RVS pin.
    RvsInduced(PINERROR),
}
#[bisync]
#[allow(missing_docs)]
#[allow(async_fn_in_trait)]
pub trait Ads8681Features<I: CommandInterface> {
    async fn get_data_output(&mut self) -> Result<OutputDataWord, I::Error>;
    async fn set_device_address(
        &mut self,
        address: DeviceAddress,
    ) -> Result<OutputDataWord, I::Error>;
    async fn get_device_address(&mut self) -> Result<DeviceAddress, I::Error>;
    async fn get_rst_pwrctrl(&mut self) -> Result<RstPwrCtrl, I::Error>;
    async fn set_rst_pwrctrl(&mut self, value: RstPwrCtrl) -> Result<OutputDataWord, I::Error>;
    async fn get_sdi_ctl(&mut self) -> Result<SdiMode, I::Error>;
    async fn set_sdi_ctl(&mut self, mode: SdiMode) -> Result<OutputDataWord, I::Error>;
    async fn get_sdo_ctl(&mut self) -> Result<SdoCtl, I::Error>;
    async fn set_sdo_ctl(&mut self, sdo_config: SdoCtl) -> Result<OutputDataWord, I::Error>;
    async fn get_dataout_ctl(&mut self) -> Result<DataOutCtl, I::Error>;
    async fn set_dataout_ctl(
        &mut self,
        dataout_ctl: DataOutCtl,
    ) -> Result<OutputDataWord, I::Error>;
    async fn get_range_sel(&mut self) -> Result<RangeSel, I::Error>;
    async fn set_range_sel(&mut self, range_sel: RangeSel) -> Result<OutputDataWord, I::Error>;
    async fn get_alarm(&mut self) -> Result<Alarm, I::Error>;
    async fn get_alarm_hysteresis(&mut self) -> Result<u8, I::Error>;
    async fn set_alarm_hysteresis(&mut self, hysteresis: u8) -> Result<OutputDataWord, I::Error>;
    async fn get_inp_alarm_high_threshold(&mut self) -> Result<u16, I::Error>;
    async fn set_inp_alarm_high_threshold(
        &mut self,
        threshold: u16,
    ) -> Result<OutputDataWord, I::Error>;
    async fn get_inp_alarm_low_threshold(&mut self) -> Result<u16, I::Error>;
    async fn set_inp_alarm_low_threshold(
        &mut self,
        threshold: u16,
    ) -> Result<OutputDataWord, I::Error>;
}

impl<I, P> Ads8681<Ads8681SpiInterface<I, P>>
where
    I: eh::spi::SpiDevice,
    Ads8681WaitMode<P>: WaitModeFeatures,
{
    /// Construct a new driver instance using a SPI driver. The [`CommandInterface`]
    /// is handled internally.
    #[only_sync]
    pub fn new_blocking(spi: I, pin: P) -> Self {
        Self {
            interface: Ads8681SpiInterface(spi, Ads8681WaitMode::SyncBlocking(pin)),
        }
    }
    /// Construct a new driver instance using a SPI driver. The [`CommandInterface`]
    /// is handled internally.
    #[only_async]
    pub fn new_async(spi: I, wait_mode: Ads8681WaitMode<P>) -> Self {
        Self {
            interface: Ads8681SpiInterface(spi, wait_mode),
        }
    }
}

#[bisync]
impl<I: CommandInterface> Ads8681Features<I> for Ads8681<I> {
    /// Send a noop command to get a [`OutputDataWord`].
    async fn get_data_output(&mut self) -> Result<OutputDataWord, I::Error> {
        let output = self.interface.noop().await?;
        Ok(output)
    }
    /// Set the 4 bit device address. Useful for daisy chaining.
    async fn set_device_address(
        &mut self,
        address: DeviceAddress,
    ) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword_ls(
                registers::DEVICE_ID_REG.one_byte_higher().one_byte_higher(),
                address.0 as u16,
            )
            .await?;
        Ok(output)
    }
    /// Get the 4 bit device address used for daisy chaining.
    async fn get_device_address(&mut self) -> Result<DeviceAddress, I::Error> {
        let output = self
            .interface
            .read(registers::DEVICE_ID_REG.one_byte_higher().one_byte_higher())
            .await?;
        Ok(DeviceAddress(output))
    }
    /// Get the value of the reset power control register. Note that this does
    /// not include the WKEY field, which this library will handle for you.
    async fn get_rst_pwrctrl(&mut self) -> Result<RstPwrCtrl, I::Error> {
        let output = self.interface.read(registers::RST_PWRCTL_REG).await?;
        Ok(RstPwrCtrl::from(output))
    }
    /// This operation is different to other register accesses in that 2 extra
    /// writes will be performed for the WKEY bits to unlock and lock the
    /// protected registers.
    async fn set_rst_pwrctrl(&mut self, value: RstPwrCtrl) -> Result<OutputDataWord, I::Error> {
        // write WKEY first to allow writes to other bits
        self.interface
            .write_hword_ms(registers::RST_PWRCTL_REG, registers::WKEY_VALUE as u16)
            .await?;
        // write value
        self.interface
            .write_hword_ls(registers::RST_PWRCTL_REG, value.into_bits() as u16)
            .await?;
        // write 0 to WKEY to lock it (not super necessary but better to be safe)
        let output = self
            .interface
            .write_hword_ms(registers::RST_PWRCTL_REG, 0)
            .await?;
        Ok(output)
    }
    /// Get the SDI pin configuration
    async fn get_sdi_ctl(&mut self) -> Result<SdiMode, I::Error> {
        let output = self.interface.read(registers::SDI_CTL_REG).await?;
        Ok(SdiMode::from(output))
    }
    /// Set the SDI pin configuration
    async fn set_sdi_ctl(&mut self, mode: SdiMode) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword_ls(registers::SDI_CTL_REG, u8::from(mode) as u16)
            .await?;
        Ok(output)
    }
    /// Get sdo pin configuration
    async fn get_sdo_ctl(&mut self) -> Result<SdoCtl, I::Error> {
        let output = self.interface.read_hword(registers::SDO_CTL_REG).await?;
        Ok(SdoCtl::from(output))
    }
    /// Set sdo pin configuration
    async fn set_sdo_ctl(&mut self, sdo_config: SdoCtl) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword(registers::SDO_CTL_REG, sdo_config.into_bits())
            .await?;
        Ok(output)
    }
    /// Get the contents of the data out register. This can be combined with
    /// [`OutputDataWord`] to produce interpretable results.
    async fn get_dataout_ctl(&mut self) -> Result<DataOutCtl, I::Error> {
        let output = self
            .interface
            .read_hword(registers::DATAOUT_CTL_REG)
            .await?;
        Ok(DataOutCtl::from(output))
    }
    /// Set the data out register. This changes how [`OutputDataWord`]s are
    /// encoded.
    async fn set_dataout_ctl(
        &mut self,
        dataout_ctl: DataOutCtl,
    ) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword(registers::DATAOUT_CTL_REG, dataout_ctl.into_bits())
            .await?;
        Ok(output)
    }
    /// Get information about the ADC range selection.
    async fn get_range_sel(&mut self) -> Result<RangeSel, I::Error> {
        let output = self.interface.read(registers::RANGE_SEL_REG).await?;
        Ok(RangeSel::from(output))
    }
    /// Set the ADC's range selection info. see [`RangeSel`] and refer to the
    /// datasheet for more information.
    async fn set_range_sel(&mut self, range_sel: RangeSel) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword_ls(registers::RANGE_SEL_REG, range_sel.into_bits() as u16)
            .await?;
        Ok(output)
    }
    /// Get information about alarm flags.
    async fn get_alarm(&mut self) -> Result<Alarm, I::Error> {
        let output = self.interface.read_hword(registers::ALARM_REG).await?;
        Ok(Alarm::from(output))
    }
    /// Get the alarm hysteresis value. Refer to the datasheet for how hysteresis
    /// is used.
    async fn get_alarm_hysteresis(&mut self) -> Result<u8, I::Error> {
        let output = self
            .interface
            .read(
                registers::ALARM_H_TH_REG
                    .one_byte_higher()
                    .one_byte_higher()
                    .one_byte_higher(),
            )
            .await?;
        Ok(output)
    }
    /// Set the alarm hysteresis value.
    async fn set_alarm_hysteresis(&mut self, hysteresis: u8) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword_ms(
                registers::ALARM_H_TH_REG
                    .one_byte_higher()
                    .one_byte_higher()
                    .one_byte_higher(),
                (hysteresis as u16) << 8,
            )
            .await?;
        Ok(output)
    }
    /// Get the high-end threshold for the input voltage alarm.
    async fn get_inp_alarm_high_threshold(&mut self) -> Result<u16, I::Error> {
        let output = self.interface.read_hword(registers::ALARM_H_TH_REG).await?;
        Ok(output)
    }
    /// Set the high-end threshold for the input voltage alarm.
    async fn set_inp_alarm_high_threshold(
        &mut self,
        threshold: u16,
    ) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword(registers::ALARM_H_TH_REG, threshold)
            .await?;
        Ok(output)
    }
    /// Get the low-end threshold for the input voltage alarm.
    async fn get_inp_alarm_low_threshold(&mut self) -> Result<u16, I::Error> {
        let output = self.interface.read_hword(registers::ALARM_L_TH_REG).await?;
        Ok(output)
    }
    /// Set the low-end threshold for the input voltage alarm.
    async fn set_inp_alarm_low_threshold(
        &mut self,
        threshold: u16,
    ) -> Result<OutputDataWord, I::Error> {
        let output = self
            .interface
            .write_hword(registers::ALARM_L_TH_REG, threshold)
            .await?;
        Ok(output)
    }
}
