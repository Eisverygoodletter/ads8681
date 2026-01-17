//! Actual implementation details. This crate uses the bisync library to
//! generate both sync and async implementations from one async implementation.
use super::{bisync, only_async, only_sync};
use crate::{
    Ads8681SpiInterface, Alarm, CommandBits, DataOutCtl, DeviceAddress, NineBitAddress,
    OutputDataWord, RangeSel, RstPwrCtrl, SdiMode, SdoCtl, registers,
};

/// Interface for input and output commands from the Ads8681.
#[bisync]
#[allow(async_fn_in_trait)]
pub trait CommandInterface {
    /// The error type from the underlying driver.
    type Error;
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
impl<T> CommandInterface for crate::Ads8681SpiInterface<T>
where
    T: eh::spi::SpiDevice,
{
    type Error = <T as embedded_hal::spi::ErrorType>::Error;
    async fn noop(&mut self) -> Result<OutputDataWord, Self::Error> {
        let mut buf = [0, 0, 0, 0];
        self.0.transfer_in_place(&mut buf).await?;
        Ok(OutputDataWord(buf))
    }
    async fn clear_hword(
        &mut self,
        addr: NineBitAddress,
        clear_bit_mask: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::ClearHword, clear_bit_mask);
        self.0.transfer_in_place(&mut buf).await?;
        Ok(OutputDataWord(buf))
    }
    async fn read_hword(&mut self, addr: NineBitAddress) -> Result<u16, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::ClearHword, 0);
        self.0.transfer_in_place(&mut buf).await?;
        Ok((buf[0] as u16) << 8 | (buf[1] as u16))
    }
    async fn read(&mut self, addr: NineBitAddress) -> Result<u8, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::Read, 0);
        self.0.transfer_in_place(&mut buf).await?;
        Ok(buf[0])
    }
    async fn write_hword(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::WriteHword, value);
        self.0.transfer_in_place(&mut buf).await?;
        Ok(OutputDataWord(buf))
    }
    async fn write_hword_ms(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::WriteHwordMs, value);
        self.0.transfer_in_place(&mut buf).await?;
        Ok(OutputDataWord(buf))
    }
    async fn write_hword_ls(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::WriteHwordLs, value);
        self.0.transfer_in_place(&mut buf).await?;
        Ok(OutputDataWord(buf))
    }
    async fn set_hword(
        &mut self,
        addr: NineBitAddress,
        value: u16,
    ) -> Result<OutputDataWord, Self::Error> {
        let mut buf = addr.form_full_command(CommandBits::SetHword, value);
        self.0.transfer_in_place(&mut buf).await?;
        Ok(OutputDataWord(buf))
    }
}

/// An error from the [`Ads8681`] driver.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Ads8681Error<I: CommandInterface> {
    /// An underlying error produced by the interface implementor
    Underlying(I::Error),
}

/// Driver for the Ads8681.
#[derive(Debug)]
pub struct Ads8681<I> {
    interface: I,
}
impl<I: CommandInterface> Ads8681<I> {
    /// Construct a new driver instance using a provider of the [`CommandInterface`].
    pub fn new_with_interface(interface: I) -> Self {
        Self { interface }
    }
}
impl<I: eh::spi::SpiDevice> Ads8681<Ads8681SpiInterface<I>> {
    /// Construct a new driver instance using a SPI driver. The [`CommandInterface`]
    /// is handled internally.
    pub fn new_with_spi(spi: I) -> Self {
        Self {
            interface: Ads8681SpiInterface(spi),
        }
    }
}

#[bisync]
impl<I: CommandInterface> Ads8681<I> {
    /// Send a noop command to get a [`OutputDataWord`].
    pub async fn get_data_output(&mut self) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .noop()
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Set the 4 bit device address. Useful for daisy chaining.
    pub async fn set_device_address(
        &mut self,
        address: DeviceAddress,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword_ls(registers::DEVICE_ID_REG.higher_half(), address.0 as u16)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get the 4 bit device address used for daisy chaining.
    pub async fn get_device_address(&mut self) -> Result<DeviceAddress, Ads8681Error<I>> {
        let output = self
            .interface
            .read(registers::DEVICE_ID_REG.higher_half())
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(DeviceAddress(output))
    }
    /// Get the value of the reset power control register. Note that this does
    /// not include the WKEY field, which this library will handle for you.
    pub async fn get_rst_pwrctrl(&mut self) -> Result<RstPwrCtrl, Ads8681Error<I>> {
        let output = self
            .interface
            .read(registers::RST_PWRCTL_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(RstPwrCtrl::from(output))
    }
    /// This operation is different to other register accesses in that 2 extra
    /// writes will be performed for the WKEY bits to unlock and lock the
    /// protected registers.
    pub async fn set_rst_pwrctrl(
        &mut self,
        value: RstPwrCtrl,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        // write WKEY first to allow writes to other bits
        self.interface
            .write_hword_ms(registers::RST_PWRCTL_REG, registers::WKEY_VALUE as u16)
            .await
            .map_err(Ads8681Error::Underlying)?;
        // write value
        self.interface
            .write_hword_ls(registers::RST_PWRCTL_REG, value.into_bits() as u16)
            .await
            .map_err(Ads8681Error::Underlying)?;
        // write 0 to WKEY to lock it (not super necessary but better to be safe)
        let output = self
            .interface
            .write_hword_ms(registers::RST_PWRCTL_REG, 0)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get the SDI pin configuration
    pub async fn get_sdi_ctl(&mut self) -> Result<SdiMode, Ads8681Error<I>> {
        let output = self
            .interface
            .read(registers::SDI_CTL_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(SdiMode::from(output))
    }
    /// Set the SDI pin configuration
    pub async fn set_sdi_ctl(&mut self, mode: SdiMode) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword_ls(registers::SDI_CTL_REG, u8::from(mode) as u16)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get sdo pin configuration
    pub async fn get_sdo_ctl(&mut self) -> Result<SdoCtl, Ads8681Error<I>> {
        let output = self
            .interface
            .read_hword(registers::SDO_CTL_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(SdoCtl::from(output))
    }
    /// Set sdo pin configuration
    pub async fn set_sdo_ctl(
        &mut self,
        sdo_config: SdoCtl,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword(registers::SDO_CTL_REG, sdo_config.into_bits())
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get the contents of the data out register. This can be combined with
    /// [`OutputDataWord`] to produce interpretable results.
    pub async fn get_dataout_ctl(&mut self) -> Result<DataOutCtl, Ads8681Error<I>> {
        let output = self
            .interface
            .read_hword(registers::DATAOUT_CTL_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(DataOutCtl::from(output))
    }
    /// Set the data out register. This changes how [`OutputDataWord`]s are
    /// encoded.
    pub async fn set_dataout_ctl(
        &mut self,
        dataout_ctl: DataOutCtl,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword(registers::DATAOUT_CTL_REG, dataout_ctl.into_bits())
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get information about the ADC range selection.
    pub async fn get_range_sel(&mut self) -> Result<RangeSel, Ads8681Error<I>> {
        let output = self
            .interface
            .read(registers::RANGE_SEL_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(RangeSel::from(output))
    }
    /// Set the ADC's range selection info. see [`RangeSel`] and refer to the
    /// datasheet for more information.
    pub async fn set_range_sel(
        &mut self,
        range_sel: RangeSel,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword_ls(registers::RANGE_SEL_REG, range_sel.into_bits() as u16)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get information about alarm flags.
    pub async fn get_alarm(&mut self) -> Result<Alarm, Ads8681Error<I>> {
        let output = self
            .interface
            .read_hword(registers::ALARM_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(Alarm::from(output))
    }
    /// Get the alarm hysteresis value. Refer to the datasheet for how hysteresis
    /// is used.
    pub async fn get_alarm_hysteresis(&mut self) -> Result<u8, Ads8681Error<I>> {
        let output = self
            .interface
            .read(registers::ALARM_H_TH_REG.higher_half().higher_quarter())
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Set the alarm hysteresis value.
    pub async fn set_alarm_hysteresis(
        &mut self,
        hysteresis: u8,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword_ms(
                registers::ALARM_H_TH_REG.higher_half(),
                (hysteresis as u16) << 8,
            )
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get the high-end threshold for the input voltage alarm.
    pub async fn get_inp_alarm_high_threshold(&mut self) -> Result<u16, Ads8681Error<I>> {
        let output = self
            .interface
            .read_hword(registers::ALARM_H_TH_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Set the high-end threshold for the input voltage alarm.
    pub async fn set_inp_alarm_high_threshold(
        &mut self,
        threshold: u16,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword(registers::ALARM_H_TH_REG, threshold)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Get the low-end threshold for the input voltage alarm.
    pub async fn get_inp_alarm_low_threshold(&mut self) -> Result<u16, Ads8681Error<I>> {
        let output = self
            .interface
            .read_hword(registers::ALARM_L_TH_REG)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
    /// Set the low-end threshold for the input voltage alarm.
    pub async fn set_inp_alarm_low_threshold(
        &mut self,
        threshold: u16,
    ) -> Result<OutputDataWord, Ads8681Error<I>> {
        let output = self
            .interface
            .write_hword(registers::ALARM_L_TH_REG, threshold)
            .await
            .map_err(Ads8681Error::Underlying)?;
        Ok(output)
    }
}
