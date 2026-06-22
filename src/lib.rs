#![no_std]
#![warn(missing_docs)]
//! Library for the Ads8681 ADC.
//! This library provides both a sync and async implementation of the driver.
//! They are in the `synchronous` and `asynchronous` modules respectively.

use defmt::Format;

/// Contains an async implementation of the ads8681 driver over embedded_hal_async.
#[path = "."]
pub mod asynchronous {
    use bisync::asynchronous::*;
    #[allow(clippy::duplicate_mod)]
    mod inner;
    pub use inner::*;
}
/// Contains a sync implementation of the ads8681 driver over embedded_hal.
#[path = "."]
pub mod synchronous {
    use bisync::synchronous::*;
    #[allow(clippy::duplicate_mod)]
    mod inner;
    pub use inner::*;
}

/// Driver for the Ads8681.
#[derive(Debug)]
pub struct Ads8681<I> {
    interface: I,
}
/// An implementor of `CommandInterface` that can be used to construct a
/// Ads8681 driver.
#[derive(Debug, Clone)]
pub struct Ads8681SpiInterface<DRIVER>(pub DRIVER);
/// Produced in every write command and noop command. You can acquire one via
/// `get_data_output`. Contains a conversion result and other appended flags.
#[derive(Debug, Clone, Copy, Format)]
pub struct OutputDataWord(pub [u8; 4]);
impl OutputDataWord {
    /// Just get the conversion result from this [`OutputDataWord`].
    pub fn get_conversion_result(self) -> u16 {
        let number = u32::from_be_bytes(self.0);
        (number >> 16) as u16
    }
    /// Interpret the output of this [`OutputDataWord`] with the configuration
    /// stored in the [`DataOutCtl`] register.
    pub fn interpret(self, dataout_ctl: &DataOutCtl) -> InterpretedOutputDataWord {
        let number = u32::from_be_bytes(self.0);
        let conversion_result = (number >> 16) as u16;
        let mut shift_index = 16;
        let device_address = if dataout_ctl.include_device_addr_value() {
            shift_index -= 4;
            Some(DeviceAddress(((number >> shift_index) & 0b1111) as u8))
        } else {
            None
        };
        let adc_input_range = if dataout_ctl.include_range_value() {
            shift_index -= 4;
            Some(AdcInputRanges::from_bits(
                ((number >> shift_index) & 0b1111) as u8,
            ))
        } else {
            None
        };
        let avdd_alarm_high_flag = if dataout_ctl.include_active_vdd_h_flag() {
            shift_index -= 1;
            Some(number >> shift_index & 0b1 != 0)
        } else {
            None
        };
        let avdd_alarm_low_flag = if dataout_ctl.include_active_vdd_l_flag() {
            shift_index -= 1;
            Some(number >> shift_index & 0b1 != 0)
        } else {
            None
        };
        let input_alarm_high_flag = if dataout_ctl.include_active_in_h_flag() {
            shift_index -= 1;
            Some(number >> shift_index & 0b1 != 0)
        } else {
            None
        };
        let input_alarm_low_flag = if dataout_ctl.include_active_in_l_flag() {
            shift_index -= 1;
            Some(number >> shift_index & 0b1 != 0)
        } else {
            None
        };
        let parity_bits = if dataout_ctl.enable_parity_bits() {
            shift_index -= 2;
            Some(((number >> shift_index) & 0b11) as u8)
        } else {
            None
        };

        InterpretedOutputDataWord {
            conversion_result,
            device_address,
            adc_input_range,
            avdd_alarm_high_flag,
            avdd_alarm_low_flag,
            input_alarm_high_flag,
            input_alarm_low_flag,
            parity_bits,
        }
    }
}
/// An [`OutputDataWord`] can be interpreted for its meaning by using a
/// [`DataOutCtl`]. See [`OutputDataWord::interpret`].
#[derive(Debug, Clone, Copy, Format, PartialEq, Eq)]
pub struct InterpretedOutputDataWord {
    /// A 16 bit conversion result
    pub conversion_result: u16,
    /// Device address used for daisy chaining
    pub device_address: Option<DeviceAddress>,
    /// ADC settings (from range_sel)
    pub adc_input_range: Option<AdcInputRanges>,
    /// AVDD Alarm high voltage
    pub avdd_alarm_high_flag: Option<bool>,
    /// AVDD Alarm low voltage
    pub avdd_alarm_low_flag: Option<bool>,
    /// Input Alarm high voltage
    pub input_alarm_high_flag: Option<bool>,
    /// Input Alarm low voltage
    pub input_alarm_low_flag: Option<bool>,
    /// Parity bits for the whole returned message. This library currently
    /// does not perform any checks using the parity bits.
    pub parity_bits: Option<u8>,
}
/// A 9 bit address referring to one of the Ads8681's registers. Note that the
/// LSB bit is ignored in half word (u16) operations. There's no reason to use
/// this type unless you're directly interfacing using an implementor of
/// `CommandInterface`.
///
/// The ads8681 registers are all 32 bit registers but are byte indexed, so
/// [`NineBitAddress::higher_half`] and [`NineBitAddress::higher_quarter`]
/// are needed for full access.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct NineBitAddress {
    pub(crate) register_address: u8,
}
impl NineBitAddress {
    /// Form a full 32 bit input command word.
    pub fn form_full_command(self, command: CommandBits, value_bits: u16) -> [u8; 4] {
        [
            (command as u8) << 1,
            self.register_address,
            (value_bits >> 8) as u8,
            (value_bits & 0x00FF) as u8,
        ]
    }
    const fn register(addr: u8) -> Self {
        Self {
            register_address: addr,
        }
    }
    /// Change this address to refer to 1 byte higher than the original.
    pub const fn one_byte_higher(self) -> Self {
        Self {
            register_address: self.register_address + 1,
        }
    }
}
/// What sending a signal through the reset pin does.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum ResetPinFunction {
    /// Full device initialization
    PorClassReset = 0b0,
    /// Application reset (only user programmed modes cleared)
    ApplicationReset = 0b1,
}
impl ResetPinFunction {
    const fn from_bits(bits: u8) -> Self {
        if bits == 0 {
            Self::PorClassReset
        } else {
            Self::ApplicationReset
        }
    }
    const fn into_bits(self) -> u8 {
        self as u8
    }
}
/// The reset power control register is responsible for enabling/disabling
/// some features on the ads8681. Note that writing to it requires unlocking
/// the register via the WKEY field first. The driver in this library should
/// handle that for you automatically.
#[bitfields::bitfield(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct RstPwrCtrl {
    #[bits(1)]
    pub power_down: bool,
    #[bits(1)]
    pub nap_mode_enabled: bool,
    #[bits(1)]
    pub reset_pin_function: ResetPinFunction,
    #[bits(1)]
    _skip: bool,
    #[bits(1)]
    pub input_alarm_enabled: bool,
    #[bits(1)]
    pub vdd_alarm_enabled: bool,
    #[bits(2)]
    _reserved: u8,
}
#[bitfields::bitfield(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct SdiMode {
    #[bits(1)]
    cphase: bool,
    #[bits(1)]
    cpol: bool,
    #[bits(6)]
    _padding: u8,
}
/// How the SDO pin should behave.
#[repr(u8)]
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum SdoMode {
    SameAsSdi = 0b00,
    InvalidConfiguration = 0b10,
    AdcControllerClockOrSourceSynchronous = 0b11,
}
impl SdoMode {
    const fn from_bits(bits: u8) -> Self {
        match bits {
            0b00 | 0b01 => Self::SameAsSdi,
            0b11 => Self::AdcControllerClockOrSourceSynchronous,
            _ => Self::InvalidConfiguration,
        }
    }
    const fn into_bits(self) -> u8 {
        self as u8
    }
}
/// How the SDO1 pin should behave. Useful for daisy chaining.
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum Sdo1Mode {
    AlwaysTriStated = 0b00,
    FunctionsAsAlarm = 0b01,
    FunctionsAsGPO = 0b10,
    TwoBitWithSdo0 = 0b11,
}
impl Sdo1Mode {
    const fn from_bits(bits: u8) -> Self {
        match bits {
            0b00 => Self::AlwaysTriStated,
            0b01 => Self::FunctionsAsAlarm,
            0b10 => Self::FunctionsAsGPO,
            0b11 => Self::TwoBitWithSdo0,
            _ => panic!("Bad SDO1 mode?"),
        }
    }
    const fn into_bits(self) -> u8 {
        self as u8
    }
}
#[bitfields::bitfield(u16)]
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct SdoCtl {
    #[bits(2)]
    pub sdo_mode: SdoMode,
    #[bits(4)]
    _reserved: u8,
    #[bits(1)]
    pub use_internal_clock: bool,
    #[bits(1)]
    _reserved2: u8,
    #[bits(2)]
    pub sdo1_config: Sdo1Mode,
    #[bits(2)]
    _reserved3: u8,
    #[bits(1)]
    pub gpo_val: bool,
    #[bits(3)]
    _reserved4: u8,
}
/// The [`DataVal`] field in the [`DataOutCtl`] register decides how the conversion
/// data bits are encoded.
#[allow(missing_docs)]
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DataVal {
    /// Output normal conversion data
    ConversionData = 0b0,
    AllZeros = 0b100,
    AllOnes = 0b101,
    AlternatingZerosAndOnes = 0b110,
    AlternatingDoubleZerosAndOnes = 0b111,
}
impl DataVal {
    const fn from_bits(bits: u8) -> Self {
        match bits {
            0b100 => Self::AllZeros,
            0b101 => Self::AllOnes,
            0b110 => Self::AlternatingZerosAndOnes,
            0b111 => Self::AlternatingDoubleZerosAndOnes,
            _ => Self::ConversionData,
        }
    }
    const fn into_bits(self) -> u8 {
        self as u8
    }
}
#[bitfields::bitfield(u16)]
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct DataOutCtl {
    #[bits(3)]
    pub data_val: DataVal,
    #[bits(1)]
    pub enable_parity_bits: bool,
    #[bits(4)]
    _reserved: u8,
    #[bits(1)]
    pub include_range_value: bool,
    #[bits(1)]
    _reserved2: u8,
    #[bits(1)]
    pub include_active_in_h_flag: bool,
    #[bits(1)]
    pub include_active_in_l_flag: bool,
    #[bits(1)]
    pub include_active_vdd_h_flag: bool,
    #[bits(1)]
    pub include_active_vdd_l_flag: bool,
    #[bits(1)]
    pub include_device_addr_value: bool,
    #[bits(1)]
    _reserved3: u8,
}
/// ADC input ranges relative to V_ref. Refer to the datasheet for more details.
#[repr(u8)]
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum AdcInputRanges {
    PlusMinusThree = 0b0000,
    PlusMinusTwoPointFive = 0b0001,
    PlusMinusOnePointFive = 0b0010,
    PlusMinusOnePointTwoFive = 0b0011,
    /// +- 0.625
    PlusMinusSubZero = 0b0100,
    Three = 0b1000,
    TwoPointFive = 0b1001,
    OnePointFive = 0b1010,
    OnePointTwoFive = 0b1011,
}
impl AdcInputRanges {
    const fn from_bits(bits: u8) -> Self {
        match bits {
            0b0000 => Self::PlusMinusThree,
            0b0001 => Self::PlusMinusTwoPointFive,
            0b0010 => Self::PlusMinusOnePointFive,
            0b0011 => Self::PlusMinusOnePointTwoFive,
            0b0100 => Self::PlusMinusSubZero,
            0b1000 => Self::Three,
            0b1001 => Self::TwoPointFive,
            0b1010 => Self::OnePointFive,
            0b1011 => Self::OnePointTwoFive,
            _ => panic!("Bad adc input range?"),
        }
    }
    const fn into_bits(self) -> u8 {
        self as u8
    }
}
#[bitfields::bitfield(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct RangeSel {
    /// Refer to datasheet for values
    #[bits(4)]
    range_sel: AdcInputRanges,
    #[bits(2)]
    _reserved: u8,
    #[bits(1)]
    pub disable_internal_adc_reference: bool,
    #[bits(1)]
    _reserved2: u8,
}
#[bitfields::bitfield(u16)]
#[derive(Clone, Copy, PartialEq, Eq, Format)]
pub struct Alarm {
    #[bits(1)]
    pub ovw_alarm: bool,
    #[bits(3)]
    _reserved: u8,
    #[bits(1)]
    pub trip_high_input_voltage: bool,
    #[bits(1)]
    pub trip_low_input_voltage: bool,
    #[bits(1)]
    pub trip_high_avdd: bool,
    #[bits(1)]
    pub trip_low_avdd: bool,
    #[bits(2)]
    _reserved2: u8,
    #[bits(1)]
    pub high_input_voltage: bool,
    #[bits(1)]
    pub low_input_voltage: bool,
    #[bits(2)]
    _reserved3: u8,
    #[bits(1)]
    pub high_avdd_voltage: bool,
    #[bits(1)]
    pub low_avdd_voltage: bool,
}

#[repr(u8)]
#[allow(clippy::unusual_byte_groupings, missing_docs)]
/// u8 input commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum CommandBits {
    Noop = 0b0,
    ClearHword = 0b11000_00,
    ReadHword = 0b11001_00,
    Read = 0b01001_00,
    WriteHword = 0b11010_00,
    WriteHwordMs = 0b11010_01,
    WriteHwordLs = 0b11010_10,
    SetHword = 0b11011_00,
}
/// A 4 bit device address. Ignore the upper 4 bits in the [`u8`] inside.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct DeviceAddress(pub u8);

/// Contains constant register addresses.
#[allow(missing_docs)]
pub mod registers {
    use super::NineBitAddress;
    /// Value the WKEY field in the reset power control register should be set
    /// to unlock the other fields.
    pub const WKEY_VALUE: u8 = 0x69;
    pub const DEVICE_ID_REG: NineBitAddress = NineBitAddress::register(0x00);
    pub const RST_PWRCTL_REG: NineBitAddress = NineBitAddress::register(0x04);
    pub const SDI_CTL_REG: NineBitAddress = NineBitAddress::register(0x08);
    pub const SDO_CTL_REG: NineBitAddress = NineBitAddress::register(0x0C);
    pub const DATAOUT_CTL_REG: NineBitAddress = NineBitAddress::register(0x10);
    pub const RANGE_SEL_REG: NineBitAddress = NineBitAddress::register(0x14);
    pub const ALARM_REG: NineBitAddress = NineBitAddress::register(0x20);
    pub const ALARM_H_TH_REG: NineBitAddress = NineBitAddress::register(0x24);
    pub const ALARM_L_TH_REG: NineBitAddress = NineBitAddress::register(0x28);
}
