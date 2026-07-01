use ads8681::{Ads8681, DataOutCtlBuilder, RangeSelBuilder, synchronous::Ads8681Features};
use embedded_hal_mock::eh1::digital::Mock as MockPin;
use embedded_hal_mock::eh1::digital::State;
use embedded_hal_mock::eh1::digital::Transaction as PTransaction;
use embedded_hal_mock::eh1::spi::Mock as MockSpi;
use embedded_hal_mock::eh1::spi::Transaction;
#[test]
fn noop() {
    let expectations = [
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0, 0, 0, 0]),
        Transaction::transaction_end(),
        Transaction::transaction_start(),
        Transaction::read_vec(vec![0x12, 0x34, 0, 0]),
        Transaction::transaction_end(),
    ];
    let spi = MockSpi::new(&expectations);
    let mut done_ender = spi.clone();
    let pin = MockPin::new(&[
        PTransaction::get(State::Low),
        PTransaction::get(State::High),
    ]);
    let mut pin_done = pin.clone();
    let mut adc: Ads8681<ads8681::Ads8681SpiInterface<_, MockPin>> =
        Ads8681::new_blocking(spi, pin);
    let output_data_word = adc.get_data_output().unwrap();
    assert_eq!(output_data_word.get_conversion_result(), 0x1234);
    done_ender.done();
    pin_done.done();
}
#[test]
fn write_data_out() {
    let expectations = [
        Transaction::transaction_start(),
        Transaction::write_vec(vec![0b11010_000, 0x10, 0b01111101, 0b00000000]),
        Transaction::transaction_end(),
        Transaction::transaction_start(),
        Transaction::read_vec(vec![0, 0, 0, 0]),
        Transaction::transaction_end(),
    ];
    let spi = MockSpi::new(&expectations);
    let mut done_ender = spi.clone();
    let pin = MockPin::new(&[
        PTransaction::get(State::Low),
        PTransaction::get(State::High),
    ]);
    let mut pin_done = pin.clone();
    let mut adc: Ads8681<ads8681::Ads8681SpiInterface<_, MockPin>> =
        Ads8681::new_blocking(spi, pin);
    let data_out = DataOutCtlBuilder::new()
        .with_data_val(ads8681::DataVal::ConversionData)
        .with_enable_parity_bits(false)
        .with_include_range_value(true)
        .with_include_active_in_h_flag(true)
        .with_include_active_in_l_flag(true)
        .with_include_active_vdd_h_flag(true)
        .with_include_active_vdd_l_flag(true)
        .with_include_device_addr_value(true)
        .build();
    assert_eq!(data_out.into_bits(), 0b01111101_00000000);
    adc.set_dataout_ctl(data_out).unwrap();
    done_ender.done();
    pin_done.done();
}

#[test]
fn change_range_sel() {
    let expectations = [
        Transaction::transaction_start(),
        Transaction::write_vec(
            0b1101010_0_00010100_0000000000001011u32
                .to_be_bytes()
                .into(),
        ),
        Transaction::transaction_end(),
        Transaction::transaction_start(),
        Transaction::read_vec(vec![0, 0, 0, 0]),
        Transaction::transaction_end(),
    ];
    let spi = MockSpi::new(&expectations);
    let mut done_ender = spi.clone();
    let pin = MockPin::new(&[
        PTransaction::get(State::Low),
        PTransaction::get(State::High),
    ]);
    let mut pin_done = pin.clone();
    let mut adc: Ads8681<ads8681::Ads8681SpiInterface<_, MockPin>> =
        Ads8681::new_blocking(spi, pin);
    let range_sel = RangeSelBuilder::new()
        .with_disable_internal_adc_reference(false)
        .with_range_sel(ads8681::AdcInputRanges::OnePointTwoFive)
        .build();
    adc.set_range_sel(range_sel).unwrap();
    done_ender.done();
    pin_done.done();
}
