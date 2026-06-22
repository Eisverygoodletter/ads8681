use ads8681::{Ads8681, DataOutCtlBuilder, synchronous::Ads8681Features};
use embedded_hal_mock::eh1::spi::{Mock, Transaction};
#[test]
fn noop() {
    let expectations = [
        Transaction::transaction_start(),
        Transaction::transfer_in_place(vec![0, 0, 0, 0], vec![0x12, 0x34, 0, 0]),
        Transaction::transaction_end(),
    ];
    let spi = Mock::new(&expectations);
    let mut done_ender = spi.clone();
    let mut adc = Ads8681::new_blocking(spi);
    let output_data_word = adc.get_data_output().unwrap();
    assert_eq!(output_data_word.get_conversion_result(), 0x1234);
    done_ender.done();
}
#[test]
fn write_data_out() {
    let expectations = [
        Transaction::transaction_start(),
        Transaction::transfer_in_place(
            vec![0b11010_000, 0x10, 0b01111101, 0b00000000],
            vec![0, 0, 0, 0],
        ),
        Transaction::transaction_end(),
    ];
    let spi = Mock::new(&expectations);
    let mut done_ender = spi.clone();
    let mut adc = Ads8681::new_blocking(spi);
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
}
