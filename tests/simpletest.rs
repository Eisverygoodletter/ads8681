use ads8681::{Ads8681SpiInterface, synchronous::Ads8681};
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
    let mut adc = Ads8681::new_with_interface(Ads8681SpiInterface(spi));
    let output_data_word = adc.get_data_output().unwrap();
    assert_eq!(output_data_word.get_conversion_result(), 0x1234);
    done_ender.done();
}
