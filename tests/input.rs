use ads8681::registers;

#[test]
fn clear_hword() {
    // let dummy_address = NineBitAddress::form_full_command(, comm
    assert_eq!(
        [0b11000000, 0b0100, 0, 0u8],
        registers::RST_PWRCTL_REG.form_full_command(ads8681::CommandBits::ClearHword, 0)
    );
}
