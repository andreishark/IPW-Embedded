use embassy_rp::{
    i2c::{
        Async, Config as I2cConfig, Error as I2cError, I2c, Instance,
        InterruptHandler as I2CInterruptHandler, Mode, SclPin, SdaPin,
    },
    interrupt::typelevel::Binding,
    Peripheral,
};
use embassy_time::Timer;

pub struct DriverAT24C256<'a, T: embassy_rp::i2c::Instance> {
    i2a_interface: I2c<'a, T, Async>,
}

impl<'a, T: Instance> DriverAT24C256<'a, T> {
    pub fn new(
        pin_i2c: impl Peripheral<P = T> + 'a,
        pin_scl: impl Peripheral<P: SclPin<T>> + 'a,
        pin_sda: impl Peripheral<P: SdaPin<T>> + 'a,
        irqs: impl Binding<T::Interrupt, I2CInterruptHandler<T>>,
    ) -> DriverAT24C256<'a, T> {
        DriverAT24C256 {
            i2a_interface: I2c::new_async(pin_i2c, pin_scl, pin_sda, irqs, I2cConfig::default()),
        }
    }

    pub async fn scan_for_addresses(&mut self) -> [Option<u16>; 128] {
        let mut func_result: [Option<u16>; 128] = [None; 128];

        let mut rx_buf: [u8; 1] = [0x00_u8; 1];
        for i in 0_u16..127 {
            let result = self.i2a_interface.read_async(i, &mut rx_buf).await;
            let Ok(_) = result else { continue };
            func_result[i as usize] = Some(i);
        }

        func_result
    }
}

impl<'a, T: Instance> DriverAT24C256<'a, T> {
    pub async fn write_byte(&mut self, address: u16, byte: u8) -> Result<(), I2cError> {
        (self.i2a_interface.write_async(address, [byte]).await)?;

        Ok(())
    }

    pub async fn read_byte(&mut self, address: u16) -> Result<u8, I2cError> {
        let mut buffer: [u8; 1] = [0x0_u8; 1];
        self.i2a_interface.read_async(address, &mut buffer).await?;

        Ok(buffer[0])
    }

    pub async fn write_bytes(
        &mut self,
        address: u16,
        bytes: impl IntoIterator<Item = u8>,
    ) -> Result<(), I2cError> {
        self.i2a_interface.write_async(address, bytes).await?;

        Ok(())
    }

    pub async fn read_bytes<const COUNT: usize>(
        &mut self,
        address: u16,
    ) -> Result<[u8; COUNT], I2cError> {
        let mut buffer: [u8; COUNT] = [0x0_u8; COUNT];
        self.i2a_interface.read_async(address, &mut buffer).await?;

        Ok(buffer)
    }

    pub async fn read_bytes_with_offset<const COUNT: usize>(
        &mut self,
        address: u16,
        offset: u16,
    ) -> Result<[u8; COUNT], I2cError> {
        let mut buffer: [u8; COUNT] = [0x0_u8; COUNT];
        Timer::after_millis(50).await;
        self.i2a_interface
            .write_read_async(address, offset.to_be_bytes(), &mut buffer)
            .await?;

        Ok(buffer)
    }

    pub fn colors_to_bytes<const COUNT: usize>(
        colors: &[(u8, u8, u8)],
    ) -> ([u8; COUNT], [u8; COUNT], [u8; COUNT]) {
        let mut buffer: [u8; COUNT] = [0x0_u8; COUNT];
        let mut buffer1: [u8; COUNT] = [0x0_u8; COUNT];
        let mut buffer2: [u8; COUNT] = [0x0_u8; COUNT];

        for (idx, color) in colors.iter().enumerate() {
            buffer[idx] = color.0;
            buffer1[idx] = color.1;
            buffer2[idx] = color.2;
        }

        (buffer, buffer1, buffer2)
    }

    pub async fn write_colors<const COUNT: usize>(
        &mut self,
        address: u16,
        colors: [(u8, u8, u8); COUNT],
    ) -> Result<(), I2cError> {
        let (buffer_1, buffer_2, buffer_3) = DriverAT24C256::<T>::colors_to_bytes::<COUNT>(&colors);

        let buffer = buffer_1
            .iter()
            .chain(buffer_2.iter())
            .chain(buffer_3.iter());

        self.write_bytes(address, buffer.cloned()).await?;

        Ok(())
    }

    fn bytes_to_colors<const COUNT: usize>(
        red_array: [u8; COUNT],
        green_array: [u8; COUNT],
        blue_array: [u8; COUNT],
    ) -> [(u8, u8, u8); COUNT] {
        let mut colors: [(u8, u8, u8); COUNT] = [(0, 0, 0); COUNT];

        for (idx, color) in colors.iter_mut().enumerate() {
            color.0 = red_array[idx];
            color.1 = green_array[idx];
            color.2 = blue_array[idx];
        }
        colors
    }

    pub async fn read_colors<const COUNT: usize>(
        &mut self,
        address: u16,
    ) -> Result<[(u8, u8, u8); COUNT], I2cError> {
        let red_buffer = self.read_bytes_with_offset::<COUNT>(address, 0).await?;
        let green_buffer = self
            .read_bytes_with_offset::<COUNT>(address, COUNT as u16)
            .await?;
        let blue_buffer = self
            .read_bytes_with_offset::<COUNT>(address, COUNT as u16 * 2)
            .await?;
        // let buffer: [u8; COUNT] = self.read_bytes(address).await?;

        let colors_array =
            DriverAT24C256::<T>::bytes_to_colors(red_buffer, green_buffer, blue_buffer);

        Ok(colors_array)
    }
}
