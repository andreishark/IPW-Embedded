use embassy_rp::{
    pwm::{ChannelAPin, ChannelBPin, Config as ConfigPwm, Pwm, Slice},
    Peripheral,
};

pub struct RgbLed<'a> {
    pwm_red_green: Pwm<'a>,
    pwm_blue: Pwm<'a>,
    config_red_green: ConfigPwm,
    config_blue: ConfigPwm,
}

impl<'a> RgbLed<'a> {
    pub fn new<T: Slice, U: Slice>(
        top: u16,
        red_pin: impl Peripheral<P: ChannelAPin<T>> + 'a,
        green_pin: impl Peripheral<P: ChannelBPin<T>> + 'a,
        blue_pin: impl Peripheral<P: ChannelAPin<U>> + 'a,
        red_green_slice: impl Peripheral<P = T> + 'a,
        blue_slice: impl Peripheral<P = U> + 'a,
    ) -> RgbLed<'a> {
        let mut config_red_blue: ConfigPwm = Default::default();
        let mut config_blue: ConfigPwm = Default::default();

        config_red_blue.top = top;
        config_blue.top = top;

        config_red_blue.compare_a = top;
        config_red_blue.compare_b = top;
        config_blue.compare_a = top;

        return RgbLed {
            pwm_red_green: Pwm::new_output_ab(
                red_green_slice,
                red_pin,
                green_pin,
                config_red_blue.clone(),
            ),
            pwm_blue: Pwm::new_output_a(blue_slice, blue_pin, config_blue.clone()),
            config_red_green: config_red_blue,
            config_blue,
        };
    }

    fn _power_to_pwm_value(&self, power: u8) -> u16 {
        (self.config_red_green.top as f32 * ((100 - power.clamp(0, 100)) as f32 / 100.0)) as u16
    }

    pub fn set_red(&mut self, power: u8) {
        self.config_red_green.compare_a = self._power_to_pwm_value(power);
        self.pwm_red_green.set_config(&self.config_red_green);
    }

    pub fn set_green(&mut self, power: u8) {
        self.config_red_green.compare_b = self._power_to_pwm_value(power);
        self.pwm_red_green.set_config(&self.config_red_green);
    }

    pub fn set_blue(&mut self, power: u8) {
        self.config_blue.compare_a = self._power_to_pwm_value(power);
        self.pwm_blue.set_config(&self.config_blue);
    }

    pub fn set_color(&mut self, power_red: u8, power_green: u8, power_blue: u8) {
        self.config_red_green.compare_a = self._power_to_pwm_value(power_red);
        self.config_red_green.compare_b = self._power_to_pwm_value(power_green);
        self.config_blue.compare_a = self._power_to_pwm_value(power_blue);

        self.pwm_red_green.set_config(&self.config_red_green);
        self.pwm_blue.set_config(&self.config_blue);
    }

    fn map_rgb_to_power(value: u8) -> u8 {
        (255.0 / 100.0 * (value as f32)) as u8
    }

    pub fn lerp(color1: (u8, u8, u8), color2: (u8, u8, u8)) -> (u8, u8, u8) {
        (
            (color1.0 as f32 + (color2.0 as f32 - color1.0 as f32) * 0.5) as u8,
            (color1.1 as f32 + (color2.1 as f32 - color1.1 as f32) * 0.5) as u8,
            (color1.2 as f32 + (color2.2 as f32 - color1.2 as f32) * 0.5) as u8,
        )
    }

    pub fn set_rgb(&mut self, red: u8, green: u8, blue: u8) {
        let red_power = RgbLed::map_rgb_to_power(red);
        let green_power = RgbLed::map_rgb_to_power(green);
        let blue_power = RgbLed::map_rgb_to_power(blue);

        self.set_color(red_power, green_power, blue_power);
    }
}
