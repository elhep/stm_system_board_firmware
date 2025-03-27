

pub struct Magneto{
    pub settings: Settings,
    bus: BusReference,
    dac: Ad5542,
    adc: Ad7682,
    io_expander: Mcp23s08,
}