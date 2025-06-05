CH9431 SPI CAN Driver
===========================
This driver can only work with SPI CAN function in these WCH devices:
CH9431

Integrated into your system method1
---------------------------------------
If you are using dts device tree to set up spi and driver, you can read this method, otherwise
please refer to method2.

1. Please copy the driver file to the package directory which be used to add additional drivers.

2. Please add the relevant Makefile and Kconfig like other drivers, generally you can copy one
from other driver then modify it.

3. Run the make menuconfig and select the ch9431 can support at "modules" item.

4. Define the spi structure on your dts file similar the follow: 
	spidev@1 {
		#address-cells = <1>;
		#size-cells = <1>;
		compatible = "wch,ch9431";
		reg = <1 0>;
		spi-max-frequency = <2000000>;
		interrupt-parent = <&gpio0>;
		interrupts = <12 IRQ_TYPE_LEVEL_LOW>;
	}
	Notice that the irq request method cannot be supported in this way in some platforms.
	You can contact us for other methods.

Integrated into your system method2
---------------------------------------
1. Please copy the driver file to the kernel directory:$kernel_src/drivers/net/ethernet

2. Please add the followed txt into the kernel file:$kernel_src/drivers/net/ethernet/Kconfig
config CH9431
	tristate "CH9431 support"
	depends on SPI
	select SERIAL_CORE
	help
	  This selects support for ch9431 ethernet.
	
3. Add the follow define into the $kernel_src/drivers/net/ethernet/Makefile for compile the driver.
obj-$(CONFIG_SERIAL_CH9431) += ch9431.o

4. Run the make menuconfig and select the ch9431 ethernet support at the driver/net/ethernet and save the config.

5. Define the spi0_board_info object on your board file similar the follow:
static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "wch,ch9431",
		.platform_data = NULL,
		.max_speed_hz = 100 * 1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
		.irq = IRQ_EINT(25),
	}
};

**Note**

Any question, you can send feedback to mail: tech@wch.cn
