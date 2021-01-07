/*
 * Definitions for Jetson tegra186-jetson-tx2.
 *
 * Definitions based on dts extracted from /boot/tegra186-quill-p3310-1000-c03-00-base-hdr40.dtbo
 *
 */

#define JETSON_COMPATIBLE "nvidia,p2597-0000+p3310-1000","nvidia,tegra186"

/* SoC function name for I2S interface on 40-pin header pins 12, 35, 38 and 40 */
#define HDR40_I2S	"i2s1"

/* SoC pin name definitions for 40-pin header */
#define HDR40_PIN3	"gpio_sen9_pee1" 
#define HDR40_PIN5	"gpio_sen8_pee0" 
#define HDR40_PIN7	"aud_mclk_pj4" 
#define HDR40_PIN8	"uart1_tx_pt0"
#define HDR40_PIN10	"uart1_rx_pr3"
#define HDR40_PIN11	"uart1_rx_pt1"
#define HDR40_PIN12	"dap1_sclk_pj0" 
#define HDR40_PIN13	"gpio_aud0_pj5"
#define HDR40_PIN16	"can_gpio0_paa0"
#define HDR40_PIN18	"gpio_mdm2_py1"
#define HDR40_PIN19	"gpio_cam6_pn5" 
#define HDR40_PIN21	"gpio_cam5_pn4"
#define HDR40_PIN23	"gpio_cam4_pn3"
#define HDR40_PIN24	"gpio_cam7_pn6"
#define HDR40_PIN27	"gen1_i2c_sda_pc6"
#define HDR40_PIN28	"gen1_i2c_scl_pc5"
#define HDR40_PIN29	"gpio_aud1_pj6"
#define HDR40_PIN31	"can_gpio2_paa2"
#define HDR40_PIN32	"can_gpio1_paa1"
#define HDR40_PIN33	"gpio_pq5_pi5"
#define HDR40_PIN35	"dap1_fs_pj3"
#define HDR40_PIN36	"uart1_cts_pt3"
#define HDR40_PIN37	"gpio_pq4_pi4"
#define HDR40_PIN38	"dap1_din_pj2" 
#define HDR40_PIN40	"dap1_dout_pj1" 

