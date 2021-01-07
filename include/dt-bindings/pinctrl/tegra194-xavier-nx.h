/*
 * Definitions for Jetson tegra194-jetson-nx.
 *
 * Definitions based on dts extracted from /boot/tegra194-p3668-all-p3509-0000-hdr40.dtbo
 *
 */

#define JETSON_COMPATIBLE	"nvidia,p3449-0000+p3668-0000","nvidia,p3449-0000+p3668-0001","nvidia,p3509-0000+p3668-0000","nvidia,p3509-0000+p3668-0001","nvidia,tegra194"

/* SoC function name for I2S interface on 40-pin header pins 12, 35, 38 and 40 */
#define HDR40_I2S	"i2s5"

/* SoC pin name definitions for 40-pin header */
#define HDR40_PIN7	"aud_mclk_ps4" 
#define HDR40_PIN8	"uart1_tx_pr2"
#define HDR40_PIN10	"uart1_rx_pr3"
#define HDR40_PIN11	"uart1_rts_pr4"
#define HDR40_PIN12	"dap5_sclk_pt5" 
#define HDR40_PIN13	"spi3_sck_py0"
#define HDR40_PIN15	"touch_clk_pcc4"
#define HDR40_PIN16	"spi3_cs1_py4"
#define HDR40_PIN18	"spi3_cs0_py3"
#define HDR40_PIN19	"spi1_mosi_pz5" 
#define HDR40_PIN21	"spi1_miso_pz4"
#define HDR40_PIN22	"spi3_miso_py1"
#define HDR40_PIN23	"spi1_sck_pz3"
#define HDR40_PIN24	"spi1_cs0_pz6"
#define HDR40_PIN26	"spi1_cs1_pz7"
#define HDR40_PIN27	"gen2_i2c_sda_pdd0"
#define HDR40_PIN28	"gen2_i2c_scl_pcc7"
#define HDR40_PIN29	"soc_gpio41_pq5"
#define HDR40_PIN31	"soc_gpio42_pq6"
#define HDR40_PIN32	"soc_gpio44_pr0"
#define HDR40_PIN33	"soc_gpio54_pn1"
#define HDR40_PIN35	"dap5_fs_pu0"
#define HDR40_PIN36	"uart1_cts_pr5"
#define HDR40_PIN37	"spi3_mosi_py2"
#define HDR40_PIN38	"dap5_din_pt7" 
#define HDR40_PIN40	"dap5_dout_pt6" 

