/*
 * Admatec C-Berry LCD for Raspberry Pi Model B
 *
 * Copyright (C) 2014 Ulrich Völkel
 *
 * based on:
 *    bcm2835 library           Copyright (C) 2011-2013 Mike McCauley   GPLv2
 *    C-Berry example code      Copyright (C) 2013 admatec GmbH         GPLv3
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>


// 320 x 240 16bpp (65K color)
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  240
#define DISPLAY_BPP     16


// RAIO register -> see datasheet RAIO8870
#define PCOD 0x00
#define PWRR 0x01
#define MRWC 0x02
#define PCLK 0x04

#define SYSR 0x10
#define DRGB 0x11
#define IOCR 0x12
#define IODR 0x13

#define HDWR  0x14
#define HNDFTR 0x15
#define HNDR  0x16
#define HSTR  0x17
#define HPWR  0x18

#define VDHR0 0x19
#define VDHR1 0x1a
#define VNDR0 0x1b
#define VNDR1 0x1c
#define VSTR0 0x1d
#define VSTR1 0x1e
#define VPWR  0x1f

#define DPCR  0x20
#define FNCR0 0x21
#define FNCR1 0x22
#define CGSR  0x23
#define HOFS0 0x24
#define HOFS1 0x25
#define VOFS0 0x26
#define VOFS1 0x27
#define ROMS  0x28

#define FLDR  0x29

#define HSAW0 0x30
#define HSAW1 0x31
#define VSAW0 0x32
#define VSAW1 0x33
#define HEAW0 0x34
#define HEAW1 0x35
#define VEAW0 0x36
#define VEAW1 0x37
#define HSSW0 0x38
#define HSSW1 0x39
#define VSSW0 0x3a
#define VSSW1 0x3b
#define HESW0 0x3c
#define HESW1 0x3d
#define VESW0 0x3e
#define VESW1 0x3f

#define MWCR0 0x40
#define MWCR1 0x41
#define TFCR  0x42
#define TBCR  0x43
#define BTCR  0x44
#define CURS  0x45
#define CURH0 0x46
#define CURH1 0x47
#define CURV0 0x48
#define CURV1 0x49
#define RCURH0 0x4a
#define RCURH01 0x4b
#define RCURV0 0x4c
#define RCURV1 0x4d
#define MRCD  0x4e
#define BECR0 0x50
#define BECR1 0x51
#define LTPR0 0x52
#define LTPR1 0x53
#define HSBE0 0x54
#define HSBE1 0x55
#define VSBE0 0x56
#define VSBE1 0x57
#define HDBE0 0x58
#define HDBE1 0x59
#define VDBE0 0x5a
#define VDBE1 0x5b
#define BEWR0 0x5c
#define BEWR1 0x5d
#define BEHR0 0x5e
#define BEHR1 0x5f

#define BGCR0 0x60
#define BGCR1 0x61
#define BGCR2 0x62
#define FGCR0 0x63
#define FGCR1 0x64
#define FGCR2 0x65
#define PTNO  0x66
#define BGTR  0x67

#define TPCR0 0x70
#define TPCR1 0x71
#define TPXH  0x72
#define TPYH  0x73
#define TPXYL 0x74

#define GCHP0 0x80
#define GCHP1 0x81
#define GCVP0 0x82
#define GCVP1 0x83
#define GCC0  0x84
#define GCC1  0x85

#define PLLC1 0x88
#define PLLC2 0x89

#define P1CR  0x8a
#define P1DCR 0x8b
#define P2CR  0x8c
#define P2DCR 0x8d
#define MCLR  0x8e
#define INTC  0x8f

#define DCR   0x90
#define DLHSR0 0x91
#define DLHSR1 0x92
#define DLVSR0 0x93
#define DLVSR1 0x94
#define DLHER0 0x95
#define DLHER1 0x96
#define DLVER0 0x97
#define DLVER1 0x98
#define DCHR0  0x99
#define DCHR1 0x9a
#define DCVR0 0x9b
#define DCVR1 0x9c
#define DCRR  0x9d

#define TCR1 0xa0
#define TCR2 0xa1
#define OEHTCR1 0xa2
#define OEHTCR2 0xa3
#define OEHTCR3 0xa4
#define OEHTCR4 0xa5
#define OEHTCR5 0xa6
#define OEHTCR6 0xa7
#define OEHTCR7 0xa8
#define OEHTCR8 0xa9

#define STHTCR1 0xaa
#define STHTCR2 0xab
#define STHTCR3 0xac
#define STHTCR4 0xad

#define Q1HCR1 0xae
#define Q1HCR2 0xaf

#define OEVTCR1 0xb0
#define OEVTCR2 0xb1
#define OEVTCR3 0xb2
#define OEVTCR4 0xb3
#define CKVTCR1 0xb4
#define CKVTCR2 0xb5
#define CKVTCR3 0xb6
#define CKVTCR4 0xb7
#define STVTCR1 0xb8
#define STVTCR2 0xb9
#define STVTCR3 0xba
#define STVTCR4 0xbb
#define STVTCR5 0xbc
#define STVTCR6 0xbd
#define STVTCR7 0xbe
#define STVTCR8 0xbf

#define COMTCR1 0xc0
#define COMTCR2 0xc1
#define RGBTCR1 0xc2
#define RGBTCR2 0xc3


// colors "RRRGGGBB"
#define COLOR_RED       0xE0
#define COLOR_BLUE      0x03
#define COLOR_GREEN     0x1C
#define COLOR_BLACK     0x00
#define COLOR_WHITE     0xFF
#define COLOR_CYAN      0x1F
#define COLOR_YELLOW    0xFC
#define COLOR_MAGENTA   0xE3
#define COLOR_DARK_GREEN 0x0C




#define BCM2835_GPIO_FSEL_INPT  0b000
#define BCM2835_GPIO_FSEL_OUTP  0b001
#define BCM2835_GPIO_FSEL_ALT0  0b100
#define BCM2835_GPIO_FSEL_MASK  0b111

#define BCM2835_GPIO_PUD_OFF    0b00    ///< Off ? disable pull-up/down
#define BCM2835_GPIO_PUD_DOWN   0b01    ///< Enable Pull Down control
#define BCM2835_GPIO_PUD_UP     0b10    ///< Enable Pull Up control


#define RPI_V2_GPIO_P1_11   17  ///< Version 2, Pin P1-11
#define RPI_V2_GPIO_P1_12   18  ///< Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5
#define RPI_V2_GPIO_P1_13   27  ///< Version 2, Pin P1-13
#define RPI_V2_GPIO_P1_15   22  ///< Version 2, Pin P1-15
#define RPI_V2_GPIO_P1_16   23  ///< Version 2, Pin P1-16
#define RPI_V2_GPIO_P1_18   24  ///< Version 2, Pin P1-18
#define RPI_V2_GPIO_P1_19   10  ///< Version 2, Pin P1-19, MOSI when SPI0 in use
#define RPI_V2_GPIO_P1_21    9  ///< Version 2, Pin P1-21, MISO when SPI0 in use
#define RPI_V2_GPIO_P1_22   25  ///< Version 2, Pin P1-22
#define RPI_V2_GPIO_P1_23   11  ///< Version 2, Pin P1-23, CLK when SPI0 in use
#define RPI_V2_GPIO_P1_24    8  ///< Version 2, Pin P1-24, CE0 when SPI0 in use
#define RPI_V2_GPIO_P1_26    7  ///< Version 2, Pin P1-26, CE1 when SPI0 in use


#define MOSI      RPI_V2_GPIO_P1_19
#define MISO      RPI_V2_GPIO_P1_21
#define SCLK      RPI_V2_GPIO_P1_23
#define OE        RPI_V2_GPIO_P1_11
#define SPI_CE1   RPI_V2_GPIO_P1_26
#define RAIO_RS   RPI_V2_GPIO_P1_12
#define RAIO_RST  RPI_V2_GPIO_P1_22
#define RAIO_CS   RPI_V2_GPIO_P1_24
#define RAIO_WR   RPI_V2_GPIO_P1_18
#define RAIO_RD   RPI_V2_GPIO_P1_16
#define RAIO_WAIT RPI_V2_GPIO_P1_15
#define RAIO_INT  RPI_V2_GPIO_P1_13



#define BCM2835_GPFSEL0                      0x0000 ///< GPIO Function Select 0
#define BCM2835_GPFSEL1                      0x0004 ///< GPIO Function Select 1
#define BCM2835_GPFSEL2                      0x0008 ///< GPIO Function Select 2
#define BCM2835_GPFSEL3                      0x000c ///< GPIO Function Select 3
#define BCM2835_GPFSEL4                      0x0010 ///< GPIO Function Select 4
#define BCM2835_GPFSEL5                      0x0014 ///< GPIO Function Select 5
#define BCM2835_GPSET0                       0x001c ///< GPIO Pin Output Set 0
#define BCM2835_GPSET1                       0x0020 ///< GPIO Pin Output Set 1
#define BCM2835_GPCLR0                       0x0028 ///< GPIO Pin Output Clear 0
#define BCM2835_GPCLR1                       0x002c ///< GPIO Pin Output Clear 1
#define BCM2835_GPLEV0                       0x0034 ///< GPIO Pin Level 0
#define BCM2835_GPLEV1                       0x0038 ///< GPIO Pin Level 1
#define BCM2835_GPEDS0                       0x0040 ///< GPIO Pin Event Detect Status 0
#define BCM2835_GPEDS1                       0x0044 ///< GPIO Pin Event Detect Status 1
#define BCM2835_GPREN0                       0x004c ///< GPIO Pin Rising Edge Detect Enable 0
#define BCM2835_GPREN1                       0x0050 ///< GPIO Pin Rising Edge Detect Enable 1
#define BCM2835_GPFEN0                       0x0058 ///< GPIO Pin Falling Edge Detect Enable 0
#define BCM2835_GPFEN1                       0x005c ///< GPIO Pin Falling Edge Detect Enable 1
#define BCM2835_GPHEN0                       0x0064 ///< GPIO Pin High Detect Enable 0
#define BCM2835_GPHEN1                       0x0068 ///< GPIO Pin High Detect Enable 1
#define BCM2835_GPLEN0                       0x0070 ///< GPIO Pin Low Detect Enable 0
#define BCM2835_GPLEN1                       0x0074 ///< GPIO Pin Low Detect Enable 1
#define BCM2835_GPAREN0                      0x007c ///< GPIO Pin Async. Rising Edge Detect 0
#define BCM2835_GPAREN1                      0x0080 ///< GPIO Pin Async. Rising Edge Detect 1
#define BCM2835_GPAFEN0                      0x0088 ///< GPIO Pin Async. Falling Edge Detect 0
#define BCM2835_GPAFEN1                      0x008c ///< GPIO Pin Async. Falling Edge Detect 1
#define BCM2835_GPPUD                        0x0094 ///< GPIO Pin Pull-up/down Enable
#define BCM2835_GPPUDCLK0                    0x0098 ///< GPIO Pin Pull-up/down Enable Clock 0
#define BCM2835_GPPUDCLK1                    0x009c ///< GPIO Pin Pull-up/down Enable Clock 1


#define BCM2835_SPI0_CS                      0x0000 ///< SPI Master Control and Status
#define BCM2835_SPI0_FIFO                    0x0004 ///< SPI Master TX and RX FIFOs
#define BCM2835_SPI0_CLK                     0x0008 ///< SPI Master Clock Divider


#define BCM2835_SPI0_CS_TXD                  0x00040000 ///< TXD TX FIFO can accept Data
#define BCM2835_SPI0_CS_RXD                  0x00020000 ///< RXD RX FIFO contains Data
#define BCM2835_SPI0_CS_DONE                 0x00010000 ///< Done transfer Done
#define BCM2835_SPI0_CS_TA                   0x00000080 ///< Transfer Active
#define BCM2835_SPI0_CS_CLEAR                0x00000030 ///< Clear FIFO Clear RX and TX
#define BCM2835_SPI0_CS_CLEAR_RX             0x00000020 ///< Clear FIFO Clear RX
#define BCM2835_SPI0_CS_CPOL                 0x00000008 ///< Clock Polarity
#define BCM2835_SPI0_CS_CPHA                 0x00000004 ///< Clock Phase
#define BCM2835_SPI0_CS_CS                   0x00000003 ///< Chip Select


#define BCM2835_SPI_MODE0               0
#define BCM2835_SPI_CS1                 1
#define BCM2835_SPI_CLOCK_DIVIDER_2     2


#define HIGH 0x1
#define LOW  0x0


struct cberryfb_par {
    u32 *gpio_base;
    u32 *spi0_base;
};


/*
 * all peri_, gpio_ and spi_ functions are based on 
 * the bcm2835 library by Mike McCauley
 */

// read from peripheral
static inline u32 peri_read(volatile u32* paddr)
{
    return readl(paddr);
}

static inline u32 peri_read_nb(volatile u32* paddr)
{
    return readl(paddr);
}

// write to peripheral
static inline void peri_write(volatile u32* paddr, u32 value)
{
    writel(value, paddr);
}

static inline void peri_write_nb(volatile u32* paddr, u32 value)
{
    writel(value, paddr);
}

// set/clear only the bits in value covered by the mask
static inline void peri_set_bits(volatile u32* paddr, u32 value, u32 mask)
{
    u32 v = peri_read(paddr);
    v = (v & ~mask) | (value & mask);
    peri_write(paddr, v);
}


// Function select
static void gpio_fsel(struct cberryfb_par *par, u8 pin, u8 mode)
{
    // Function selects are 10 pins per 32 bit word, 3 bits per pin
    volatile u32* paddr = par->gpio_base + BCM2835_GPFSEL0/4 + (pin/10);
    u8   shift = (pin % 10) * 3;
    u32  mask = BCM2835_GPIO_FSEL_MASK << shift;
    u32  value = mode << shift;
    peri_set_bits(paddr, value, mask);
}

// Set the state of an output
static void gpio_write(struct cberryfb_par *par, u8 pin, u8 on)
{
    volatile u32* paddr = par->gpio_base + (on ? BCM2835_GPSET0 : BCM2835_GPCLR0)/4 + pin/32;
    u8 shift = pin % 32;
    peri_write(paddr, 1 << shift);
}

// Read input pin
u8 gpio_lev(struct cberryfb_par *par, u8 pin)
{
    volatile u32* paddr = par->gpio_base + BCM2835_GPLEV0/4 + pin/32;
    u8  shift = pin % 32;
    u32 value = peri_read(paddr);
    return (value & (1 << shift)) ? HIGH : LOW;
}

// Set pullup/down
static void gpio_pud(struct cberryfb_par *par, u8 pud)
{
    volatile u32* paddr = par->gpio_base + BCM2835_GPPUD/4;
    peri_write(paddr, pud);
}

// Pullup/down clock
// Clocks the value of pud into the GPIO pin
static void gpio_pudclk(struct cberryfb_par *par, u8 pin, u8 on)
{
    volatile u32* paddr = par->gpio_base + BCM2835_GPPUDCLK0/4 + pin/32;
    u8 shift = pin % 32;
    peri_write(paddr, (on ? 1 : 0) << shift);
}

// Set the pullup/down resistor for a pin
static void gpio_set_pud(struct cberryfb_par *par, u8 pin, u8 pud)
{
    gpio_pud(par, pud);
    udelay(10);
    gpio_pudclk(par, pin, 1);
    udelay(10);
    gpio_pud(par, BCM2835_GPIO_PUD_OFF);
    gpio_pudclk(par, pin, 0);
}



static void spi_set_clock_divider(struct cberryfb_par *par, u16 divider)
{
    volatile u32* paddr = par->spi0_base + BCM2835_SPI0_CLK/4;
    peri_write(paddr, divider);
}

static void spi_set_data_mode(struct cberryfb_par *par, u8 mode)
{
    volatile u32* paddr = par->spi0_base + BCM2835_SPI0_CS/4;
    // Mask in the CPO and CPHA bits of CS
    peri_set_bits(paddr, mode << 2, BCM2835_SPI0_CS_CPOL | BCM2835_SPI0_CS_CPHA);
}

static void spi_chip_select(struct cberryfb_par *par, u8 cs)
{
    volatile u32* paddr = par->spi0_base + BCM2835_SPI0_CS/4;
    // Mask in the CS bits of CS
    peri_set_bits(paddr, cs, BCM2835_SPI0_CS_CS);
}

static void spi_set_chip_select_polarity(struct cberryfb_par *par, u8 cs, u8 active)
{
    volatile u32* paddr = par->spi0_base + BCM2835_SPI0_CS/4;
    u8 shift = 21 + cs;
    // Mask in the appropriate CSPOLn bit
    peri_set_bits(paddr, active << shift, 1 << shift);
}

// Writes an number of bytes to SPI
static void spi_writenb(struct cberryfb_par *par, char* tbuf, u32 len)
{
    volatile u32* paddr = par->spi0_base + BCM2835_SPI0_CS/4;
    volatile u32* fifo = par->spi0_base + BCM2835_SPI0_FIFO/4;
    u32 i;

    // This is Polled transfer as per section 10.6.1
    // BUG ALERT: what happens if we get interupted in this section, and someone else
    // accesses a different peripheral?

    // Clear TX and RX fifos
    peri_set_bits(paddr, BCM2835_SPI0_CS_CLEAR, BCM2835_SPI0_CS_CLEAR);

    // Set TA = 1
    peri_set_bits(paddr, BCM2835_SPI0_CS_TA, BCM2835_SPI0_CS_TA);

    for (i = 0; i < len; i++)
    {
        // Maybe wait for TXD
        while (!(peri_read(paddr) & BCM2835_SPI0_CS_TXD))
            ;

        // Write to FIFO, no barrier
        peri_write_nb(fifo, tbuf[i]);

        // Read from FIFO to prevent stalling
        while (peri_read(paddr) & BCM2835_SPI0_CS_RXD)
            (void) peri_read_nb(fifo);
        }

        // Wait for DONE to be set
        while (!(peri_read_nb(paddr) & BCM2835_SPI0_CS_DONE)) {
        while (peri_read(paddr) & BCM2835_SPI0_CS_RXD)
            (void) peri_read_nb(fifo);
    };

    // Set TA = 0, and also set the barrier
    peri_set_bits(paddr, 0, BCM2835_SPI0_CS_TA);
}





/*
 * all tft_ and raio_ functions are based on 
 * the C-Berry example code from admatec
 */

// initialization of GPIO and SPI
static void tft_init_board(struct fb_info *info)
{
    struct cberryfb_par *par = info->par;
    volatile u32* paddr = par->spi0_base + BCM2835_SPI0_CS/8;

    // set the pins to be an output and turn them on
    gpio_fsel(par, OE, BCM2835_GPIO_FSEL_OUTP);
    gpio_write(par, OE, HIGH);

    gpio_fsel(par, RAIO_RST, BCM2835_GPIO_FSEL_OUTP);
    gpio_write(par, RAIO_RST, HIGH);

    gpio_fsel(par, RAIO_CS, BCM2835_GPIO_FSEL_OUTP);
    gpio_write(par, RAIO_CS, HIGH);

    gpio_fsel(par, RAIO_RS, BCM2835_GPIO_FSEL_OUTP);
    gpio_write(par, RAIO_RS, HIGH);

    gpio_fsel(par, RAIO_WR, BCM2835_GPIO_FSEL_OUTP);
    gpio_write(par, RAIO_WR, HIGH);

    gpio_fsel(par, RAIO_RD, BCM2835_GPIO_FSEL_OUTP);
    gpio_write(par, RAIO_RD, HIGH);


    // now the inputs
    gpio_fsel(par, RAIO_WAIT, BCM2835_GPIO_FSEL_INPT);
    gpio_set_pud(par, RAIO_WAIT, BCM2835_GPIO_PUD_UP);

    gpio_fsel(par, RAIO_INT, BCM2835_GPIO_FSEL_INPT);
    gpio_set_pud(par, RAIO_INT, BCM2835_GPIO_PUD_UP);


    // set pins for SPI
    gpio_fsel(par, MISO, BCM2835_GPIO_FSEL_ALT0);
    gpio_fsel(par, MOSI, BCM2835_GPIO_FSEL_ALT0);
    gpio_fsel(par, SCLK, BCM2835_GPIO_FSEL_ALT0);
    gpio_fsel(par, SPI_CE1, BCM2835_GPIO_FSEL_ALT0);

    // set the SPI CS register to the some sensible defaults
    peri_write(paddr, 0);

    // clear TX and RX fifos
    peri_write_nb( paddr, BCM2835_SPI0_CS_CLEAR );


    // MSBFIRST is the only byteorder suported by SPI0
    spi_set_data_mode(par, BCM2835_SPI_MODE0);
    spi_set_clock_divider(par, BCM2835_SPI_CLOCK_DIVIDER_2);
    spi_chip_select(par, BCM2835_SPI_CS1);
    spi_set_chip_select_polarity(par, BCM2835_SPI_CS1, LOW);
}

// hard reset of the graphic controller and the tft
static void tft_hard_reset(struct fb_info *info)
{
    struct cberryfb_par *par = info->par;

    gpio_write(par, RAIO_RST, LOW);
    msleep(10);
    gpio_write(par, RAIO_RST, HIGH);
    msleep(1);
}

// wait during raio is busy
static void tft_wait_for_raio(struct cberryfb_par *par)
{
    while (!gpio_lev(par, RAIO_WAIT));
}

// write data via SPI to tft
static void tft_spi_data_out(struct cberryfb_par *par, u16 data)
{
    char buffer[2];
    buffer[0] = (u8)(data >> 8);
    buffer[1] = (u8)(data & 0xFF);
    spi_writenb(par, &buffer[0], 2);
}

// write byte to register
static void tft_register_write(struct cberryfb_par *par, u16 reg)
{
    gpio_write(par, RAIO_RS, HIGH);
    gpio_write(par, RAIO_CS, LOW);
    gpio_write(par, RAIO_WR, LOW);
    gpio_write(par, OE, LOW);

    tft_spi_data_out(par, reg);

    gpio_write(par, RAIO_WR, HIGH);
    gpio_write(par, RAIO_CS, HIGH);
    gpio_write(par, OE, HIGH);
}

// write byte to tft
static void tft_data_write(struct cberryfb_par *par, u16 data)
{
    gpio_write(par, RAIO_RS, LOW);
    gpio_write(par, RAIO_CS, LOW);
    gpio_write(par, RAIO_WR, LOW);
    gpio_write(par, OE, LOW);

    tft_spi_data_out(par, data);

    gpio_write(par, RAIO_WR, HIGH);
    gpio_write(par, RAIO_CS, HIGH);
    gpio_write(par, OE, HIGH);
}

// write 'count'-bytes to tft
static void tft_data_multiwrite(struct cberryfb_par *par,  u16 *data, u32 count )
{
    volatile u32* paddr = par->spi0_base + BCM2835_SPI0_CS/4;
    volatile u32* fifo  = par->spi0_base + BCM2835_SPI0_FIFO/4;

    volatile u32* gpio_set   = par->gpio_base + BCM2835_GPSET0/4;
    volatile u32* gpio_clear = par->gpio_base + BCM2835_GPCLR0/4;

    u32 i;

    gpio_write(par, RAIO_RS, LOW);
    gpio_write(par, RAIO_CS, LOW);
    gpio_write(par, OE, LOW);

    for( i=0; i<count; i++ )
    {
        // WR = 0
        *gpio_clear = ( 1 << RAIO_WR );

        // activate SPI transfer
        *paddr |= BCM2835_SPI0_CS_TA;

        // fill the FIFO
        *fifo = (u8)(data[i] >> 8);
        *fifo = (u8)(data[i] & 0xFF);

        // write fifo data to SPI TX buffer
        while (!(*paddr & BCM2835_SPI0_CS_DONE)) {
            // clear SPI RX buffer
            *paddr |=BCM2835_SPI0_CS_CLEAR_RX;
        };

        // deactivate SPI transfer
        *paddr &= ~BCM2835_SPI0_CS_TA;

        // WR = 1
        *gpio_set = ( 1 << RAIO_WR );
    }

    gpio_write(par, RAIO_CS, HIGH);
    gpio_write(par, OE, HIGH);
}





// write command to a register
static void raio_set_register(struct cberryfb_par *par, u8 reg, u8 value)
{
    tft_register_write(par, (u16)reg);
    tft_data_write(par, (u16)value);
}

// set PWM value for backlight
void raio_set_backlight_pwm_value(struct cberryfb_par *par, u8 value)
{
     raio_set_register(par, P1CR, 0x88);    // Enable PWM1 output divider 256
     raio_set_register(par, P1DCR, value);  // vaue = 0 (0% PWM) - 255 (100% PWM)
}

// set coordinates for active window
static void raio_set_active_window(struct cberryfb_par *par,  u16 XL, u16 XR , u16 YT, u16 YB)
{
    u8 buffer[2];

    // setting active window X
    buffer[0] = (u8)(XL >> 8);
    buffer[1] = (u8)(XL & 0xFF);
    raio_set_register(par, HSAW0, buffer[1]);
    raio_set_register(par, HSAW1, buffer[0]);

    buffer[0] = (u8)(XR >> 8);
    buffer[1] = (u8)(XR & 0xFF);
    raio_set_register(par, HEAW0, buffer[1]);
    raio_set_register(par, HEAW1, buffer[0]);

    //setting active window Y
    buffer[0] = (u8)(YT >> 8);
    buffer[1] = (u8)(YT & 0xFF);
    raio_set_register(par, VSAW0, buffer[1]);
    raio_set_register(par, VSAW1, buffer[0]);

    buffer[0] = (u8)(YB >> 8);
    buffer[1] = (u8)(YB & 0xFF);
    raio_set_register(par, VEAW0, buffer[1]);
    raio_set_register(par, VEAW1, buffer[0]);
}

// initialization of RAIO8870
static void raio_init(struct fb_info *info)
{
    struct cberryfb_par *par = info->par;

    raio_set_register(par, PLLC1, 0x07);    // set sys_clk
    udelay(200);
    raio_set_register(par, PLLC2, 0x03);    // set sys_clk
    udelay(200);

    raio_set_register(par, PWRR, 0x01);     // Raio software reset ( bit 0 ) set
    raio_set_register(par, PWRR, 0x00);     // Raio software reset ( bit 0 ) set to 0
    msleep(100);

    // color modes (color depths)
    raio_set_register(par, SYSR, 0x0A);
    raio_set_register(par, DPCR, 0x00);


    // horizontal settings
    raio_set_register(par, HDWR , (DISPLAY_WIDTH / 8) - 1);
    raio_set_register(par, HNDFTR, 0x02);       // Horizontal Non-Display Period Fine Tuning
    raio_set_register(par, HNDR, 0x03);         // HNDR , Horizontal Non-Display Period Bit[4:0]
    raio_set_register(par, HSTR, 0x04);         // HSTR , HSYNC Start Position[4:0], HSYNC Start Position(PCLK) = (HSTR + 1)*8     0x02
    raio_set_register(par, HPWR, 0x03);         // HPWR , HSYNC Polarity ,The period width of HSYNC.

    // vertical settings
    raio_set_register(par, VDHR0, ((DISPLAY_HEIGHT-1) & 0xFF));
    raio_set_register(par, VDHR1, ((DISPLAY_HEIGHT-1) >> 8));

    // VNDR0 , Vertical Non-Display Period Bit [7:0]
    // Vertical Non-Display area = (VNDR + 1)
    // VNDR1 , Vertical Non-Display Period Bit [8]
    // Vertical Non-Display area = (VNDR + 1)
    raio_set_register(par, VNDR0, 0x10);
    raio_set_register(par, VNDR1, 0x00);

    // VPWR , VSYNC Polarity ,VSYNC Pulse Width[6:0]
    // VSYNC , Pulse Width(PCLK) = (VPWR + 1)
    raio_set_register(par, VPWR, 0x00);


    // miscellaneous settings

    // active Window
    raio_set_active_window(par, 0, DISPLAY_WIDTH - 1, 0, DISPLAY_HEIGHT - 1);

    // PCLK fetch data on rising edge
    raio_set_register(par, PCLK, 0x00);

    // Backlight dimming
    raio_set_backlight_pwm_value(par, 50);

    // memory clear with background color
    raio_set_register(par, TBCR, COLOR_BLACK);
    raio_set_register(par, MCLR, 0x81);
    tft_wait_for_raio(par);

    raio_set_register(par, IODR, 0x07);
    raio_set_register(par, PWRR, 0x80);
}

// write memory to TFT
static void raio_write_vmem(struct fb_info *info)
{
    struct cberryfb_par *par = info->par;
    tft_register_write(par, MRWC);
    tft_data_multiwrite(par, (u16 __force*)info->screen_base, info->fix.smem_len / 2);
}





static void cberryfb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
    sys_fillrect(info, rect);
    raio_write_vmem(info);
}

static void cberryfb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
    sys_copyarea(info, area);
    raio_write_vmem(info);
}

static void cberryfb_imageblit(struct fb_info *info, const struct fb_image *image)
{
    sys_imageblit(info, image);
    raio_write_vmem(info);
}

static ssize_t cberryfb_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t ret = fb_sys_write(info, buf, count, ppos);
    raio_write_vmem(info);
    return ret;
}

static void cberryfb_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
    raio_write_vmem(info);
}



static struct fb_fix_screeninfo cberryfb_fix = {
    .id             = "cberryfb",
    .type           = FB_TYPE_PACKED_PIXELS,
    .visual         = FB_VISUAL_TRUECOLOR,
    .accel          = FB_ACCEL_NONE,
    .xpanstep       = 0,
    .ypanstep       = 0,
    .ywrapstep      = 0,
    .line_length    = DISPLAY_WIDTH * DISPLAY_BPP / 8,
};

static struct fb_var_screeninfo cberryfb_var = {
    .width          = DISPLAY_WIDTH,
    .height         = DISPLAY_HEIGHT,
    .bits_per_pixel = DISPLAY_BPP,
    .xres           = DISPLAY_WIDTH,
    .yres           = DISPLAY_HEIGHT,
    .xres_virtual   = DISPLAY_WIDTH,
    .yres_virtual   = DISPLAY_HEIGHT,
    .nonstd         = 1,
    .activate       = FB_ACTIVATE_NOW,
    .vmode          = FB_VMODE_NONINTERLACED,
};

static struct fb_ops cberryfb_ops = {
    .owner          = THIS_MODULE,
    .fb_read        = fb_sys_read,
    .fb_write       = cberryfb_write,
    .fb_fillrect    = cberryfb_fillrect,
    .fb_copyarea    = cberryfb_copyarea,
    .fb_imageblit   = cberryfb_imageblit,
};

static struct fb_deferred_io cberryfb_defio = {
    .delay          = HZ/20,
    .deferred_io    = cberryfb_deferred_io,
};

static int cberryfb_probe(struct platform_device *dev)
{
    struct fb_info *info;
    struct cberryfb_par *par;
    int retval = -ENOMEM;
    int vmem_size;
    unsigned char *vmem;


    vmem_size = cberryfb_var.width * cberryfb_var.height * cberryfb_var.bits_per_pixel/8;
    vmem = vzalloc(vmem_size);
    if (!vmem) {
        return -ENOMEM;
    }
    memset(vmem, 0, vmem_size);


    info = framebuffer_alloc(sizeof(struct cberryfb_par), &dev->dev);
    if (!info) {
        vfree(vmem);
        return -ENOMEM;
    }


    info->screen_base = (char __force __iomem*)vmem;
    info->fbops = &cberryfb_ops;
    info->fix = cberryfb_fix;
    info->fix.smem_len = vmem_size;
    info->var = cberryfb_var;
    info->pseudo_palette = (void *)(info + 1);
    info->flags = FBINFO_DEFAULT | FBINFO_VIRTFB;

    info->fbdefio = &cberryfb_defio;
    fb_deferred_io_init(info);


    retval = register_framebuffer(info);
    if (retval < 0) {
        framebuffer_release(info);
        vfree(vmem);
        return retval;
    }

    platform_set_drvdata(dev, info);

    par = info->par;
    par->gpio_base = ioremap(GPIO_BASE, SZ_16K);
    par->spi0_base = ioremap(SPI0_BASE, SZ_16K);

    tft_init_board(info);
    tft_hard_reset(info);
    raio_init(info);

    printk(KERN_INFO "fb%d: admatec C-Berry LCD frame buffer device\n", info->node);
    return 0;
}



static int cberryfb_remove(struct platform_device *dev)
{
    struct fb_info *info = platform_get_drvdata(dev);
    struct cberryfb_par* par;

    if (info) {
        fb_deferred_io_cleanup(info);
        unregister_framebuffer(info);
        vfree((void __force *)info->screen_base);

        par = info->par;
        if (par) {
            iounmap(par->gpio_base);
            iounmap(par->spi0_base);
        }
        
        framebuffer_release(info);
    }
    return 0;
}


static struct platform_driver cberryfb_driver = {
    .probe  = cberryfb_probe,
    .remove = cberryfb_remove,
    .driver = {
        .name   = "cberryfb",
    },
};

static struct platform_device *cberryfb_device;

static int __init cberryfb_init(void)
{
    int ret = platform_driver_register(&cberryfb_driver);
    if (0 == ret) {
        cberryfb_device = platform_device_alloc("cberryfb", 0);
        if (cberryfb_device) {
            ret = platform_device_add(cberryfb_device);
        } else {
            ret = -ENOMEM;
        }
        if (0 != ret) {
            platform_device_put(cberryfb_device);
            platform_driver_unregister(&cberryfb_driver);
        }
    }
    return ret;
}

static void __exit cberryfb_exit(void)
{
    platform_device_unregister(cberryfb_device);
    platform_driver_unregister(&cberryfb_driver);
}

module_init(cberryfb_init);
module_exit(cberryfb_exit);

MODULE_DESCRIPTION("admatec C-Berry LCD frame buffer driver");
MODULE_AUTHOR("Ulrich Völkel");
MODULE_LICENSE("GPL");
