#ifndef EFM32HG
#define EFM32HG // taken care of by the makefile, but this makes the 
#endif			// vscode linting work properly.

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/i2c.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define AUTORUN
#ifdef AUTORUN
#include <toboot.h>
TOBOOT_CONFIGURATION(TOBOOT_CONFIG_FLAG_AUTORUN);
#endif

#pragma region time set
// #define TIME_SET
#ifdef TIME_SET
#define SECONDS 0
#define MINUTES 8
#define HOURS   7
#define DAY_OF_WEEK 6
#define DAY     21
#define MONTH   12
#define YEAR    18
#endif
#pragma endregion

#pragma region general config
/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY 24000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7
#define LED_BLUE_PORT  GPIOE
#define LED_BLUE_PIN   GPIO13
#define BUTTON_PORT    GPIOE
#define BUTTON_PIN     GPIO12

#pragma endregion

#pragma region USB keyboard

#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0101    /* Program version */

// // Declare injectkeys function
// void injkeys(char *source);

bool g_usbd_is_connected = false;
bool once=true;
usbd_device *g_usbd_dev = 0;

static const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
        0x09, 0x06, // USAGE (Keyboard)
        0xa1, 0x01, // COLLECTION (Application)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
        0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x01, // LOGICAL_MAXIMUM (1)
        0x75, 0x01, // REPORT_SIZE (1)
        0x95, 0x08, // REPORT_COUNT (8)
        0x81, 0x02, // INPUT (Data,Var,Abs) //1 byte

        0x95, 0x01, // REPORT_COUNT (1)
        0x75, 0x08, // REPORT_SIZE (8)
        0x81, 0x03, // INPUT (Cnst,Var,Abs) //1 byte

        0x95, 0x06, // REPORT_COUNT (6)
        0x75, 0x08, // REPORT_SIZE (8)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x65, // LOGICAL_MAXIMUM (101)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
        0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
        0x81, 0x00, // INPUT (Data,Ary,Abs) //6 bytes

        0xc0, // END_COLLECTION
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 8,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 1, // 1=keyboard, 2=mouse
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static const char *usb_strings[] = {
	"ClocKey",
	"Keyboard Clock",
	"CUSTOM_001",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];


static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

    /* Dirty way to know if we're connected */
    g_usbd_is_connected = true;

	return 1;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
}

void usb_isr(void)
{
    usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
    while(1);
}

// HID Usage Tables
// https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf

#define NO_MOD      0
#define SHIFT       2
#define NO_KEY      0
const int keyDuration=150;
const int keyDelay=140000;

void injkeys(char *source)
{
	static uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // key pressed
	static uint8_t buf2[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Key released
	int i;
    uint8_t mod;
    uint8_t key;
	if(g_usbd_is_connected) {
        int lstr=strlen(source);
        // change ascii to keyboard map
        for (int j = 0; j < lstr; j++){
            mod=NO_MOD;
            key=NO_KEY;
            if(source[j]>='1'&&source[j]<='9')
                key=source[j]-19;
            else if (source[j]=='0')
                key=39;
            else if(source[j]>='a'&&source[j]<='z')
                key=source[j]-'a'+4;
            else if(source[j]>='A'&&source[j]<='Z'){
                mod=SHIFT;
                key=source[j]-'A'+4;
            }
            else if (source[j]=='\n')
                key=40;
            else if (source[j]==' ')
                key=44;
            else if(source[j]=='.')
                key=55;
            else if(source[j]==',')
                key=54;
            else if(source[j]==':'){
                mod=SHIFT;
                key=51;
            }
            else if(source[j]==';')
                key=51;
            else if(source[j]=='-')
                key=45;
            else if(source[j]=='_'){
                mod=SHIFT;
                key=45;
            }
            else
                key=NO_KEY;
            buf[0]=mod;
            buf[2]=key;
            usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8);
            for(i = 0; i != keyDuration; ++i) __asm__("nop");
            usbd_ep_write_packet(g_usbd_dev, 0x81, buf2, 8);
            for(i = 0; i != keyDelay; ++i) __asm__("nop"); // Wait a little
            if(j<lstr-1){
                if(source[j]==source[j+1]){
                    buf[0]=NO_MOD;
                    buf[2]=NO_KEY;
                    usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8);
                    for(i = 0; i != keyDuration; ++i) __asm__("nop");
                    usbd_ep_write_packet(g_usbd_dev, 0x81, buf2, 8);
                    for(i = 0; i != keyDelay; ++i) __asm__("nop"); // Wait a little
                }
            }

        }
		usbd_ep_write_packet(g_usbd_dev, 0x81, buf2, 8); // Be sure key is released
        for(i = 0; i != keyDelay; ++i) __asm__("nop"); // Wait a little
    }
 }

#pragma endregion

#pragma region debug

// #define DEBUG_PRINT
void debug_print_str(char *source) {
    #ifdef DEBUG_PRINT
    injkeys(source);
    #endif
}

void debug_print_reg(uint32_t reg) {
    #ifdef DEBUG_PRINT
    for (int i=0; i<32; i++){
        debug_print_str(((reg & 1<<i) != 0) ? "1" : "0");
    }
    debug_print_str("\n");
    #endif
}

char get_hex_digit(uint8_t nibble) {

    switch (nibble & 0x0F) {
        case 0x00:
            return '0';
        case 0x01:
            return '1';
        case 0x02:
            return '2';
        case 0x03:
            return '3';
        case 0x04:
            return '4';
        case 0x05:
            return '5';
        case 0x06:
            return '6';
        case 0x07:
            return '7';
        case 0x08:
            return '8';
        case 0x09:
            return '9';
        case 0x0A:
            return 'A';
        case 0x0B:
            return 'B';
        case 0x0C:
            return 'C';
        case 0x0D:
            return 'D';
        case 0x0E:
            return 'E';
        case 0x0F:
            return 'F';
        default:
            return 'X';
    }
}

void debug_print_byte(uint8_t byte) {
    #ifdef DEBUG_PRINT
    char string[] = {get_hex_digit(byte >> 4), get_hex_digit(byte)};
    debug_print_str(string);
    #endif
}

void debug_I2C_registers(){
    #ifdef DEBUG_PRINT
    debug_print_reg(I2C0_CLKDIV);
    debug_print_reg(I2C0_CMD);
    debug_print_reg(I2C0_CTRL);
    debug_print_reg(I2C0_IEN);
    debug_print_reg(I2C0_IF);
    debug_print_reg(I2C0_ROUTE);
    debug_print_reg(I2C0_RXDATAP);
    debug_print_reg(I2C0_STATE);
    debug_print_reg(I2C0_STATUS);
    #endif
}

#define ZEROB               1
#define ONEB                2
#define RED_LED             3
#define GREEN_LED           4
#define DEBUG_PIN
#define DEBUG_LABEL_1       ONEB
#define DEBUG_PORT_1        LED_BLUE_PORT
#define DEBUG_PIN_1         LED_BLUE_PIN
#define DEBUG_LABEL_2       ZEROB
#define DEBUG_PORT_2        BUTTON_PORT
#define DEBUG_PIN_2         BUTTON_PIN

void debug_to_pin(int label) {
    #ifdef DEBUG_PIN
    if (label == DEBUG_LABEL_1)
        gpio_toggle(DEBUG_PORT_1, DEBUG_PIN_1);
    // if (label == DEBUG_LABEL_2)
    //     gpio_toggle(DEBUG_PORT_2, DEBUG_PIN_2);
    if (label == RED_LED)
        gpio_toggle(LED_RED_PORT, LED_RED_PIN);
    if (label == GREEN_LED)
        gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
    #endif
}

#pragma endregion

#pragma region I2C

/* I2C buffer declarations */
#define write_buffer_size 24
static uint8_t write_extended_buffer[write_buffer_size];
volatile uint8_t write_length = 0;
volatile uint8_t write_index = 0;
#define read_buffer_size 24
static uint8_t read_extended_buffer[read_buffer_size];
volatile uint8_t read_length = 0;
volatile uint8_t read_index = 0;
typedef enum {
    IDLE,               // Idle
    WRITE,              // Write mode (just write entire write buffer)
    READ_W_ADDR_W,      // Read mode - sending address for write
    READ_W_REG_ADDR,    // Read mode - sending register address
    READ_W_ADDR_R,      // Read mode - sending address for read
    READ_W_FINISH,       // Read mode - waiting for read address to finish to queue reads
    READ_R_REGS         // Read mode - receiving (just read entire read buffer)
} I2C_state;
volatile I2C_state I2C_active_state = IDLE;

void I2C_wait_for_transaction_complete() {
    while ((I2C_active_state != IDLE) |
           ((I2C0_STATE & I2C_STATE_BUSHOLD) != 0)) __asm__("nop");
        //    ((I2C0_STATE & I2C_STATE_STATE_MASK) != I2C_STATE_STATE_IDLE)) __asm__("nop");
}

void config_I2C(){
    // debug_I2C_registers();

    // Set pins as outputs
    gpio_mode_setup(GPIOC, GPIO_MODE_WIRED_AND, GPIO0);
    gpio_mode_setup(GPIOC, GPIO_MODE_WIRED_AND, GPIO1);
    gpio_set(GPIOC, GPIO0);
    gpio_set(GPIOC, GPIO1);
    gpio_set_drive_strength(GPIOC, GPIO_STRENGTH_HIGH); // Set to high for debugging...
    
                
    // configure clock
	cmu_periph_clock_enable(CMU_I2C0);
    for(int i = 0; i != 1000000; ++i) __asm__("nop"); // wait for clock to stabilize
    I2C0_CLKDIV = 150; // What speed do I want? This should be ~10kHz, I think.

    // Master config
    I2C0_CTRL |= I2C_CTRL_EN | // Enable
                // I2C_CTRL_AUTOSE | // Automatic STOP when empty
                I2C_CTRL_CLHR_STANDARD | // Standard 4 high / 4 low timing ratio
                I2C_CTRL_AUTOSN | // Automatic STOP on NACK
                I2C_CTRL_AUTOACK; // Automatic ACK

    // Route I2C to pin pair 4: SDA/SCL: PC0/PC1: CAP0A/CAP1A
    I2C0_ROUTE = I2C_ROUTE_LOCATION_LOC4 << I2C_ROUTE_LOCATION_SHIFT | I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN;

    // Interrupt configuration
    I2C0_IEN = 0; // No interrupts enabled
    
    nvic_set_priority(NVIC_I2C0_IRQ, 0x10);
    I2C0_IFC = 0x0001FFFF;
    nvic_enable_irq(NVIC_I2C0_IRQ);
    // Reset, because the datasheet tells me to.
    I2C0_CMD |= I2C_CMD_ABORT;
    for(int i = 0; i != 1000000; ++i) __asm__("nop"); // wait for abort to complete?
    // debug_print_str("Conf\n");
    // debug_I2C_registers();
}

void I2C_write(uint8_t address, uint8_t reg_addr, uint8_t* data, uint8_t length){
    I2C0_CMD |= I2C_CMD_ABORT;
    I2C_wait_for_transaction_complete();

    // set operating mode
    I2C_active_state = WRITE;

    // Load data into software extended buffer
    write_length = length + 2 > write_buffer_size ? write_buffer_size : length + 2; // no buffer overflow!
    write_index = 0;
    write_extended_buffer[0] = address<<1;
    write_extended_buffer[1] = reg_addr;
    for(uint8_t i=2; i<write_length; i++)
    {
        write_extended_buffer[i] = data[i - 2];
    }

    // Enable I2C interrupt
    I2C0_IFC = 0x0001FFFF;

    I2C0_IEN = I2C_IEN_TXBL; // Enable interrupt when transmit buffer is empty
    // nvic_enable_irq(NVIC_I2C0_IRQ);

    // Start! Sends a start bit, then automatically start sending data.
    I2C0_CMD |= I2C_CMD_START;
}

void I2C_read(uint8_t address, uint8_t reg_addr, uint8_t length){
    I2C0_CMD |= I2C_CMD_ABORT;
    I2C_wait_for_transaction_complete();

    // set operating mode
    I2C_active_state = READ_W_ADDR_W;

    // Load data into software extended write buffer
    write_length = 3;
    write_index = 0;
    write_extended_buffer[0] = address<<1; // don't set read bit, first writing the desired register address
    write_extended_buffer[1] = reg_addr;
    write_extended_buffer[2] = address<<1 | 1; // Set read bit this time!

    // Clear software extended read buffer
    read_length = length > read_buffer_size ? read_buffer_size : length; // no buffer overflow!
    read_index = 0;
    for(uint8_t i=0; i<read_length; i++)
    {
        read_extended_buffer[i] = 0;
    }

    // clear read buffer
    uint8_t temp = I2C0_RXDATA;

    // Enable I2C interrupt
    I2C0_IFC = 0x0001FFFF;
    I2C0_IEN = I2C_IEN_TXBL; // Enabe interrupt when transmit buffer is empty
    // nvic_enable_irq(NVIC_I2C0_IRQ);

    // Start! Sends a start bit, then automatically starts communicating.
    I2C0_CMD |= I2C_CMD_START;
}

void I2C_close_connection(){
        I2C0_IEN = 0; // No interrupts enabled
        // nvic_disable_irq(NVIC_I2C0_IRQ);
        I2C_active_state = IDLE;
}

/* Returns a boolean indicating whether data was written. */
bool I2C_write_next_byte(){
    // Replace with DMA?
    bool data_available = write_index < write_length;
    if (data_available) {
        I2C0_TXDATA = ((int)write_extended_buffer[write_index++] & 0xFF);
    }
    return data_available;
}

bool I2C_read_next_byte(){
    // Replace with DMA?
    if (read_index < read_length) {
        read_extended_buffer[read_index++] = I2C0_RXDATA;
        if (read_index >= read_length) {
            // schedule nack for next read
            I2C0_CMD |= I2C_CMD_NACK; // Does this actually send at the right time?
        }
        return true;
    } else {
        // close down connection
        I2C0_CMD |= I2C_CMD_STOP; // Does this actually send at the right time?
        I2C_close_connection();
        return false;
    }
}

void i2c0_isr(){
    int interrupt_flags = I2C0_IF;
    uint8_t temp = 0;
    if ((interrupt_flags | I2C_IF_RXDATAV) != 0) {
        // Receive data is available
        if (I2C_active_state == READ_R_REGS) {
            // We sometimes get receive data when not in receive mode? Why?
            I2C_read_next_byte();
        }
    }
    if ((interrupt_flags | I2C_IF_TXBL) != 0)
        // Write register has been transfered to shift register
        switch (I2C_active_state) {
            case WRITE:
                if (!I2C_write_next_byte()){
                    I2C0_CMD = I2C_CMD_STOP;
                    I2C_close_connection();
                }
                break;
            case READ_W_ADDR_W:
                I2C_write_next_byte();
                I2C_active_state = READ_W_REG_ADDR;
                break;
            case READ_W_REG_ADDR:
                I2C_write_next_byte();
                I2C_active_state = READ_W_ADDR_R;
                break;
            case READ_W_ADDR_R:
                // Send a repeated start, then send the address to initiate read
                I2C0_CMD |= I2C_CMD_START;
                // Change the enabled interrupt flag from transmit buffer 
                // empty to receive data available.
                I2C_write_next_byte();
                I2C_active_state = READ_W_FINISH;
                break;
            case READ_W_FINISH:
                temp = I2C0_RXDATA;
                I2C0_IEN = I2C_IEN_RXDATAV;
                I2C_active_state = READ_R_REGS;
                break;
            case IDLE:
                // This should happen on startup, but we may also encounter it at other times.
                // I2C0_CMD = I2C_CMD_ABORT;
                // I2C_close_connection();
                break;
            case READ_R_REGS:
                // We are reading, no need to do anything here.
                break;
        }
}

#pragma endregion

#pragma region RTC

#ifdef TIME_SET
uint8_t int_to_bcd(uint8_t value) {
    return ((value / 10) << 4) | (value % 10 & 0x0F);
}

void set_time(){
    uint8_t registers[17] = {0};
    registers[0] = int_to_bcd(SECONDS) & 0x7F;
    registers[1] = int_to_bcd(MINUTES) & 0x7F;
    registers[2] = int_to_bcd(HOURS) & 0x3F; // Don't set bit 6. This sets 24 hour mode.
    registers[3] = int_to_bcd(DAY_OF_WEEK) & 0x07;
    registers[4] = int_to_bcd(DAY) & 0x3F;
    registers[5] = int_to_bcd(MONTH) & 0x1F;
    registers[6] = int_to_bcd(YEAR);
    registers[0xE] = 0x04; // selects interrupt on the int/1pps pin
    I2C_write(0b1101000, 0x00, registers, 17);
 }
#endif

char int_to_char(uint8_t value) {
    value = value & 0x0F;
    if (value == 0)
        return '0';
    if (value <= 9)
        return '1' - 1 + value;
    return 'Z';
}

#define SECONDS_R   read_extended_buffer[0]
#define MINUTES_R   read_extended_buffer[1]
#define HOURS_R     read_extended_buffer[2]
#define DAY_R         read_extended_buffer[4]
#define MONTH_R       read_extended_buffer[5]
#define YEAR_R        read_extended_buffer[6]
void get_time(){
    I2C_read(0b1101000, 0, 7);
    I2C_wait_for_transaction_complete();
    char time_string[] = "20YY-MM-DD HH:MM:SS\n";
    time_string[2] = int_to_char(YEAR_R >> 4);
    time_string[3] = int_to_char(YEAR_R);
    time_string[5] = int_to_char(MONTH_R >> 4);
    time_string[6] = int_to_char(MONTH_R);
    time_string[8] = int_to_char(DAY_R >> 4);
    time_string[9] = int_to_char(DAY_R);
    time_string[11] = int_to_char(HOURS_R >> 4);
    time_string[12] = int_to_char(HOURS_R);
    time_string[14] = int_to_char(MINUTES_R >> 4);
    time_string[15] = int_to_char(MINUTES_R);
    time_string[17] = int_to_char(SECONDS_R >> 4);
    time_string[18] = int_to_char(SECONDS_R);
    injkeys(time_string);
}

#pragma endregion

#pragma region sys_tick

#define SYSTICK
// Create flag to limit rate to once per second.
volatile bool sent_this_second = false;
#ifdef SYSTICK
/* Systick interrupt frequency, Hz NOT ACCURATE!?!*/
// Manually tuned the seconds counter to approximately give a 1 second tick
#define SYSTICK_FREQUENCY 1000
volatile int second_counter = 0;
void sys_tick_handler(void)
{
    if (second_counter++ >= 880) {
        second_counter = 0;
        debug_to_pin(RED_LED);
        debug_to_pin(GREEN_LED);
        debug_to_pin(ONEB);
        sent_this_second = false;
    }
}
#endif
#pragma endregion

#pragma region button



#pragma endregion

int main(void)
{
    /* Make sure the vector table is relocated correctly (after the Tomu bootloader) */
    SCB_VTOR = 0x4000;

    /* Disable the watchdog that the bootloader started. */
    WDOG_CTRL = 0;

    /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Configure the UI */
    // Set up both LEDs as outputs
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);
    // Set up button
    gpio_mode_setup(LED_BLUE_PORT, GPIO_MODE_WIRED_AND, LED_BLUE_PIN);
    gpio_mode_setup(BUTTON_PORT, GPIO_MODE_INPUT_PULL, BUTTON_PIN);
    
    
    
    // Turn LEDs off
    gpio_set(LED_RED_PORT, LED_RED_PIN);
    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_set(LED_BLUE_PORT, LED_BLUE_PIN);

    // set button pull resistor to pull-down
    gpio_clear(BUTTON_PORT, BUTTON_PIN);


    /* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, hid_set_config);

    /* Enable USB IRQs */
    nvic_set_priority(NVIC_USB_IRQ, 0x40);
	nvic_enable_irq(NVIC_USB_IRQ);

    #ifdef SYSTICK
    /* Configure the system tick, at lower priority than USB IRQ */
    systick_set_frequency(SYSTICK_FREQUENCY, AHB_FREQUENCY);
    systick_counter_enable();
    systick_interrupt_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0x20);
    #endif

    // Wait to allow USB to finish initializing
    for(int i = 0; i != 5000000; ++i) __asm__("nop");
    debug_print_str("0\n");
    /* Configure the I2C */
    config_I2C();

    #ifdef TIME_SET
    set_time();
    I2C_wait_for_transaction_complete();
    for(int i = 0; i != 100000; ++i) __asm__("nop");
    #endif

    bool queued = false;
    int debounce_counter = 0;

    while(1) {
        if (!sent_this_second && queued) {
            sent_this_second = true;
            debounce_counter = 0;
            queued = false;
            get_time();
        }
        if ((gpio_port_read(BUTTON_PORT) & BUTTON_PIN) != 0) {
            debounce_counter++;
            if (debounce_counter > 1000) {
                debounce_counter = 0;
                queued = true;
            }
        }        
    }
}

