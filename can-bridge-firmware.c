#include "can-bridge-firmware.h"

#define EXTRAGIDS	65	//65 used for 30kWh bruteforce upgrade

//globals
volatile	uint8_t		charging_state				= 0;
volatile	uint8_t		voltage_samples				= 0;
volatile	uint8_t		LBC_CapacityBars			= 0;
volatile	uint16_t	voltage_sum					= 0;
volatile	uint16_t	filtered_voltage			= 345; //Initialized to 345, otherwise it will be 0 for a few seconds before enough filtering has happened
volatile	uint16_t	LBC_MainGids				= 0;
volatile	uint16_t	LBC_StateOfCharge			= 0;
volatile	uint16_t	LBC_BatteryVoltageSignal	= 0;


//example valid CAN frame
volatile	can_frame_t	static_message = {.can_id = 0x5BC, .can_dlc = 8, .data = {0,0,0,0,0,0,0,0}};

//USB variables
volatile	uint8_t		configSuccess		= false;	//tracks whether device successfully enumerated
static		FILE		USBSerialStream;				//fwrite target for CDC
volatile	uint8_t		signature[11];					//signature bytes

//variables for ProcessCDCCommand()
volatile	int16_t		cmd, cmd2;
volatile	uint16_t	i = 0, j = 0, k = 0;
volatile	uint32_t	temp;
volatile	uint16_t	ReportStringLength;
			char *		ReportString;	
				
volatile	uint8_t		can_busy			= 0;		//tracks whether the can_handler() subroutine is running
volatile	uint8_t		print_char_limit	= 0;		//serial output buffer size

//timer variables
volatile	uint8_t		ten_sec_timer		= 1;		//increments on every sec_timer underflow
volatile	uint16_t	sec_timer			= 1;		//actually the same as ms_timer but counts down from 1000
volatile	uint8_t		sec_interrupt		= 0;		//signals main loop to output debug data every second
volatile	uint16_t	ms_timer			= 1;		//increments on every TCC0 overflow (ever ms)

//because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
can_frame_t tx0_buffer[TXBUFFER_SIZE];
uint8_t		tx0_buffer_pos		= 0;
uint8_t		tx0_buffer_end		= 0;

can_frame_t tx2_buffer[TXBUFFER_SIZE];
uint8_t		tx2_buffer_pos		= 0;
uint8_t		tx2_buffer_end		= 0;

can_frame_t tx3_buffer[5];
uint8_t		tx3_buffer_pos		= 0;
uint8_t		tx3_buffer_end		= 0;

void hw_init(void){
	uint8_t caninit;

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, 48000000);		
	
	//turn off everything we don' t use
	PR.PRGEN		= PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
	PR.PRPA			= PR_ADC_bm | PR_AC_bm;
	PR.PRPC			= PR_TWI_bm | PR_USART0_bm | PR_HIRES_bm;
	PR.PRPD			= PR_TWI_bm | PR_USART0_bm | PR_TC0_bm | PR_TC1_bm;
	PR.PRPE			= PR_TWI_bm | PR_USART0_bm;
	
	//blink output
	PORTB.DIRSET	= 3;
	
	//start 16MHz crystal and PLL it up to 48MHz
	OSC.XOSCCTRL	= OSC_FRQRANGE_12TO16_gc |		//16MHz crystal
	OSC_XOSCSEL_XTAL_16KCLK_gc;						//16kclk startup
	OSC.CTRL	   |= OSC_XOSCEN_bm;				//enable crystal
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));			//wait until ready
	OSC.PLLCTRL		= OSC_PLLSRC_XOSC_gc | 2;		//XTAL->PLL, 2x multiplier (32MHz)
	OSC.CTRL	   |= OSC_PLLEN_bm;					//start PLL
	while (!(OSC.STATUS & OSC_PLLRDY_bm));			//wait until ready
	CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
	CLK.CTRL		= CLK_SCLKSEL_PLL_gc;			//use PLL output as system clock	
	
	//output 16MHz clock to MCP25625 chips (PE0)
	//next iteration: put this on some other port, pin 4 or 7, so we can use the event system
	TCE0.CTRLA		= TC0_CLKSEL_DIV1_gc;						//clkper/1
	TCE0.CTRLB		= TC0_CCAEN_bm | TC0_WGMODE_SINGLESLOPE_bm;	//enable CCA, single-slope PWM
	TCE0.CCA		= 1;										//compare value
	TCE0.PER		= 1;										//period of 1, generates 24MHz output
	
	PORTE.DIRSET	= PIN0_bm;									//set CLKOUT pin to output
	
	//setup CAN pin interrupts
	PORTC.INTCTRL	= PORT_INT0LVL_HI_gc;
	PORTD.INTCTRL	= PORT_INT0LVL_HI_gc | PORT_INT1LVL_HI_gc;	
	
	PORTD.INT0MASK	= PIN0_bm;						//PORTD0 has can1 interrupt
	PORTD.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	PORTD.INT1MASK	= PIN5_bm;						//PORTD5 has can2 interrupt
	PORTD.PIN5CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	#ifndef DISABLE_CAN3
	PORTC.INT0MASK	= PIN2_bm;						//PORTC2 has can3 interrupt
	PORTC.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	#endif
	
	//buffer checking interrupt
	TCC1.CTRLA		= TC0_CLKSEL_DIV1_gc;			//48M/1/4800 ~ 100usec
	TCC1.PER		= 4800;
	TCC1.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//same priority as can interrupts
	
	//we want to optimize performance, so we're going to time stuff
	//48MHz/48=1us timer, which we just freerun and reset whenever we want to start timing something
	//frame time timer
	TCC0.CTRLA		= TC0_CLKSEL_DIV1_gc;
	TCC0.PER		= 48000;						//48MHz/48000=1ms
	TCC0.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//interrupt on overflow
	
	PORTB.OUTCLR	= (1 << 0);
	
	can_system_init:
			
	//Init SPI and CAN interface:
	if(RST.STATUS & RST_WDRF_bm){ //if we come from a watchdog reset, we don't need to setup CAN
		caninit = can_init(MCP_OPMOD_NORMAL, 1); //on second thought, we do
	} else {
		caninit = can_init(MCP_OPMOD_NORMAL, 1);
	}
	
	if(caninit){		
		PORTB.OUTSET |= (1 << 0);					//green LED
	} else {		
		PORTB.OUTSET |= (1 << 1);					//red LED
		_delay_ms(10);
		goto can_system_init;
	}
	
	//Set and enable interrupts with round-robin
	XMEGACLK_CCP_Write((void * ) &PMIC.CTRL, PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);//PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm| PMIC_HILVLEN_bm;
	
	USB_Init(USB_OPT_RC32MCLKSRC | USB_OPT_BUSEVENT_PRILOW);
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	
	wdt_enable(WDTO_15MS);
	
	sei();
}


int main(void){
	char * str = "";
	hw_init();

    while(1){
		if(!output_can_to_serial){
			if(sec_interrupt){
				sec_interrupt = 0;
				
				//sample text output every second
				str = "ms 0000\n";
				int_to_4digit_nodec(ms_timer, (char *) (str + 3));
				print(str,8);
			}
		}
	}
}

/* services commands received over the virtual serial port */
void ProcessCDCCommand(void)
{
	ReportStringLength = 0;
	cmd = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	
	if(cmd > -1){
		switch(cmd){	
			case 48: //0
				break;
			case 0: //reset when sending 0x00
			case 90: //'Z' - also reset when typing a printable character (fallback for serial terminals that do not support sending non-printable characters)
				_delay_ms(1000);
				CCP				= CCP_IOREG_gc;			//allow changing CLK.CTRL
				RST.CTRL		= RST_SWRST_bm;			//perform software reset
				break;
			case 64: //@ - dump all CAN messages to USB
				output_can_to_serial = 1 - output_can_to_serial;
				break;
			case 255: //send ident
				ReportString = "MUXSAN CAN bridge\n"; ReportStringLength = 18;
				break;
					
			default: //when all else fails
				ReportString = "Unrecognized Command:   \n"; ReportStringLength = 25;
				ReportString[22] = cmd;
				break;
		}
		if(ReportStringLength){
			print(ReportString, ReportStringLength);
		}
	}
}

// Event handler for the LUFA library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void){}

void EVENT_USB_Device_Connect(void){}

// Event handler for the LUFA library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void){ configSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface); }

// Event handler for the LUFA library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(void){	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface); }

//appends string to ring buffer and initiates transmission
void print(char * str, uint8_t len){
	if((print_char_limit + len) <= 120){
		fwrite(str, len, 1, &USBSerialStream);
		print_char_limit += len;
	} else { //if the buffer is full, show that by sending an X (happens on very busy CAN buses)
		fwrite("X\n",2,1,&USBSerialStream);
	}
}

//fires every 1ms
ISR(TCC0_OVF_vect){	
	wdt_reset();
	ms_timer++;
	if(!can_busy) ProcessCDCCommand();
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
	
	//handle print buffer
	if(print_char_limit <= 64) { print_char_limit = 0; }
	else { print_char_limit -= 64; }	
	
	sec_timer--;	
	
	//fires every second
	if(sec_timer == 0){
		PORTB.OUTCLR = (1 << 1);
		sec_timer = 1000;
		sec_interrupt = 1;
		ten_sec_timer--;
		
		//fires every 10 seconds
		if(ten_sec_timer == 0){
			ten_sec_timer = 10;
		}
	}
}

//fires approx. every 100us
ISR(TCC1_OVF_vect){
	check_can1();
	check_can2();
	check_can3();
}

//can1 interrupt
ISR(PORTD_INT0_vect){
	can_busy = 1;
	can_handler(1);
}

//can2 interrupt
ISR(PORTD_INT1_vect){
	can_busy = 1;
	can_handler(2);
}

//can3 receive interrupt
ISR(PORTC_INT0_vect){
	can_busy = 1;
	can_handler(3);
}

//VCM side of the CAN bus (in Muxsan)
void can_handler(uint8_t can_bus){	
	can_frame_t frame;
	
	uint8_t mux_5bc_CapCharge = 0;
	uint8_t	corrected_chargebars = 0;
	uint16_t total_gids = 0;
	uint16_t total_voltage = 0;
		
	char strbuf[] = "1|   |                \n";
	if(can_bus == 2){ strbuf[0] = 50; }
	if(can_bus == 3){ strbuf[0] = 51; }
		
	uint8_t flag = can_read(MCP_REG_CANINTF, can_bus);
		
	if (flag & (MCP_RX0IF | MCP_RX1IF)){		
	
		if(flag & MCP_RX0IF){
			can_read_rx_buf(MCP_RX_0, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX0IF, 0x00, can_bus);
		} else {
			can_read_rx_buf(MCP_RX_1, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX1IF, 0x00, can_bus);
		}		
		
		switch(frame.can_id){
			case 0x1DB:
				//Check what voltage the LBC reports  (10 bits, all 8bits from frame[2], and 2 bits from frame[3] 0xC0)
				total_voltage = (frame.data[2] << 2) | ((frame.data[3] & 0xC0) >> 6);
				LBC_BatteryVoltageSignal = (total_voltage/2); //Convert LBC battery voltage to human readable voltage (1 LSB = 0.5V)
			break;
			case 0x55B:
				//Check what SOC the LBC reports  (10 bits, all 8bits from frame[0], and 2 bits from frame[1] 0xC0)
				LBC_StateOfCharge = (frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6);  //Further improvement idea, modify this also!
				break;
			case 0x5BC: //This frame contains: GIDS, temp readings, capacity bars and mux
				mux_5bc_CapCharge = (frame.data[4] & 0x0F); //Save the mux containing info if we have capacity or chargebars in the message [8 / 9] Only valid for 2014+!
				//Filter battery voltage measurement
				if (voltage_samples < 20) //Keep summing up voltage averages
				{
					voltage_sum += LBC_BatteryVoltageSignal;
					voltage_samples++;
				}
				else
				{
					filtered_voltage = (voltage_sum/20); //When we have enough samples, write an average (this helps during low SOC)
					voltage_samples = 0;
					voltage_sum = 0;
				}

				if(charging_state != 0x20) //Don't manipulate anything if car is slowcharging. Causes a TDC from TCU otherwise for 2013+ leafs
				{
					LBC_MainGids = (frame.data[0] << 2) | ((frame.data[1] & 0xC0) >> 6); //check how many the stock GIDs the BMS reports (10 bits, all 8bits from frame[0], and 2 bits from frame[1] 0xC0)

					if(LBC_MainGids>=10)
					{
						total_gids = (LBC_MainGids + EXTRAGIDS); //This algorithm has a drawback, it wont age as the battery does!
					
						if (total_gids >= 320)
						{
							corrected_chargebars = 14; //12
						}
						else if (total_gids >= 295)
						{
							corrected_chargebars = 13; //11
						}
						else if (total_gids >= 270)
						{
							corrected_chargebars = 12; //10
						}
						else if (total_gids >= 245)
						{
							corrected_chargebars = 11; //9
						}
						else if (total_gids >= 220)
						{
							corrected_chargebars = 10; //8
						}
						else if (total_gids >= 195)
						{
							corrected_chargebars = 9; //7
						}
						else if (total_gids >= 170)
						{
							corrected_chargebars = 8; //6
						}
						else if (total_gids >= 145)
						{
							corrected_chargebars = 7; //5
						}
						else if (total_gids >= 120)
						{
							corrected_chargebars = 6; //4
						}
						else if (total_gids >= 95)
						{
							corrected_chargebars = 5; //3
						}
						else if (total_gids >= 70)
						{
							corrected_chargebars = 4; //3
						}
						else
						{
							corrected_chargebars = 3;
						}
					}
					else if(LBC_MainGids<=9) //There is still some juice left in the battery when it reports 0gids. (ca 30% on a 30kWh bruteforce)
					{						 //Start estimating SOC from battery voltage
						if(filtered_voltage >= 347) //7GIDS = 347V on 24kWh LBC (345V is roughly 27% on 30kWh leaf)
						{
							total_gids = 60;
							corrected_chargebars = 3;
						}
						else if(filtered_voltage >= 346) //358V = 45% SOC on 30kWh Leaf //351V = 37% SOC on 30kWh Leaf //345V is roughly 27% on 30kWh leaf
						{
							total_gids = 57;
							corrected_chargebars = 3;
						}
						else if(filtered_voltage >= 345)
						{
							total_gids = 55;
							corrected_chargebars = 3;
						}
						else if(filtered_voltage >= 344)
						{
							total_gids = 52;
							corrected_chargebars = 3;
						}
						else if(filtered_voltage >= 343)
						{
							total_gids = 48; //at 48gids the stock BMS triggers LBW!
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 342)
						{
							total_gids = 45;
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 341)
						{
							total_gids = 42;
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 340)
						{
							total_gids = 40;
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 339)
						{
							total_gids = 38;
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 338)
						{
							total_gids = 36;
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 337) //337V = 21.4%<->18% SOC on 30kWh Leaf (with low mV diff!) (3.51V min)
						{
							total_gids = 34;
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 336)
						{
							total_gids = 32;
							corrected_chargebars = 2;
						}
						else if(filtered_voltage >= 335)
						{
							total_gids = 30;
							corrected_chargebars = 1;
						}
						else if(filtered_voltage >= 334)
						{
							total_gids = 25;
							corrected_chargebars = 1;
						}
						else if(filtered_voltage >= 333)
						{
							total_gids = 20;
							corrected_chargebars = 1;
						}
						else if(filtered_voltage >= 332)
						{
							total_gids = 15;
							corrected_chargebars = 1;
						}
						else if(filtered_voltage >= 331)
						{
							total_gids = 10;
							corrected_chargebars = 1;
						}
						else if(filtered_voltage >= 330)
						{
							total_gids = 5;
							corrected_chargebars = 0;
						}
						else if(filtered_voltage <= 329) //Contactors will open at ~300V, but it's not worth mapping this area. One single cell can trigger turtle quite easily here.
						{
							total_gids = LBC_MainGids;
							corrected_chargebars = 0;
						}
						else
						{
							total_gids = LBC_MainGids;
							corrected_chargebars = 0;
						}
					}
					//Add the total GIDs
					frame.data[0] = (uint8_t) (total_gids >> 2);
					frame.data[1] = (uint8_t) ((frame.data[1] & 0x3F) | (total_gids & 3) << 6);
				
					//Write capacity and charge bars (this is only valid for AZE0 leaf)
					if (mux_5bc_CapCharge == 9) //Only when the muxed field is equal to 9, capacity bars can be written
					{
						//frame.data[2] = (uint8_t) ((frame.data[2] & 0x0F) | spoofed_capacity << 4); //This will write 15/15, causing capacity to read full. Useful if you cannot reset the BMS
					}
					else if (mux_5bc_CapCharge == 8) //Only when the muxed field is equal to 8, charge bars can be written
					{
						frame.data[2] = (uint8_t) ((frame.data[2] & 0x0F) | corrected_chargebars << 4);
					}
				}
			break;
			case 0x1F2: //Collect the charging state (Needed for 2013+ Leafs, throws DTC otherwise) [VCM->LBC]
				charging_state = frame.data[2];
			break;
			default:
			break;
			}
		
		
		//if you enable CAN repeating between bus 1 and 2, we end up here	
		if(repeat_can){
			//you can blacklist certain messages or message contents like this, blocking them from both being forwarded and being displayed
			uint8_t blacklist = 0;
			switch(frame.can_id){				
				case 0x59E: 
					//blacklist = 1;
					break;
				default:
					blacklist = 0;					
					break;
			}
			if(!blacklist){
				if(can_bus == 1){send_can2(frame);} else {send_can1(frame);}
								
				if(output_can_to_serial){
					SID_to_str(strbuf + 2, frame.can_id);
					canframe_to_str(strbuf + 6, frame);
					print(strbuf,23);
				}
			}
		}		
	}
					
	
	if(flag & 0xA0){
		uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
		if(flag2 & 0xC0){
			can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors
			ReportString = "CANX RX OVF\n";
			ReportString[3] = 48 + can_bus;
			print(ReportString,12);
		}
		if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
		if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);	}
	}
	can_busy = 0;
}


void send_can(uint8_t can_bus, can_frame_t frame){
	if(can_bus == 1) send_can1(frame);
	if(can_bus == 2) send_can2(frame);
	if(can_bus == 3) send_can3(frame);
}

void send_can1(can_frame_t frame){	
	//put in the buffer
	memcpy(&tx0_buffer[tx0_buffer_end++], &frame, sizeof(frame));
	
	if(tx0_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx0_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can1();
}



void check_can1(void){
	uint8_t reg;
	
	if(tx0_buffer_end != tx0_buffer_pos){
		//check if TXB0 is free use
		reg = can1_read(MCP_REG_TXB0CTRL);
	
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can1_load_txbuff(0, (can_frame_t *) &tx0_buffer[tx0_buffer_pos++]);
			can1_rts(0);
			if(tx0_buffer_pos == tx0_buffer_end){ //end of buffer, reset
				tx0_buffer_end = 0;
				tx0_buffer_pos = 0;
			}
		}
	}
}

void send_can2(can_frame_t frame){
	//put in the buffer
	memcpy(&tx2_buffer[tx2_buffer_end++], &frame, sizeof(frame));
	
	if(tx2_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx2_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can2();
}

void check_can2(void){
	uint8_t reg;
	
	if(tx2_buffer_end != tx2_buffer_pos){
		//check if TXB0 is free use
		reg = can2_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can2_load_txbuff(0, (can_frame_t *) &tx2_buffer[tx2_buffer_pos++]);
			can2_rts(0);
			if(tx2_buffer_pos == tx2_buffer_end){ //end of buffer, reset
				tx2_buffer_end = 0;
				tx2_buffer_pos = 0;
			}
		}
	}
}

void send_can3(can_frame_t frame){
	//put in the buffer
	memcpy(&tx3_buffer[tx3_buffer_end++], &frame, sizeof(frame));
	
	if(tx3_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx3_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can3();
}

void check_can3(void){
	uint8_t reg;
	
	if(tx3_buffer_end != tx3_buffer_pos){
		//check if TXB0 is free use
		reg = can3_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can3_load_txbuff(0, (can_frame_t *) &tx3_buffer[tx3_buffer_pos++]);
			can3_rts(0);
			if(tx3_buffer_pos == tx3_buffer_end){ //end of buffer, reset
				tx3_buffer_end = 0;
				tx3_buffer_pos = 0;
			}
		}
	}
}

