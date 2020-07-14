#include <msp430.h>

#define BUTTON_0 BIT1		// On-board button P1.1
#define BUTTON_1 BIT2		// On-board button P1.2
#define STATUS_LED_0 BIT0	// On-board LED P1.0
#define STATUS_LED_1 BIT7	// On-board LED P9.7
#define TOTAL_NODES 0x20

// define message timing
#define START_REQUEST_DATA 0x0400		// Initial time to request data
#define TIME_REQUEST_DATA 0x0400		// delta time between data requests
#define START_SAMPLE 0x0060				// Initial time to sample analog input
#define TIME_SAMPLE 0x0200				// delta time between sampling analog input
#define TIME_NULL_TRANSMISSION 0x0400	// delta time between sending null transmission
											// in order to reset the child watchdog timer
#define MAPPING_DELAY 0x0800			// delta time between checks if the child node has finished mapping

// define opcodes
#define OP_bits 0xE0			// the bits used for transmitting opcodes b1110 0000
#define OP_NULL 0x00			// the null opcode
#define OP_REQUEST_DATA 0x20	// the opcode for requesting data from child nodes
#define OP_FIND_NODES 0x40		// the opcode for notifying of mapping and
#define OP_CHECK_HARDWARE 0x60	// the opcode for requesting hardware ids from children
#define OP_ASSIGN_NODE 0x80		// the opcode to assign node numbers

// define hardware id
#define NODE_TYPE 0x0002
#define NODE_ID 0x0000

#define WATCHDOG_RESET (WDTPW | WDTSSEL_1 | WDTCNTCL | WDTIS_3) // Reset watchdog timer with trigger of 16 seconds

//prototype functions
void waitingLoop();
void receiveMessage();
void requestData();
void sample();
void buildMap();
void findNodes();
void returnHardware();
void assignNode();
void setup_ports();
void setup_spi();
void setup_display();
void setup_buttons();
void setup_adc();
void setup_timers();
unsigned char message(unsigned char opCode, unsigned char nodeNumber, unsigned char direction);
void reply(unsigned char transmission);
void mapChildren(unsigned char* totalCount, unsigned char direction);
unsigned long getHardware(volatile unsigned int *destination, unsigned char count);

volatile unsigned int *connections[2] = {&UCA0TXBUF, &UCB0TXBUF};	// child connection transmission buffers
unsigned char thisNode = 0xFF;					// nodeNumber for this node
volatile unsigned char recentData[TOTAL_NODES] = {0};				// most recent data received from child nodes
volatile unsigned char childrenDirection[TOTAL_NODES] = {0};		// direction of child nodes: 0-not a child, 1-UCA0, 2-UCB0, 0xFF-no node
volatile unsigned long hardwareId[TOTAL_NODES] = {0};				// hardware id of the child nodes
volatile unsigned char hardwareDirection[TOTAL_NODES] = {0};		// direction of the hardware id
volatile unsigned char childrenCount = 0;		// Number of children of this node
volatile unsigned char RXData;					// Most recent transmission received
volatile unsigned char state = 0;				// state machine for controlling waiting loop
volatile unsigned char mapBuilt = 1;			// boolean used to indicate that the map is finished being built

/**
 * main.c
 */
int main(void) {
	WDTCTL = WATCHDOG_RESET;	// Setup watchdog timer

	hardwareId[0] = NODE_TYPE;
	hardwareId[0] = (hardwareId[0] << 16) | NODE_ID;

	setup_ports();
	setup_spi();
	setup_display();
	setup_buttons();
	setup_adc();
	setup_timers();

	PM5CTL0 &= ~LOCKLPM5; // Enable the GPIO pins
	__bis_SR_register(GIE);
	waitingLoop();
}

/**
 * waitingLoop
 * Node mainloop waiting for command from master
 */
void waitingLoop(){
	while(1){
		__bis_SR_register(LPM0_bits);
		TA3CTL &= ~MC_3;
		switch(state){
		case 1: // Received a message from master
			receiveMessage();
			break;
		case 2: // Request Data
			requestData();
			P9OUT ^= STATUS_LED_1;
			break;
		case 3: // Sample Data
			sample();
			break;
		default:
			break;
		}
		state = 0;
		TA3CTL |= MC_2;
	}
}

/**
 * receiveMessage
 * Process the message received by this node from its master.
 */
void receiveMessage(){
	switch(RXData&OP_bits){
	case OP_NULL:			// Null opcode: do nothing
		break;
	case OP_REQUEST_DATA:	// reply with the most recent data for the requested node
		if((RXData&(~OP_bits)) == thisNode){
			P1OUT ^= STATUS_LED_0;
		}
		reply(recentData[RXData&(~OP_bits)]);
		break;
	case OP_FIND_NODES:		// Start mapping, or return children count
		findNodes();
		break;
	case OP_CHECK_HARDWARE:	// return the hardware id for the requested node
		returnHardware();
		break;
	case OP_ASSIGN_NODE:	// assign a node number to a given hardware id
		assignNode();
		break;
	}
}

/**
 * requestData
 * request data from all child nodes
 */
void requestData(){
	unsigned char node = TOTAL_NODES, m;
	// Request data through all child nodes
	while(--node){
		m = message(OP_REQUEST_DATA, node, childrenDirection[node]);	// Request data from $node
		if(m)
			recentData[node] = m;										// Save the data returned by the child node
	}
}

/**
 * sample
 * sample the adc input to P8.4
 */
void sample(){
	ADC12CTL0 |= ADC12SC;    // Start sampling/conversion
	__bis_SR_register(LPM0_bits);		// LPM0, ADC12_ISR will force exit
	//recentData[thisNode] = ADC12MEM0 >> 4;
}

/**
 * findNodes
 * Build the map or return the number of children of this node
 */
void findNodes(){
	unsigned char subcode = RXData&(~OP_bits);
	switch(subcode){
	case 0: // Return the number of children
		reply(childrenCount);
		break;
	case 1:
		// this shouldn't be reached
		break;
	case 2: // build map and notify children of mapping command
		buildMap();
		break;
	}
}

/**
 * buildMap
 * find the hardware addresses of the nodes in the network.
 */
void buildMap(){
	P1OUT ^= STATUS_LED_0;
	UCB1TXBUF = 0xFF;		// notify the parent node that the build map command was received
	mapBuilt = 0;			// Mapping started

	// Command all nodes of the network to map their children
	// start on the edge of the network and move towards the base node
	unsigned char returns = 0;
	UCA0TXBUF = OP_FIND_NODES|2;	// Send mapping command to UCA0 child
	__bis_SR_register(LPM0_bits);
	__delay_cycles(100);
	UCA0TXBUF = 0;
	__bis_SR_register(LPM0_bits);
	if(RXData)		// There is a child node in the UCA0 direction
		returns |= 0x01;

	UCB0TXBUF = OP_FIND_NODES|2;	// Send mapping command to UCB0 child
	__bis_SR_register(LPM0_bits);
	__delay_cycles(100);
	UCB0TXBUF = 0;
	__bis_SR_register(LPM0_bits);
	if(RXData)		// There is a child node in the UCB0 direction
		returns |= 0x02;

	if(!returns){ // No Children; this is the only node in the network
		unsigned char node = TOTAL_NODES;
		while(--node){ // clear childrenDirection & hardwareDirection
			hardwareId[node] = 0;
		}
		mapBuilt = 1;		// Mapping finished
		childrenCount = 1;
		return;
	}

	// Wait until all children have finished mapping
	while(returns){
		TA1CTL |= MC_1 | TACLR;
		__bis_SR_register(LPM0_bits);
		if(returns & 0x01){		// child on UCA0 hasn't finished mapping
			UCA0TXBUF = OP_FIND_NODES|1;	// check if the child on UCA0 has finished mapping
			__bis_SR_register(LPM0_bits);
			__delay_cycles(100);
			UCA0TXBUF = 0;
			__bis_SR_register(LPM0_bits);
			if(!UCA0RXBUF)		// UCA0 child has finished mapping
				returns &= 0xFE;
		}

		if(returns & 0x02){		// child on UCB0 hasn't finished mapping
			UCB0TXBUF = OP_FIND_NODES|1;	// check if the child on UCB0 has finished mapping
			__bis_SR_register(LPM0_bits);
			__delay_cycles(100);
			UCB0TXBUF = 0;
			__bis_SR_register(LPM0_bits);
			if(!UCB0RXBUF)		// UCB0 child has finished mapping
				returns &= 0xFD;
		}
	}
	unsigned char node = TOTAL_NODES, totalCount = 1;
	while(--node){ // clear childrenDirection & hardwareDirection
		hardwareId[node] = 0;
	}

	mapChildren(&totalCount, 0);	// map the UCA0 child
	mapChildren(&totalCount, 1);	// map the UCB0 child
	childrenCount = totalCount;		// update childrenCount

	mapBuilt = 1;	// mapping finished
}

/**
 * mapChildren
 * Receive the list of all the nodes connected to the child node in the given direction.
 *
 * @params:
 * 	unsigned char*	totalCount	(pointer to the total count of child nodes, which will be
 * 								 increased by the number of child nodes in the given direction)
 * 	unsigned char	direction	(the direction of the child node being mapped)
 */
void mapChildren(unsigned char* totalCount, unsigned char direction){
	*connections[direction] = OP_FIND_NODES;	// Request the number of child nodes of the child
	__bis_SR_register(LPM0_bits);
	__delay_cycles(100);
	*connections[direction] = OP_NULL;
	__bis_SR_register(LPM0_bits);
	unsigned char count = RXData;
	while(count){			// Iterate through getting the hardware ids of the child's children
		unsigned long id = getHardware(connections[direction], --count);
		hardwareId[*totalCount] = id;					// save the hardware id
		hardwareDirection[*totalCount] = direction+1;	// save the hardware direction
		(*totalCount)++;
	}
}

/**
 * getHardware
 * Get the hardware id from the child node in the given count position.
 *
 * @params:
 * unsigned int*	destination		(eUSCI TX buffer used to ask the child node for its child id)
 * unsigned char	count			(index of the hardware id being requested)
 *
 * @return:
 * unsigned long	hardwareId		(hardware id of the requested node)
 */
unsigned long getHardware(volatile unsigned int *destination, unsigned char count){
	unsigned long id = 0;
	*destination = OP_CHECK_HARDWARE | count;	// Retrieve the hardware id from the child at $destination of $count
	__bis_SR_register(LPM0_bits);
	__delay_cycles(200);
	*destination = OP_NULL;						// Retrieve the first byte of the hardware id
	__bis_SR_register(LPM0_bits);
	id |= RXData;								// Save the first byte
	id <<= 8;
	__delay_cycles(200);
	*destination = OP_NULL;						// Retrieve the second byte of the hardware id
	__bis_SR_register(LPM0_bits);
	id |= RXData;								// Save the second byte
	id <<= 8;
	__delay_cycles(200);
	*destination = OP_NULL;						// Retrieve the third byte of the hardware id
	__bis_SR_register(LPM0_bits);
	id |= RXData;								// Save the third byte
	id <<= 8;
	__delay_cycles(200);
	*destination = OP_NULL;						// Retrieve the fourth byte of the hardware id
	__bis_SR_register(LPM0_bits);
	id |= RXData;								// Save the fourth byte
	return id;
}

/**
 * returnHardware
 * Reply to the master node with the hardware id of the requested node
 */
void returnHardware(){
	unsigned char node = RXData&(~OP_bits);
	reply((hardwareId[node] & 0xFF000000) >> 24);	// Send the first byte of the assigned id number
	__bis_SR_register(LPM0_bits);
	reply((hardwareId[node] & 0xFF0000) >> 16);		// Send the second byte of the assigned id number
	__bis_SR_register(LPM0_bits);
	reply((hardwareId[node] & 0xFF00) >> 8);		// Send the third byte of the assigned id number
	__bis_SR_register(LPM0_bits);
	reply(hardwareId[node] & 0xFF);					// Send the fourth byte of the assigned id number
	__bis_SR_register(LPM0_bits);
	P1OUT ^= STATUS_LED_0;
}

/**
 * assignNode
 * assign a hardware id to a node number
 */
void assignNode(){
	unsigned char node = RXData & (~OP_bits);
	unsigned long id = 0;
	__bis_SR_register(LPM0_bits);
	id |= RXData;						// Recieve the first byte of the hardware id
	id <<= 8;
	__bis_SR_register(LPM0_bits);
	id |= RXData;						// Recieve the second byte of the hardware id
	id <<= 8;
	__bis_SR_register(LPM0_bits);
	id |= RXData;						// Recieve the third byte of the hardware id
	id <<= 8;
	__bis_SR_register(LPM0_bits);
	id |= RXData;						// Recieve the fourth byte of the hardware id
	if(id == hardwareId[0]){	// the hardware id assignment is for this node
		childrenDirection[node] = 0xFF;
		thisNode = node;
	} else {
		unsigned int n = 0;
		while(id != hardwareId[++n])// find the hardware entry number for the prescribed node
			if(n == TOTAL_NODES)
				return;
		childrenDirection[node] = hardwareDirection[n];
		volatile unsigned int* destination = connections[childrenDirection[node]-1];

		*destination = OP_ASSIGN_NODE | node;		// Assign node to have the node number = $totalCount
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = (id & 0xFF000000) >> 24;		// Send the first byte of the assigned id number
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = (id & 0xFF0000) >> 16;		// Send the second byte of the assigned id number
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = (id & 0xFF00) >> 8;			// Send the third byte of the assigned id number
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = id & 0xFF;					// Send the fourth byte of the assigned id number
	}
}

/**
 * setup_ports
 * Initialize ports to reduce power usage
 */
void setup_ports(){
	P1DIR = 0xFF;
	P1OUT = 0x00;
	P2DIR = 0xFF;
	P2OUT = 0x00;
	P3DIR = 0xFF;
	P3OUT = 0x00;
	P4DIR = 0xFF;
	P4OUT = 0x00;
	P5DIR = 0xFF;
	P5OUT = 0x00;
	P6DIR = 0xFF;
	P6OUT = 0x00;
	P7DIR = 0xFF;
	P7OUT = 0x00;
	P8DIR = 0xFF;
	P8OUT = 0x00;
	P9DIR = 0xFF;
	P9OUT = 0x00;
	P10DIR = 0xFF;
	P10OUT = 0x00;
}

/**
 * setup_spi
 * setup USCI_A0, USCI_B0 for SPI master operation.
 * setup USCI_B1 for SPI slave operation.
 */
void setup_spi(){
	// Configure USCI_A0 for SPI operation
	UCA0CTLW0 = UCSWRST;
	UCA0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // Setup USCI_A0 to be a 3-pin, 8-bit SPI master

	UCA0CTLW0 |= UCSSEL_1;
	UCA0BR0 = 2;
	UCA0BR1 = 0;
	UCA0MCTLW = 0;

	UCA0CTLW0 &= ~UCSWRST; // Initialize USCI_A0 state machine

	UCA0IE |= UCRXIE;

	P2SEL0 |= BIT0 | BIT1 | BIT2;

	// Configure USCI_B0 for SPI operation
	UCB0CTLW0 = UCSWRST;
	UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // Setup USCI_A0 to be a 3-pin, 8-bit SPI master

	UCB0CTLW0 |= UCSSEL_1;
	UCB0BR0 = 2;
	UCB0BR1 = 0;

	UCB0CTLW0 &= ~UCSWRST; // Initialize USCI_B0 state machine

	UCB0IE |= UCRXIE;

	P1SEL0 |= BIT4 | BIT6 | BIT7;

	// Configure USCI_B1 for SPI operation
	UCB1CTLW0 = UCSWRST;
	UCB1CTLW0 |= UCSYNC | UCCKPL | UCMSB; // Setup USCI_A0 to be a 3-pin, 8-bit SPI slave

	UCB1CTLW0 |= UCSSEL_3;
	UCB1BR0 = 2;
	UCB1BR1 = 0;

	UCB1CTLW0 &= ~UCSWRST; // Initialize USCI_B1 state machine

	UCB1IE |= UCRXIE;

	P4SEL1 |= BIT0 | BIT1 | BIT2;

}

/**
 * setup_display
 * Setup on-board LEDs and LCD.
 */
void setup_display(){
	P1DIR |= STATUS_LED_0;
	P1OUT &= ~STATUS_LED_0;
	P9DIR |= STATUS_LED_1;
	P9OUT &= ~STATUS_LED_1;
}

/**
 * setup_buttons
 * Setup on-board button interrupts.
 */
void setup_buttons(){
	// Set P1.1 and P1.2 to input
	P1DIR &= ~(BUTTON_0 | BUTTON_1);
	// Enable pullup resistors for P1.1 & P1.2
	P1REN |= BUTTON_0 | BUTTON_1;
	P1OUT |= BUTTON_0 | BUTTON_1;
	// Enable interrupts for P1.1 & P1.2 on a falling edge
	P1IE |= BUTTON_0 | BUTTON_1;
	P1IES |= BUTTON_0 | BUTTON_1;
	P1IFG &= ~(BUTTON_0 | BUTTON_1);
}

/**
 * setup_adc
 * Setup ADC12 registers to sample P8.4
 */
void setup_adc(){
	ADC12CTL0 &= ~ADC12ENC;				// Disable ADC in order to modify the ADC12 control registers
	ADC12CTL0 |= ADC12SHT0_2 | ADC12ON;	// Setup ADC12MEM0-ADC12MEM7 to hold for 16 clock cycles
	ADC12CTL1 |= ADC12SHP;				// Use the sampling timer
	ADC12CTL2 |= ADC12RES_0;			// Use 8-bit resolution
	ADC12MCTL0 |= ADC12INCH_7;
	ADC12CTL0 |= ADC12ENC;				// Enable ADC
	ADC12IER0 |= 0x0001;				// Enable interrupts for ADC12MEM0
}

/**
 * setup_timers
 * Setup timers for periodic messaging
 */
void setup_timers(){
	// Setup timers to periodically request data, sample inputs, and update the map of the network
	TA3CCTL1 |= CCIE;
	TA3CCR1 = START_REQUEST_DATA;
	TA3CCTL1 &= ~CCIFG;
	TA3CCTL2 |= CCIE;
	TA3CCR2 = START_SAMPLE;
	TA3CCTL2 &= ~CCIFG;
	TA3CCTL3 |= CCIE;
	TA3CCR3 = TIME_NULL_TRANSMISSION;
	TA3CCTL3 &= ~CCIFG;
	TA3CCTL4 |= CCIE;
	TA3CCR4 = TIME_NULL_TRANSMISSION;
	TA3CCTL4 &= ~CCIFG;
	TA3CTL |= TASSEL_1 | ID_3 | MC_2 | TACLR;

	// Setup mapping timer
	TA1CCTL0 |= CCIE;
	TA1CCR0 = MAPPING_DELAY;
	TA1CCTL0 &= ~CCIFG;
	TA1CTL = TASSEL_1 | ID_3 | MC_0 | TACLR;
}

/**
 * message
 * Send a message with the opCode as a command, and the nodeNumber as a
 */
unsigned char message(unsigned char opCode, unsigned char data, unsigned char direction){
	switch(direction){
	case 0: // Not a child
		return 0;
	case 1: // send on UCA0 SPI
		TA3CCR3 = TA3R + TIME_NULL_TRANSMISSION;
		while(!(UCA0IFG & UCTXIFG));
		UCA0TXBUF = opCode | data;
		__bis_SR_register(LPM0_bits);		// LPM0, UCA0_ISR will force exit

		__delay_cycles(100);
		UCA0TXBUF = OP_NULL;
		__bis_SR_register(LPM0_bits);		// LPM0, UCA0_ISR will force exit

		return RXData;
	case 2: // send on UCB0 SPI
		TA3CCR4 = TA3R + TIME_NULL_TRANSMISSION;
		while(!(UCB0IFG & UCTXIFG));
		UCB0TXBUF = opCode | data;
		__bis_SR_register(LPM0_bits);		// LPM0, UCB0_ISR will force exit

		__delay_cycles(100);
		UCB0TXBUF = OP_NULL;
		__bis_SR_register(LPM0_bits);		// LPM0, UCB0_ISR will force exit

		return RXData;
	case 0xFF: // This node
		return 0;
	default: // ERROR
		return 0xFF;
	}
}

/**
 * reply
 * Send a reply to the master node.
 */
void reply(unsigned char transmission){
	UCB1TXBUF = transmission;
}

/**
 * UCA0 interrupt
 * SPI communication interrupt from slave
 */
#pragma vector = USCI_A0_VECTOR
__interrupt void uca0_isr(){
	switch(UCA0IV){
	case 0x02: // Received message from slave on UCA0
		RXData = UCA0RXBUF;
		__bic_SR_register_on_exit(LPM0_bits);	// Exit low power mode
		break;
	default:
		break;
	}
}

/**
 * UCB0 interrupt
 * SPI communication interrupt from slave
 */
#pragma vector = USCI_B0_VECTOR
__interrupt void ucb0_isr(){
	switch(UCB0IV){
	case 0x02: // Received message from slave on UCB0
		RXData = UCB0RXBUF;
		__bic_SR_register_on_exit(LPM0_bits);	// Exit low power mode
		break;
	default:
		break;
	}
}

/**
 * UCB1 interrupt
 * SPI communication interrupt from master
 */
#pragma vector = USCI_B1_VECTOR
__interrupt void ucb1_isr(){
	switch(UCB1IV){
	case 0x02: // Received message from master on UCB1
		WDTCTL = WATCHDOG_RESET;	// Reset watchdog timer
		RXData = UCB1RXBUF;
		if(RXData == (OP_FIND_NODES | 1)){
			if(!mapBuilt)
				UCB1TXBUF = 0xFF;
			else
				UCB1TXBUF = 0;
			break;
		}
		state = 1;
		__bic_SR_register_on_exit(LPM0_bits);	// Exit low power mode
		break;
	default:
		break;
	}
}

/**
 * Port 1 interrupt
 * Button interrupts
 */
#pragma vector = PORT1_VECTOR
__interrupt void p1_isr(){
	switch(P1IV){
	case P1IV_P1IFG1: // P1.1 falling edge interrupt
		if(recentData[thisNode] == 0x01)
			recentData[thisNode] = 0x3F;
		else if(recentData[thisNode] == 0x3F)
			recentData[thisNode] = 0xFF;
		else
			recentData[thisNode] = 0x01;
		P1IFG &= ~BIT1;
		break;
	}
}

/**
 * adc_isr
 * ADC12 interrupt subroutine
 * Clear interrupt flag for ADC12 and exit low power mode.
 */
#pragma vector = ADC12_VECTOR
__interrupt void adc_isr(){
	switch(ADC12IV){
	case ADC12IV_ADC12IFG0:
		ADC12IFGR0 &= ~0x0001;	// Clear interrupt flag for ACD12MEM0
		__bic_SR_register_on_exit(LPM0_bits);	// Exit low power mode
		break;
	default:
		break;
	}
}

/**
 * Timer3 A1 interrupt
 * Timer for periodic data requests and data sampling
 */
#pragma vector = TIMER3_A1_VECTOR
__interrupt void t0a1_isr(){
	switch(TA3IV){
	case TA3IV_TACCR1: // Request Data
		TA3CCR1 += TIME_REQUEST_DATA;
		TA3CCTL1 &= ~CCIFG;
		state = 2;
		__bic_SR_register_on_exit(LPM0_bits);
		break;
	case TA3IV_TACCR2: // Sample Data
		TA3CCR2 += TIME_SAMPLE;
		TA3CCTL2 &= ~CCIFG;
		state = 3;
		__bic_SR_register_on_exit(LPM0_bits);
		break;
	case TA3IV_TACCR3:
		TA3CCR3 += TIME_NULL_TRANSMISSION;
		TA3CCTL3 &= ~CCIFG;
		//UCA0TXBUF = 0;
		break;
	case TA3IV_TACCR4:
		TA3CCR4 += TIME_NULL_TRANSMISSION;
		TA3CCTL4 &= ~CCIFG;
		//UCB0TXBUF = 0;
		break;
	}
}

/**
 * Timer1 A0 interrupt
 * Timer for mapping delay
 */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void t1a0_isr(){
	TA1CTL &= ~MC_3;
	TA1CTL |= TACLR;
	TA1CCTL0 &= ~CCIFG;
	__bic_SR_register_on_exit(LPM0_bits);
}
