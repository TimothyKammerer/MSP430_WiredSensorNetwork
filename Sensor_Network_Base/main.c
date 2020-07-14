#include <msp430.h>

/*********************************************
 * TODO:
 * network building with finding new nodes
 * network breaking
 * future features: Pg. 1 of notebook
 * Communicate Data to Computer
 *
 * FIXME:
 * node sampling isn't sampling accurately
 *********************************************/

#define BUTTON_0 BIT1		// On-board button P1.1
#define BUTTON_1 BIT2		// On-board button P1.2
#define STATUS_LED_0 BIT0	// On-board LED P1.0
#define STATUS_LED_1 BIT7	// On-board LED P9.7
#define TOTAL_NODES 0x20

// define message timing
#define START_REQUEST_DATA 0x0400		// Initial time to request data
#define TIME_REQUEST_DATA 0x0400		// delta time between data requests
#define TIME_NULL_TRANSMISSION 0x0800	// delta time between sending null transmission
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
#define NODE_TYPE 0x0001
#define NODE_ID 0x0000

//prototype functions
void mainloop();
void requestData();
void buildMap();
void setup_ports();
void setup_spi();
void setup_display();
void setup_buttons();
void setup_timers();
unsigned char message(unsigned char opCode, unsigned char nodeNumber, unsigned char direction);
void mapChildren(unsigned char* totalCount, unsigned char direction);
unsigned long getHardware(volatile unsigned int* destination, unsigned char count);

#define thisNode 0						// nodeNumber for this node
volatile unsigned char recentData[TOTAL_NODES] = {0};			// most recent data received from child nodes
volatile unsigned char childrenDirection[TOTAL_NODES] = {0};	// direction of child nodes: 0-not a child, 1-UCA0, 2-UCB0, 0xFF-no node
volatile unsigned long hardwareId[TOTAL_NODES] = {0};			// hardware id of the child nodes
volatile unsigned int *connections[3] = {&UCA0TXBUF, &UCB0TXBUF, &UCB1TXBUF};	// child connection transmission buffers
volatile unsigned char RXData;				// Most recent transmission received
volatile unsigned char PWM_input = 1;		// input used for the PWM of status led 0
volatile unsigned char state = 0;			// state machine for controlling mainloop
volatile unsigned char dormant = 0;			// boolean for whether the machine is dormant

/**
 * main.c
 */
int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	hardwareId[0] = NODE_TYPE;
	hardwareId[0] = (hardwareId[0] << 16) | NODE_ID;

	setup_ports();
	setup_spi();
	setup_display();
	setup_buttons();
	setup_timers();

	PM5CTL0 &= ~LOCKLPM5; // Enable the GPIO pins
	dormant = 1;
	__bis_SR_register(GIE);
	mainloop();
}

/**
 * mainloop
 */
void mainloop(){
	while(1){
		__bis_SR_register(LPM0_bits);
		dormant = 0;	// Disable interrupts
		switch(state){
		case 1: // Request Data
			requestData();						// Send data requests to all children
			P9OUT ^= STATUS_LED_1;				// Indicate that a data request has been sent
			TA0CCR1 = recentData[PWM_input];	// Update the status led 0 PWM to display the most recent data point
			break;
		case 2: // Update Map
			buildMap();
			break;
		default:
			break;
		}
		state = 0;
		dormant = 1;
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
 * buildMap
 * find the hardware addresses of the nodes in the network and assign node numbers to each one.
 */
void buildMap(){
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

	UCB1TXBUF = OP_FIND_NODES|2;	// send mapping command to UCB1 child
	__bis_SR_register(LPM0_bits);
	__delay_cycles(100);
	UCB1TXBUF = 0;
	__bis_SR_register(LPM0_bits);
	if(RXData)		// There is a child node in the UCB1 direction
		returns |= 0x04;

	if(!returns) // No Children; this is the only node in the network
		return;

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

		if(returns & 0x04){		// child on UCB1 hasn't finished mapping
			UCB1TXBUF = OP_FIND_NODES|1;	// check if the child on UCB1 has finished mapping
			__bis_SR_register(LPM0_bits);
			__delay_cycles(100);
			UCB1TXBUF = 0;
			__bis_SR_register(LPM0_bits);
			if(!UCB1RXBUF)		// UCB1 child has finished mapping
				returns &= 0xFB;
		}
	}

	// map all of the children of this node
	unsigned char totalCount = 1;
	mapChildren(&totalCount, 0);	// map the UCA0 child
	mapChildren(&totalCount, 1);	// map the UCB0 child
	mapChildren(&totalCount, 2);	// map the UCB1 child

	// notify nodes of node number assignments
	volatile unsigned int* destination;
	while(--totalCount){		// iterate through all of the
		destination = connections[childrenDirection[totalCount]-1];
		*destination = OP_ASSIGN_NODE | totalCount;					// Assign node to have the node number = $totalCount
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = (hardwareId[totalCount] & 0xFF000000) >> 24;	// Send the first byte of the assigned id number
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = (hardwareId[totalCount] & 0xFF0000) >> 16;	// Send the second byte of the assigned id number
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = (hardwareId[totalCount] & 0xFF00) >> 8;		// Send the third byte of the assigned id number
		__bis_SR_register(LPM0_bits);
		__delay_cycles(100);
		*destination = hardwareId[totalCount] & 0xFF;				// Send the fourth byte of the assigned id number

		// Delay before sending the next node number assignment
		TA1CTL |= MC_1 | TACLR;
		__bis_SR_register(LPM0_bits);
		TA1CTL |= MC_1 | TACLR;
		__bis_SR_register(LPM0_bits);
		TA1CTL |= MC_1 | TACLR;
		__bis_SR_register(LPM0_bits);
	}
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
		childrenDirection[*totalCount] = direction+1;	// save the hardware direction
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
	id |= RXData;
	id <<= 8;
	__delay_cycles(200);
	*destination = OP_NULL;						// Retrieve the second byte of the hardware id
	__bis_SR_register(LPM0_bits);
	id |= RXData;
	id <<= 8;
	__delay_cycles(200);
	*destination = OP_NULL;						// Retrieve the third byte of the hardware id
	__bis_SR_register(LPM0_bits);
	id |= RXData;
	id <<= 8;
	__delay_cycles(200);
	*destination = OP_NULL;						// Retrieve the fourth byte of the hardware id
	__bis_SR_register(LPM0_bits);
	id |= RXData;
	return id;
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
 * setup USCI_A0, USCI_B0, USCI_B1 for SPI master operation.
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

	UCA0IE |= UCRXIE | UCTXIE;

	P2SEL0 |= BIT0 | BIT1 | BIT2;

	// Configure USCI_B0 for SPI operation
	UCB0CTLW0 = UCSWRST;
	UCB0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // Setup USCI_B0 to be a 3-pin, 8-bit SPI master

	UCB0CTLW0 |= UCSSEL_1;
	UCB0BR0 = 2;
	UCB0BR1 = 0;

	UCB0CTLW0 &= ~UCSWRST; // Initialize USCI_B0 state machine

	UCB0IE |= UCRXIE | UCTXIE;

	P1SEL0 |= BIT4 | BIT6 | BIT7;

	// Configure USCI_B1 for SPI operation
	UCB1CTLW0 = UCSWRST;
	UCB1CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // Setup USCI_B1 to be a 3-pin, 8-bit SPI master

	UCB1CTLW0 |= UCSSEL_1;
	UCB1BR0 = 2;
	UCB1BR1 = 0;

	UCB1CTLW0 &= ~UCSWRST; // Initialize USCI_B1 state machine

	UCB1IE |= UCRXIE | UCTXIE;

	P4SEL1 |= BIT0 | BIT1 | BIT2;

}

/**
 * Setup on-board LEDs
 * STATUS_LED_0 = P1.0 (Red on-board LED)
 * STATUS_LED_1 = P9.7 (Green on-board LED)
 */
void setup_display(){
	// Status LED 0 is the RED LED on P1.0
	P1DIR |= STATUS_LED_0;
	P1OUT &= ~STATUS_LED_0;
	// Status LED 1 is the GREEN LED on P9.7
	P9DIR |= STATUS_LED_1;
	P9OUT &= ~STATUS_LED_1;
}

/**
 * setup_buttons
 * Setup on-board button interrupts.
 * BUTTON_0 = P1.1
 * BUTTON_1 = P1.2
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
 * setup_timers
 * Setup timers for periodic messaging
 * TA3CCR1 @ 8 Hz		(Control Data requesting)
 * TA0CCR1 @ 3.5 kHz w/ variable duty cycle		(STATUS_LED_0 PWM control)
 * TA1CCR0 @ 32 Hz		(Delay timer for mapping)
 */
void setup_timers(){
	// Setup timers to periodically request data and update the map of the network
	TA3CCTL1 |= CCIE;
	TA3CCR1 = START_REQUEST_DATA;
	TA3CCTL1 &= ~CCIFG;
	TA3CCTL2 |= CCIE;
	TA3CCR2 = TIME_NULL_TRANSMISSION;
	TA3CCTL2 &= ~CCIFG;
	TA3CCTL3 |= CCIE;
	TA3CCR3 = TIME_NULL_TRANSMISSION;
	TA3CCTL3 &= ~CCIFG;
	TA3CCTL4 |= CCIE;
	TA3CCR4 = TIME_NULL_TRANSMISSION;
	TA3CCTL4 &= ~CCIFG;
	TA3CTL = TASSEL_1 | ID_3 | MC_2 | TACLR;

	// Setup status LED to display received data as LED PWM
	P1SEL0 |= STATUS_LED_0;
	TA0CCTL1 |= OUTMOD_7;
	TA0CCR0 = 280;
	TA0CCR1 = 0;
	TA0CTL = TASSEL_2 | MC_1 | TACLR;

	// Setup mapping timer
	TA1CCTL0 |= CCIE;
	TA1CCR0 = MAPPING_DELAY;
	TA1CCTL0 &= ~CCIFG;
	TA1CTL = TASSEL_1 | ID_3 | MC_0 | TACLR;
}

/**
 * message
 * Send a message with the opCode as a command, and the nodeNumber as a destination.
 * @param:
 * 		unsigned char opCode		(operation code sent to leaf node)
 * 		unsigned char data			(data being sent with the operation code)
 * 		unsigned char direction		(Direction of the child node to which the message is being sent)
 *
 * @return:
 * 		unsigned char returnMessage (message returned by leaf node based on message sent)
 */
unsigned char message(unsigned char opCode, unsigned char data, unsigned char direction){
	switch(direction){
	case 0: // Not a child
		return 0;
	case 1: // send on UCA0 SPI
		TA3CCR2 = TA3R + TIME_NULL_TRANSMISSION;
		//while(!(UCA0IFG & UCTXIFG));
		UCA0TXBUF = opCode | data;
		__bis_SR_register(LPM0_bits);		// LPM0, UCA0_ISR will force exit

		__delay_cycles(100);
		UCA0TXBUF = OP_NULL;
		__bis_SR_register(LPM0_bits);

		return RXData;
	case 2: // send on UCB0 SPI
		TA3CCR3 = TA3R + TIME_NULL_TRANSMISSION;
		//while(!(UCB0IFG & UCTXIFG));
		UCB0TXBUF = opCode | data;
		__bis_SR_register(LPM0_bits);		// LPM0, UCB0_ISR will force exit

		__delay_cycles(100);
		UCB0TXBUF = OP_NULL;
		__bis_SR_register(LPM0_bits);		// LPM0, UCB0_ISR will force exit

		return RXData;
	case 3: // send on UCB1 SPI
		TA3CCR4 = TA3R + TIME_NULL_TRANSMISSION;
		//while(!(UCB1IFG & UCTXIFG));
		UCB1TXBUF = opCode | data;
		__bis_SR_register(LPM0_bits);		// LPM0, UCB1_ISR will force exit

		__delay_cycles(100);
		UCB1TXBUF = OP_NULL;
		__bis_SR_register(LPM0_bits);		// LPM0, UCB1_ISR will force exit

		return RXData;
	case 0xFF: // This node
		return 0;
	default: // ERROR
		return 0xFF;
	}
}

/**
 * UCA0 interrupt
 * SPI communication interrupt from slave
 * SPI transmission buffer empty
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
 * SPI transmission buffer empty
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
 * SPI communication interrupt from slave
 * SPI transmission buffer empty
 */
#pragma vector = USCI_B1_VECTOR
__interrupt void ucb1_isr(){
	switch(UCB1IV){
	case 0x02: // Received message from slave on UCB1
		RXData = UCB1RXBUF;
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
		P1IFG &= ~BUTTON_0;
		if(dormant)
			PWM_input ^= 0x3;
		break;
	case P1IV_P1IFG2: // P1.2 falling edge interrupt
		// map the network
		P1IFG &= ~BUTTON_1;
		if(dormant){
			state = 2;
			__bic_SR_register_on_exit(LPM0_bits);
		}
		break;
	}
}

/**
 * Timer3 A1 interrupt
 * Timer for periodic data requests
 */
#pragma vector = TIMER3_A1_VECTOR
__interrupt void t3a1_isr(){
	switch(TA3IV){
	case TA3IV_TACCR1: // Request Data
		TA3CCR1 += TIME_REQUEST_DATA;
		TA3CCTL1 &= ~CCIFG; // Clear interrupt flag
		if(dormant){
			state = 1;
			__bic_SR_register_on_exit(LPM0_bits);
		}
		break;
	case TA3IV_TACCR2:	// Null transmission to UCA0 when dormant
		TA3CCR2 += TIME_NULL_TRANSMISSION;
		TA3CCTL2 &= ~CCIFG;
		if(dormant)
			UCA0TXBUF = 0;
		break;
	case TA3IV_TACCR3:	// Null transmission to UCB0 when dormant
		TA3CCR3 += TIME_NULL_TRANSMISSION;
		TA3CCTL3 &= ~CCIFG;
		if(dormant)
			UCB0TXBUF = 0;
		break;
	case TA3IV_TACCR4:	// Null transmission to UCB1 when dormant
		TA3CCR4 += TIME_NULL_TRANSMISSION;
		TA3CCTL4 &= ~CCIFG;
		if(dormant)
			UCB1TXBUF = 0;
		break;
	}
}

/**
 * Timer1 A0 interrupt
 * Timer for mapping delay
 */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void t1a0_isr(){
	TA1CCTL0 &= ~CCIFG;
	TA1CTL &= ~MC_3;
	TA1CTL |= TACLR;
	__bic_SR_register_on_exit(LPM0_bits);
}
