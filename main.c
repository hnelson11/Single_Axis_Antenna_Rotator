#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MAX_SUBSTRINGS 100
#define MAX_COMMAND_ARG 5
#define ARG_2 1
#define ARG_1 0
#define COMMAND_MAX_COUNT 5
#define ANGLE_MAX_COUNT 10

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 2
#define STOP 3

/**--GLOBAL VARIABLES DECLARATION--**/

char RXData[20] = "";
char command[20] = "";
char* cmd = "";
char response[20];
unsigned int command_index = 0;
unsigned int K = 0;
uint8_t command_received = 0;
uint8_t vtest = 0;
//The time at which TA1R was last reset. Keeps track of a longer maximum time (~17 million years, compared to TA1R's 2 seconds)
int64_t stored_time = 0;
char* command_substrings[MAX_COMMAND_ARG];

struct AngleCommand {
    int16_t targetAngle;
    int64_t time;
};
uint8_t current_command_index=0;
struct AngleCommand previous_commands[COMMAND_MAX_COUNT];

struct ADCAngle {
    int16_t angleRead;
    int64_t time;
};
uint8_t current_angle_index=0;
struct ADCAngle previous_angles[ANGLE_MAX_COUNT];


/*--FUNCTION DECLARATIONS--*/
void init_LEDs(void);
void init_UART(int Rx, int Tx);
void UART_Transmit(char string[]);
//void delay_ms(flaot ms);
void UART_Read_Command(void);
void initButton(void);
void init_GPIO(void);
int split_by_space(char *str, char *substrings[]);
void init_ADC(void);
void init_PWM_p1_7(int counter_period);
long int millis();
int64_t currentTime();
int16_t getPWMRequired();
int64_t getAcceleration(int32_t Vr, int32_t Vt, int32_t v, int8_t B, int8_t k, int8_t m);
const char digits[10] = {'0','1','2','3','4','5','6','7','8','9'};

//Check if string starts with a substring
int startsWith(char * wholeString, char * prefixString) {
    return strncmp(wholeString, prefixString, strlen(prefixString))==0;
}

int32_t abs(int32_t input) {
    if(input < 0) return -input;
    return input;
}
int32_t max(int32_t input1, int32_t input2) {
    if(abs(input1) > abs(input2)) return abs(input1);
    return abs(input2);
}

/**
 * main.c
 */
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    init_UART(1,1);
    init_LEDs();
    init_GPIO();
    TA1CCTL0 = CCIE;
    TA1CCR0 = 0xFFFF;
    TA1CTL = TASSEL__ACLK | MC__UP;
    init_ADC();
    // Enable UART Interrupt
    UCA0IE |= UCRXIE;          // LOCAL ENABLE FOR A0 RXIFG
    // Enable all interrupts
    __enable_interrupt();       //global en for maskables.... (GIE bit in SR)

    //ADC enable conversion and start conversion
    ADCCTL0 |= ADCENC | ADCSC;

    PM5CTL0 &= ~LOCKLPM5;           //TURN ON I/O

    init_PWM_p1_7(0);
    ADCCTL0 |= ADCSC;
    TA0CTL = MC__STOP;
    while(1)
    {
        __no_operation(); //for debug
        int response_length = 0;
        if(command_received) {
            command_received = 0;
            if (startsWith(cmd,"AZ"))
            {
                if(!(cmd[2] == ' ' || cmd[2] == '\r' || cmd[2] == '\n' || cmd[2]=='\0')) {
                    //TODO: rotate azimuth
                    int target_angle = atoi(cmd+2);
                    int16_t internal_target_angle = (int16_t)((target_angle * 65536/2160));
                    struct AngleCommand ac;
                    ac.targetAngle = internal_target_angle;
                    ac.time = currentTime();
                    current_command_index++;
                    previous_commands[current_command_index]=ac;
                    if(current_command_index>=COMMAND_MAX_COUNT)
                        current_command_index = 0;
                }
                int16_t currentAngle = previous_angles[current_angle_index].angleRead;
                int16_t angleApprox = currentAngle / 30 - currentAngle / 2671; //Taylor series approximation of angle/(65536/2160)
                uint8_t i=0;
                response[i++]='A';
                response[i++]='Z';
                if(angleApprox < 0) {
                    response[i++]='-';
                    angleApprox = -angleApprox;
                }
                int div = 1000;
                int k = i+4;
                uint8_t significant = 0;
                while(i < k) {

                    uint8_t digit = (angleApprox/div)%10;
                    if(digit != 0 || significant) {
                        //only move forward if significant digit (trim leading zeros)
                        response[i++]=digits[digit];
                        significant=1;
                    } else {
                        //reduce length when trimming leading zeros
                        k--;
                    }
                    div=div/10;
                }
                //Do not provide elevation as this is a 1 DOF rotator
                char b[] = ".0 EL0.0\r\n";
                memcpy(response+i,b,11);
                i+=11;
                response_length = i;
            }
            else if (startsWith(cmd,"SA"))
            {
                //Set ALL previous commands to current position
                struct AngleCommand ac;
                ac.targetAngle = previous_angles[current_angle_index].angleRead;
                ac.time = currentTime();
                int i;
                for(i = 0;i<COMMAND_MAX_COUNT;i++) {
                    previous_commands[i]=ac;
                }
            } else if (startsWith(cmd,"vtest_5")) {
                vtest=1;
                int i;
                int16_t currentAngle = previous_angles[current_angle_index].angleRead;
                int64_t currenttime = currentTime();
                int16_t deltaAngle = -8192-currentAngle;
                int64_t deltaTime = (abs((int32_t)deltaAngle)*32768)/151;
                for(i = 0;i<3;i++) {
                    struct AngleCommand ac;
                    if(currentAngle<-8192) {
                        ac.targetAngle = currentAngle + 152*i;
                    } else {
                        ac.targetAngle = currentAngle - 152*i;
                    }
                    ac.time = currenttime + 32768* i;
                    previous_commands[i+2]=ac;
                }
                current_command_index = 3;
            }
        }

        //Send command
        if(response_length) {

            int i;
            for (i = 0; i<response_length;i++) {
                while(!(UCA0IFG & UCTXIFG)); //wait for TX to be free
                UCA0TXBUF=response[i];
            }
        }
        //velocity test
        if(vtest==1) {
            if(previous_angles[current_angle_index].angleRead<-8192) {
                int i;
                int16_t currentAngle = previous_angles[current_angle_index].angleRead;
                int64_t currenttime = currentTime();
                for(i = 0;i<3;i++) {
                    struct AngleCommand ac;
                    ac.targetAngle = currentAngle + 152*i;
                    ac.time = currenttime + 32768 * i;
                    previous_commands[i+1]=ac;
                }
                vtest = 2;
            }
            current_command_index = 3;


        } else if (vtest==2) {
            if(previous_angles[current_angle_index].angleRead>8192) {
                struct AngleCommand ac;
                ac.targetAngle = previous_angles[current_angle_index].angleRead;
                ac.time = currentTime();
                int i;
                for(i = 0;i<COMMAND_MAX_COUNT;i++)
                    previous_commands[i]=ac;
                vtest=0;
            }
            current_command_index = 3;

        }

        //PWM frequency in microsteps per second
        int16_t PWMfrequency = getPWMRequired();
        //Negative values mean the direction should be clockwise
        if(PWMfrequency < 0) {
            P1OUT &= ~BIT4;
            PWMfrequency = -PWMfrequency;
        //Positive means counter-clockwise
        } else {
            P1OUT |= BIT4;
        }
        //32.768kHz timer 0, so 32768/f = period
        init_PWM_p1_7(32768/PWMfrequency);



        __no_operation(); //for debug
        __bis_SR_register(LPM0_bits);       // Enter LPM. Will not leave until ADC read or command receive.
    }
}
int32_t previousVelocity = 0;
uint8_t accelerationIndex=0;
int64_t accelerations[8];
int16_t getPWMRequired() {
    //Get the index of the most recent ADC reading as well as the oldest ADC reading.
    uint8_t nextADCIndex = current_angle_index+1;
    if (nextADCIndex==ANGLE_MAX_COUNT) nextADCIndex=0;

    //Get the index of the most recent command as well as the second most recent command.
    uint8_t nextCommandIndex = current_command_index-1;
    if (nextCommandIndex>=COMMAND_MAX_COUNT) nextCommandIndex=COMMAND_MAX_COUNT-1;
    uint8_t nextNextCommandIndex = nextCommandIndex-1;
    if (nextNextCommandIndex>=COMMAND_MAX_COUNT) nextNextCommandIndex=COMMAND_MAX_COUNT-1;

    //Current ADC angle/time
    int16_t currentADCAngle = (previous_angles[current_angle_index].angleRead);
    int64_t currentADCTime = (previous_angles[current_angle_index].time);

    //Current Command angle/time
    int16_t currentCommandAngle = (previous_commands[current_command_index].targetAngle);
    int64_t currentCommandTime = (previous_commands[current_command_index].time);

    //Change in measured angle
    int16_t deltaADCAngle = (previous_angles[current_angle_index].angleRead - previous_angles[nextADCIndex].angleRead);
    //Change in measured time
    int64_t deltaADCTime = (previous_angles[current_angle_index].time - previous_angles[nextADCIndex].time);

    //Change in command angle
    int16_t deltaCommandAngle = (previous_commands[current_command_index].targetAngle - previous_commands[nextCommandIndex].targetAngle);
    //Change in command time
    int64_t deltaCommandTime = (previous_commands[current_command_index].time - previous_commands[nextCommandIndex].time);
    //old changes in command times/angles
    int16_t deltaCommandAngleOld = (previous_commands[nextCommandIndex].targetAngle - previous_commands[nextNextCommandIndex].targetAngle);
    int64_t deltaCommandTimeOld = (previous_commands[nextCommandIndex].time - previous_commands[nextNextCommandIndex].time);
    //Velocity required to make it to target angle on schedule
    int32_t velocityRequired=0;
    //Velocity required to maintain tracking
    int32_t velocityTarget;
    //Current measured velocity
    int32_t velocityCurrent;

    //If ADC has no time in between, cannot calculate current velocity due to divide by zero
     if(deltaADCTime==0) {
         velocityCurrent=0;
     } else {
         velocityCurrent=(deltaADCAngle*32768)/deltaADCTime;
     }

     //If commands have no time in between, cannot calculate target velocity due to divide by zero
     if(deltaCommandTime==0) {
         velocityTarget=0;
     } else {
         int32_t velocityTargetNew = (deltaCommandAngle * 32768) / deltaCommandTime;
         int32_t velocityTargetOld = (deltaCommandAngleOld * 32768) / deltaCommandTimeOld;
         //If the most recent target velocity is more than 20% off of the old one, then it is considered a "new" tracking with unknown velocity.
         if(abs(velocityTargetNew-velocityTargetOld) > (max(velocityTargetNew,velocityTargetOld)*3)/10) {
             velocityTarget=0;
         } else {
             velocityTarget=velocityTargetNew;
         }
     }

     //Want to be in line with tracking 5 seconds from now (will take longer to properly converge)
     int64_t targetTime = currentADCTime + 5*32768;
     //Predict future target angle based on most recent command and the target velocity
     int16_t targetAngle = currentCommandAngle+(velocityTarget*(targetTime-currentCommandTime))/32768;
     //Velocity required to make it to the target angle at the target time
     velocityRequired=(currentADCAngle-targetAngle)*32768/(currentADCTime-targetTime);
     //Acceleration function to make movement smooth and reduce power spikes
     int32_t acceleration = getAcceleration(velocityRequired, velocityTarget, velocityCurrent, 3, 2, 5);
     //Limit acceleration to prevent a significant amount of jerk
     //max: 5 degrees/s^2
     //5 degrees = 5*65536/2160 = 151 steps
     //It made more sense in the calculator app
     if(acceleration > 151)
         acceleration = 151;
     if(acceleration < -151)
         acceleration = -151;
     //8th order lowpass filter to reduce jerk

     //v(t)=v(t-dt)+a*dt
     int32_t velocity = previousVelocity + ((acceleration * deltaADCTime/ANGLE_MAX_COUNT)/32768);
     //Limit velocity to 303 steps/second (10 degrees/second) to prevent bad things
     if(velocity > 303)
         velocity = 303;
     if(velocity < -303)
         velocity = -303;
     previousVelocity=velocity;
     //velocityTarget=151;
     //200 steps/360 degrees * 256 microsteps/step = 142 microsteps/degree
     int32_t PWM_freq = (velocity*2160*142)/65536;
     return (int16_t)PWM_freq;
}

//Acceleration function, to promote convergence towards the target velocity. B, k, m, are shape parameters.
int64_t getAcceleration(int32_t Vr, int32_t Vt, int32_t v, int8_t B, int8_t k, int8_t m) {
    return (Vr-Vt)/k+B*(Vr-v)/(1+(m*(Vr-Vt)*(Vr-Vt))/920);
}

#pragma vector = USCI_A0_VECTOR
__interrupt void EUSCI_A0_RX_ISR(void)
{
    char received_char;
    received_char = UCA0RXBUF;
    if (received_char == 10 || received_char == 13)
    {
        __bic_SR_register_on_exit(LPM0_bits); //Exit LPM so that velocity control can run
        command_index = 0;
        //RXData[K] = '\0';
        memcpy(command, RXData, K);
        memset(RXData,'\0',K);
        /*for (command_index = 0; command_index <= K; command_index++)
        {
            command[command_index] = RXData[command_index];
            RXData[command_index] = '\0';
        }*/
        cmd = &command[0];
        K = 0;
        command_received = 1;
    }
    else
    {
        RXData[K] = received_char;
        K++;
    }

    UCA0IFG &= ~UCRXIFG;

}

void init_UART(int Rx, int Tx)
{
    UCA0CTLW0 |= UCSWRST;           //put A0 into SW reset

    UCA0CTLW0 |= UCSSEL__SMCLK;     //BRCLK = SMCLK (want 19200 baud)
    UCA0BRW = 104;                    // prescalar = 52 => baud rate = 19200
    UCA0MCTLW = 0xD600;             //set modulation & low freq


    //-- setup ports
    if (Rx ==1)
        P1SEL0 |= BIT1;                 //P1.0 set function to UART A0 TX
    else
        P1SEL0 &= ~BIT1;                 //P1.0 set function to UART A0 TX
    if (Tx == 1)
        P1SEL0 |= BIT0;                 //P1.1 set function to UART A0 RX
    else
        P1SEL0 &= ~BIT0;                 //P1.0 set function to UART A0 TX

    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
}

void init_PWM_p1_7(int counter_period)
{
    P1DIR |= BIT7;        // set Pin 1.7 as output
    P1SEL0 |= BIT7;       // select Timer_A output for Pin 1.7
    if(counter_period==0) {
        TA0CTL = MC__STOP;
        return;
    }
    TA0CCR0 = counter_period;       // set the period of the PWM signal (1000 cycles)
    TA0CCTL1 = OUTMOD_7;  // set Timer_A output mode to reset/set (PWM mode)
    TA0CCR1 = (counter_period/2);        // set the duty cycle of the PWM signal (50%)
    TA0CTL = TASSEL__ACLK| MC__UP; // select SMCLK as clock source, UP mode,
}

int split_by_space(char *str, char *substrings[])
{
    int num_substrings = 0; // Number of substrings found
    char *token; // Pointer to the current substring

    // Split the string into substrings using strtok()
    token = strtok(str, " ");
    while (token != NULL && num_substrings < MAX_SUBSTRINGS)
    {
        substrings[num_substrings] = token;
        num_substrings++;
        token = strtok(NULL, " ");
    }

    return num_substrings;
}


// Initializes GPIO
void init_GPIO(void)
{
    // P1.3 can be MOTOR_OUT_PIN
    P1DIR |= BIT3;
    P1OUT |= BIT3;

    // P1.4 can be the MOTOR_DIR_PIN
    P1DIR |= BIT4;
    P1OUT &= ~BIT4;
}

void init_ADC(void)
{
    // Configure P8.0 as the ADC input
    P8SEL0 |= BIT0; // Set P8.0 as an analog input

    // Configure ADC
    ADCCTL0 &= ~ADCSHT2;    // Clearing the ADCSHTx bits of control register
    ADCCTL0 |= ADCSHT2;                          // 16 cycles conversion
    ADCCTL0 |= ADCON;                            //Turn ADC on

    // Use the SMCLK and SAMPCON signal is sourced from the sampling timer
    ADCCTL1 |= ADCSSEL1;
    ADCCTL1 |= ADCSHP;                  // Confused about this part
    //ADCCTL1 |= ADCDIV_4;              //Divide ADC clock by 4

    // Setting the ADC resolution to be 10 bits
    ADCCTL2 &= ~ADCRES_1;
    ADCCTL2 |= ADCRES_1;

    // Set ADCENC bit of ADCCTL0 so that ADCMCTL0 channel can be selected
    ADCCTL0 &= ~ADCENC;

    // ADC Conversion Memory Control Register
    // Choosing A8 as the input channel
    ADCMCTL0 |= ADCINCH_8;

    // Enable the ADC conversion complete interrupt
    ADCIE |= ADCIE0;

    //-- Setup IRQ      Conversion complete
    __enable_interrupt();
}
uint16_t adcCounter = 0;
uint32_t adcSum = 0;
//--- ISR
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    adcSum += ADCMEM0;
    if(adcCounter>=1024) {
        //64th order lowpass filter
        int16_t current_angle = (adcSum>>4) - 32768;
        //currentTime function; replaced with non-call because this is an ISR
        int64_t current_time = stored_time + TA1R;


        struct ADCAngle adc;
        adc.angleRead = current_angle;
        adc.time = current_time;
        current_angle_index++;
        if(current_angle_index>=ANGLE_MAX_COUNT)
            current_angle_index = 0;
        previous_angles[current_angle_index]=adc;
        adcSum=0;
        adcCounter=0;
        __bic_SR_register_on_exit(LPM0_bits);         //exit LPM so velocity control can run
    }
    adcCounter++;
    ADCCTL0 |= ADCSC;
}


// Initializes LED on P1.0 and P4.0 for use
void init_LEDs(void)
{

//    P1DIR |= BIT0;
//    P1OUT |= BIT0;

    P4DIR |= BIT0;
    P4OUT &= ~BIT0;
}

// Initializes button 2 for use
void initButton(void)
{
  P1DIR &= ~BIT2;        // Set P1.1 as input
  P1REN |= BIT2;         // Enable pull-up/down resistor on P1.2
  P1OUT |= BIT2;         // Set pull-up resistor on P1.2
  P1IES |= BIT2;         // Set P1.1 to trigger on falling edge
//  P1IE |= BIT2;          // Enable interrupt on P1.2
}
/**
 * Returns the current time *IN 1/2^9 SECONDS.* This is not milliseconds.
 */
int64_t currentTime()
{
    return stored_time + TA1R;
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A_ISR(void) {
    stored_time = stored_time + 0xFFFF;
}

//#pragma vector = PORT1_VECTOR
//__interrupt void Button1_2(void)
//{
//    if (!(P1IN & BIT2))
//    {
//        P4OUT |= BIT0;
//        delay_ms(5000);
//        P4OUT &= ~BIT0;
//    }
//    P1IFG &= ~BIT2; // clear P1.2 interrupt flag
//}

//
//// Delay function using ISR
//void delay_ms(flaot ms) {
//    // Setup Timer_A
//    TA1CTL = TASSEL__ACLK | MC__STOP;  // Use ACLK, stop timer
//    TA1CCR0 =(int) (32768.0f /1000.0f * ms);      // Set period to match desired delay
//    TA1CCTL0 = CCIE;                  // Enable interrupt
//
//    // Start Timer_A
//    TA1CTL |= MC__UP;                 // Start timer in up mode
//    __bis_SR_register(LPM3_bits + GIE);  // Enter low-power mode with interrupts enabled
//}
//

