/* D18 ACC DECODER

Introduction:
  ENGLISH:

    Without extension, this decoder allows to control 16 direct outputs (or 8 output pairs).
        This is the double of traditional DCC accessories decoders having 4 output pairs.
        Output can be fix or generate pulses (20ms->65s with 20ms resolution).
        Configuration is done in this code (in USER part) instead of using CVs.
    You can add MAX7219/21 modules in place of 3 direct outputs. Each module can drive 64 LEDs.
        Actual SW supports 4 modules so 256 leds.
        You can also control LEDs blinking and blinking phase.
    You can add PCA9685 modules in place of 2 direct outputs. Each module adds 16 outputs.
        Each of these outputs can work in PWM mode (0-100%), this allow dimming to control for instance lights intensity or motor speed.
            To have the behavior of a traditional output, you can une 0% and 100%.
        Each of these outputs can also work in SERVO mode to control a servo-motor.
        Actual SW supports 6 modules so 96 new outputs.
    You can also drive Neopixels (chained RGB leds) in place of 1 direct out.
    To be compatible with any centrals (and mainly your central) and be versatile,
        User can map any order ("Accessory" or "Output") from DCC signal to any output.
    By default SW is delivered to manage 8 pairs on address 10 & 11     

  FRANCAIS:

   Sans extension, ce décodeur permet de contrôler 16 sorties directes (ou 8 paires de sorties).
       Cela est le double d'un décodeur d'accessoires traditionnel (4 paires de sorties).
       Les sorties peuvent être fixes ou générer des impulsions (20ms->65s avec une résolution de 20ms).
   Vous pouvez ajouter des modules MAX7219/21 à la place de 3 sorties directes. Chaque module supporte 64 LEDs.
       Le programme actuel supporte 4 modules, soit 256 LEDs.
       Vous pouvez aussi contrôler le clignotement des LEDs ainsi que la phase.
   Vous pouvez ajouter des modules PCA9685 à la place de 2 sorties directes. Chaque module ajoute 16 sorties.
       Chacune de ces sorties peut fonctionner en mode PWM (0-100%), cela permet de faire varier l'éclairage ou la vitesse d'un moteur.
           Pour une sortie traditionnelle, utilisez 0% ou 100%
       Chacune de ces sorties peut fonctioner en mode SERVO (500ms->2500ms 50Hz), pour contrôler un servo-moteur.
       Le programme actuel supporte 6 modules, soit 96 nouvelles sorties.  
   Vous pouvez aussi contrôler des Néopixels (LEDs chainables multicolores) à la place d'une sortie directe.
   Pour être compatible avec toutes les centrales (et surtout la vôtre) et être passe partout.
       L’utilisateur peut utiliser n'importe quel ordre DCC ("Accessory" ou "Output") pour contrôler n'importe quelle sortie. 
       La configuration se fait dans ce code (dans la partie USER) au lieu d'utiliser les CV.
       Par défaut le programme est configuré pour contrôler 8 paires d sorties.
 
History:
    2017-02-08: Ulysse Delmas-Begue: ajout de la rotation lente des servos
                                     changement de la resolution des tempos de 20ms a 125ms
    2017-01-12: Ulysse Delmas-Begue: support Arduino Mega board to add 50 direct outputs in addition to Arduino UNO and Nano
                                     AT Mega is enbaled when user select Arduino Mega board in Arduino IDE
                                     serigraphied number is now used for direct outputs instead of out 0-15
                                     output number is checked in user_out function
    2017-01-12: Ulysse Delmas-Begue: support 256LEDs (via 4 MAX7219/21) and 96PWM/SERCO (via 6 PCA9685)
    2017-01-11: Ulysse Delmas-Begue: add 128leds (via MAX7219/21) OK, 48pwm/servos (via PCA9685 I2C), 60neopixels  (test MAX, PCA(PWM&SRV) et pulses ok, neo not stable board finish to freeze)
    2017-01-10: Ulysse Delmas-Begue: creation (et test out ok avec D17)

To be done:
    None

License: GPL v2    
                                  
Notes: 
    - I used foloowing code as startup point:
        Simple DCC Led Accessory Decoder Luca Dentella, 25.11.2017
        http://www.lucadentella.it/en/2017/11/25/dcc-decoder-accessori-per-led/
    - This code needs the following library to be installed in the Arduino IDE
        https://github.com/mrrwa/NmraDcc
    - switching on an output of the pair, automaticaaly switch off the other output
    - onboard led blinks at 1Hz or 4 Hz during 1sec when a packet matches
    
    Normally decoder address and pulses are set by following CVs but here they are hardcoded
    - CV513 ADR LSB 6bits
    - CV515 pair 0 pulse
    - CV516 pair 1 pulse
    - CV517 pair 2 pulse
    - CV518 pair 3 pulse duration
    - CV521 ADR MSB 3bits

Pinout on an Arduino UNO

     CLI +++ +++ +++ +++ +++    +++ +++ +++ +++ +++ DCC  TX  RX
     D13 D12 D11 D10  D9  D8     D7  D6  D5  D4  D3  D2  D1  D0
    ###########################################################      +++ = out
    #USB###########################################TOP#view####      +-+ = out or used by MAX,PCA,NEO
    ###########################################################      use following numbers in SW: 3-12 & A0-A5 
    ###########################################################      max out = 16
     x   x   x   x   x   x   x   x      A0  A1  A2  A3  A4  A5  
                                   +-+ +-+ +-+ +-+ +-+ +-+  
                                   CLK DAT LLED NEO SDA SCK

Pinout on an Arduino Nano

     +++ +++ +++ +++ +++  +++ +++ +++ +++ +++ DCC          TX  RX
     D12 D11 D10  D9  D8  D7  D6  D5  D4  D3  D2 GND  RST  D1  D0    +++ = out
    ##############################################################   +-+ = out when not used by MAX,PCA,NEO
    USB ##############################################TOP#view####   use following numbers in SW: 3-12 & A0-A5 
    ##############################################################   max out = 16
     D13 3.3   X  A0  A1  A2  A3  A4  A5  A6  A7  5V  RST GND VIN
     CLI          +-+ +-+ +-+ +-+ +-+ +-+
                  CLK DAT LLED NEO SDA SCK

Pinout on Arduino Mega              

  horizontal connectors
  
     CLI +++ +++ +++ +++ +++    +++ +++ +++ +++ +++ DCC  TX  RX    +++ +++ +++ +++ +++ +++ +++ +++
     D13 D12 D11 D10  D9  D8     D7  D6  D5  D4  D3  D2  D1  D0    D14 D15 D16 D17 D18 D19 D20 D21
    ####################################################################################################### 
    USB############################################TOP#view################################################ 
    #######################################################################################################  
    #######################################################################################################  
     x   x   x   x   x   x   x   x      A0  A1  A2  A3  A4  A5  A6 A7     A8  A9 A10 A11 A12 A13 A14 A15
                                       +-+ +-+ +-+ +-+ +-+ +-+ +++ +++   +++ +++ +++ +++ +++ +++ +++ +++
                                       CLK DAT LLED NEO SDA SCK

  vertical connector:
  
         GND ##### GND
     +++ D22 ##### D23 +++      +++ = out
     +++ D24 ##### D25 +++      out when not used by MAX,PCA,NEO
     +++ D26 ##### D27 +++      use following numbers in SW: 3-12,14-53 & A0-A15 
     +++ D28 ##### D29 +++      max out = 66
     +++ D30 ##### D31 +++
     +++ D32 ##### D33 +++
     +++ D34 ##### D35 +++
     +++ D36 ##### D37 +++
     +++ D38 ##### D39 +++
     +++ D40 ##### D41 +++
     +++ D42 ##### D43 +++
     +++ D44 ##### D45 +++
     +++ D46 ##### D47 +++
     +++ D48 ##### D49 +++
     +++ D50 ##### D51 +++
     +++ D52 ##### D53 +++
         GND ##### GND
 
 
*/


//==============================================================================================
//     USER PART
//==============================================================================================

// functions available for user
void user_out(byte num, byte val);
void user_led(byte num, byte val);
void user_led_cli(byte num, byte val);
void user_led_pha(byte num, byte val);
void user_neo(byte num, byte r, byte v, byte b);
void user_pwm_0_100(byte num, byte val);
void user_servo_500_2500(byte num, unsigned int val);
void user_servo_speed(byte num, byte speed);   //delta pulse us / 125ms 
void user_tempo_start(byte num_tempo, unsigned int duration_ms);
void user_blink(void); 

// PIN assignation
#define USER_USE_MAX  0   // 1 to use MAX7219/21 (out A0, A1 & A2 no more available)
#define USER_USE_PCA  0   // 1 to use PCA9685 (out A3 & A4 no more available)
#define USER_USE_NEO  0   // 1 to use neopixels (out A5 no more available)

// User defines
#define DECODER_ADR_1 10  // first 4 pairs
#define DECODER_ADR_2 11  // last  4 pairs

void user_init(void)
{
}

void user_notify_bas_acc_dec(unsigned int adr_1_510, byte out_0_7, byte val_0_1)
{
    if(adr_1_510 == DECODER_ADR_1)
    {
        if(out_0_7 == 0) { user_out(12, val_0_1); user_out(11, 0); }
        if(out_0_7 == 1) { user_out(11, val_0_1); user_out(12, 0); }
        if(out_0_7 == 2) { user_out(10, val_0_1); user_out( 9, 0); }
        if(out_0_7 == 3) { user_out( 9, val_0_1); user_out(10, 0); }
        if(out_0_7 == 4) { user_out( 8, val_0_1); user_out( 7, 0); }
        if(out_0_7 == 5) { user_out( 7, val_0_1); user_out( 8, 0); }
        if(out_0_7 == 6) { user_out( 6, val_0_1); user_out( 5, 0); }
        if(out_0_7 == 7) { user_out( 5, val_0_1); user_out( 6, 0); }
    }
    if(adr_1_510==DECODER_ADR_2)
    {
        if(out_0_7 == 0) { user_out( 4, val_0_1); user_out( 3, 0); }
        if(out_0_7 == 1) { user_out( 3, val_0_1); user_out( 4, 0); }
        if(out_0_7 == 2) { user_out(A0, val_0_1); user_out(A1, 0); }
        if(out_0_7 == 3) { user_out(A1, val_0_1); user_out(A0, 0); }
        if(out_0_7 == 4) { user_out(A2, val_0_1); user_out(A3, 0); }
        if(out_0_7 == 5) { user_out(A3, val_0_1); user_out(A2, 0); }
        if(out_0_7 == 6) { user_out(A4, val_0_1); user_out(A5, 0); }
        if(out_0_7 == 7) { user_out(A5, val_0_1); user_out(A4, 0); }
    }
}

void user_notify_ext_acc_dec(unsigned int adr_1_2044, byte val_0_31)
{
}

void user_notify_tempo_end(byte num_tempo)
{
}

void user_125ms()
{
}



//==============================================================================================
// IO
//==============================================================================================

#define PIN_LED  13
#define PIN_DCC   2

#define CLK_PIN  A0
#define DAT_PIN  A1
#define LLED_PIN A2
#define NEO_PIN  A3
#define SDA_PIN  A4
#define SCK_PIN  A5



//==============================================================================================
// Leds (256)
//==============================================================================================

#define IO_LED_NB 256

byte io_led[IO_LED_NB / 8] = { 0, 0, 0, 0 ,0, 0, 0, 0,    0, 0, 0, 0 ,0, 0, 0, 0 };  // possibilite d'initialiser les valeurs
byte io_cli[IO_LED_NB / 8] = { 0, 0, 0, 0 ,0, 0, 0, 0,    0, 0, 0, 0 ,0, 0, 0, 0 };  // ex led clignotante
byte io_pha[IO_LED_NB / 8] = { 0, 0, 0, 0 ,0, 0, 0, 0,    0, 0, 0, 0 ,0, 0, 0, 0 };
byte led_cpt = 0;

void user_led(byte num, byte val)   //on
{
    byte msk;

    if(num > IO_LED_NB) return;
    msk = 1 << (num & 7);
    if(val) io_led[num / 8] |= msk;
    else    io_led[num / 8] &= (0xff ^ msk);
}

void user_led_cli(byte num, byte val)
{
    byte msk;

    if(num > IO_LED_NB) return;
    msk = 1 << (num & 7);
    if(val) io_cli[num / 8] |= msk;
    else    io_cli[num / 8] &= (0xff ^ msk);
}

void user_led_pha(byte num, byte val)
{
    byte msk;
    
    if(num > IO_LED_NB) return;
    msk = 1 << (num & 7);
    if(val) io_pha[num / 8] |= msk;
    else    io_pha[num / 8] &= (0xff ^ msk);
}

void led_shift8(unsigned char dat)
{
    unsigned char i;

    pinMode(DAT_PIN, OUTPUT);

    for(i = 0; i < 8; i++)
    {
         if(dat & 0x80) digitalWrite(DAT_PIN, HIGH);
         else           digitalWrite(DAT_PIN, LOW);
             
         delayMicroseconds(4);
         digitalWrite(CLK_PIN, HIGH);
         delayMicroseconds(4);  //periode=8us = 125KHz (ok sur longue distances)
         digitalWrite(CLK_PIN, LOW);
         
         dat = dat<<1;
    }
}

void led_reg(unsigned char reg0, unsigned char dat0, unsigned char reg1, unsigned char dat1, unsigned char reg2, unsigned char dat2, unsigned char reg3, unsigned char dat3)
{
     digitalWrite(CLK_PIN, LOW);
     digitalWrite(LLED_PIN, HIGH); delayMicroseconds(10); digitalWrite(LLED_PIN, LOW); delayMicroseconds(10);
     led_shift8(reg3); led_shift8(dat3);
     led_shift8(reg2); led_shift8(dat2);
     led_shift8(reg1); led_shift8(dat1);
     led_shift8(reg0); led_shift8(dat0);
     digitalWrite(LLED_PIN, HIGH); delayMicroseconds(10); 
}

byte compute_led_on_cli_phase(byte on, byte cli, byte phase)
{
    byte led;

    if(led_cpt >=  4) led = 0xff;  else led = 0;  //clignotement 1Hz
    led ^= phase;                                 //changement de phase
    led |= ~cli;                                  //allumer si pas clignotant
    led &= on;                                    //eteindre si pas on

    return led;
}

// needs about 2x 2.5ms to update all LEDs
void led_maj(byte index)
{
    byte i, j, dat0, dat1, dat2, dat3;

    if(index == 0) for(i = 0; i < 4; i++)
    {
        dat0 = compute_led_on_cli_phase(io_led[     i], io_cli[     i], io_pha[     i]);
        dat1 = compute_led_on_cli_phase(io_led[ 8 + i], io_cli[ 8 + i], io_pha[ 8 + i]);
        dat2 = compute_led_on_cli_phase(io_led[16 + i], io_cli[16 + i], io_pha[16 + i]);
        dat3 = compute_led_on_cli_phase(io_led[24 + i], io_cli[24 + i], io_pha[24 + i]);
        led_reg(1 + i, dat0, 1 + i, dat1, 1 + i, dat2, 1 + i, dat3);
    }
    
    if(index == 1) for(i = 4; i < 8; i++)
    {
        dat0 = compute_led_on_cli_phase(io_led[     i], io_cli[     i], io_pha[     i]);
        dat1 = compute_led_on_cli_phase(io_led[ 8 + i], io_cli[ 8 + i], io_pha[ 8 + i]);
        dat2 = compute_led_on_cli_phase(io_led[16 + i], io_cli[16 + i], io_pha[16 + i]);
        dat3 = compute_led_on_cli_phase(io_led[24 + i], io_cli[24 + i], io_pha[24 + i]);
        led_reg(1 + i, dat0, 1 + i, dat1, 1 + i, dat2, 1 + i, dat3);
    }

}

void led_init(void)
{
                                              // R0    : bypass
                                              // R1-8  : data
    led_reg(9  ,  0,  9,  0,  9,  0,  9,  0); // R9    : decode mode off (1ere fois pas sure que ca marche) 
    led_reg(9  ,  0,  9,  0,  9,  0,  9,  0); // R9    : decode mode off
    led_reg(0xa,0x4,0xa,0x4,0xa,0x4,0xa,0x4); // R10   : intensite 4/15 (0xf=max)
    led_reg(0xb,  7,0xb,  7,0xb,  7,0xb,  7); // R11   : 8 digit    
    led_reg(0xc,  1,0xc,  1,0xc,  1,0xc,  1); // R12   :shutdown mode off
                                              // R13-14: NA
    led_reg(0xf,  0,0xf,  0,0xf,  0,0xf,  0); // R15   :test mode off (1=on)

    led_maj(0); 
    led_maj(1); 
}



//==============================================================================================
// NEOPIXELS (60)
//==============================================================================================

/*
neopixel
official ns
H0  200-500  L0  650-950
H1  550-850  L1  450-750
RESET 50000 min
test
H0 200-500
H1 550-oo   L 450-5000
RESET 6000 min
G(8bits7-->0)R(8bits)B(8bits)  
*/

#define NEO_NB 60
#define NEO_R 0
#define NEO_V 1
#define NEO_B 2
volatile unsigned char neo_rvb[NEO_NB][3];
volatile byte neo_to_be_updated = 0;

#define NOP __asm__ __volatile__ ("nop\n\t")    //62.5ns

/*
    Addr    Name     Bit7   Bit6   Bit5   Bit4   Bit3   Bit2   Bit1   Bit0
    ...
    0x03   PINB     PINB7  PINB6  PINB5  PINB4  PINB3  PINB2  PINB1  PINB0
    0x04   DDRB     DDB7   DDB6   DDB5   DDB4   DDB3   DDB2   DDB1   DDB0
    0x05   PORTB    PORTB7 PORTB6 PORTB5 PORTB4 PORTB3 PORTB2 PORTB1 PORTB0
    0x06   PINC     x      PINC6  PINC5  PINC4  PINC3  PINC2  PINC1  PINC0
    0x07   DDRC     x      DDC6   DDC5   DDC4   DDC3   DDC2   DDC1   DDC0
    0x08   PORTC    x      PORTC6 PORTC5 PORTC4 PORTC3 PORTC2 PORTC1 PORTC0
    0x09   PIND     PIND7  PIND6  PIND5  PIND4  PIND3  PIND2  PIND1  PIND0
    0x0A   DDRD     DDD7   DDD6   DDD5   DDD4   DDD3   DDD2   DDD1   DDD0
    0x0B   PORTD    PORTD7 PORTD6 PORTD5 PORTD4 PORTD3 PORTD2 PORTD1 PORTD0 
*/

void neo_reset(void)  //6 us mini  -> 10us
{
    unsigned char i;
    
    __asm__ __volatile__ ("cbi 8, 3\n\t");  // 8,3 = PORTC bit3 = AN3    ou 5,5=D13
    for(i = 0; i < 100; i++)  // par bcl: 3cycle+nb nop   1nop -> 4*62.5ns = 250ns   01us=40bcl
    {
      //__asm__ __volatile__("nop\n\t"); //62.5ns
      NOP;
    }
}

//  mesures
// 1court: 0.375us
// 1long : 0.750us
// bas   : 0.750-937us   1625us entre 2 bytes  3125us entre 2 24bits
void neo_tx_byte(unsigned char val) //UU: check the code of this function in assembly (mainly after opimization !!!!)
{
    unsigned char i;
    
    for(i = 0; i < 8; i++)
    {
        if(val & 0x80)   // TX_H_1   550-oo ns --> 600ns   10nop+1port=10*62.5=625ns  //uu:try to reduce
        {
            asm volatile(
            "sbi 8, 3\n\t"      //mise a 1 AN3 PC3
            "nop\n\t"           //9x NOP
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "cbi 8, 3\n\t"); //mise a 0 AN3 PC3
        } 
        else   // TX_H_0  200-500ns --> 400ns 4nop+port=5*62.5=310ns
        {
            asm volatile(
            "sbi 8, 3\n\t"   //mise a 1 digital 13 PB5
            "nop\n\t"           //4x NOP
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "cbi 8, 3\n\t"); //mise a 0 digital 13 PB5
        }
        // TX_L  450->5000ns --> 600ns      sans les nop:500ns, 4nop: +248ns -> 1us  //uu: try tu reduce      
        asm volatile(
        "nop\n\t"
        "nop\n\t"
        "nop\n\t"
        "nop\n\t"); 
        val = val << 1;
    }  
}

// dure 60 * 24* 1.5us = 2.1ms max pour 60 neopixels
// desactive les ITs 
void neo_display(void)
{
    unsigned char i;
    
    //wait an end of packet
    //loco_pkt = 0;
    //while(loco_pkt == 0) { process_packet(); }
    noInterrupts();

    neo_reset();
    
    for(i = 0; i < NEO_NB; i++)
    {
        neo_tx_byte(neo_rvb[i][NEO_V]);    
        neo_tx_byte(neo_rvb[i][NEO_R]);    
        neo_tx_byte(neo_rvb[i][NEO_B]);    
    }
    
    neo_reset();
    
    interrupts();
    Serial.print("$");
}

void neo_set_rvb_pixel(unsigned char pixel, unsigned char r, unsigned char v, unsigned char b)
{
    neo_rvb[pixel][NEO_R] = r;
    neo_rvb[pixel][NEO_V] = v;
    neo_rvb[pixel][NEO_B] = b;
}

void neo_init(void)
{
    byte i;
    
    for(i = 0; i < NEO_NB; i++)
    {
        neo_set_rvb_pixel(i, 0, 0, 0);  //on demarre a 0 pour ne pas trop charger une faible alimentation
    }
    neo_display();  
    neo_display();
}

void neo_maj(void)
{
    neo_to_be_updated = 1;
    //neo_display();  
}

void user_neo(byte num, byte r, byte v, byte b)
{
    if(num <= NEO_NB) neo_set_rvb_pixel(num, r, v, b);
}


//==============================================================================================
// I2C (I2C fait a la main pour pouvoir utiliser une basse frequence et n'importe quelle patte)
//==============================================================================================

//on emule un collecteur ouvert en mettant les sorties a 0 et en jouant sur la direction.
#define SDA_1 pinMode(SDA_PIN, INPUT)
#define SDA_0 pinMode(SDA_PIN, OUTPUT)
#define SCK_1 pinMode(SCK_PIN, INPUT)
#define SCK_0 pinMode(SCK_PIN, OUTPUT)

void i2c_init(void)
{
    pinMode(SCK_PIN, INPUT); digitalWrite(SCK_PIN, LOW);
    pinMode(SDA_PIN, INPUT); digitalWrite(SDA_PIN, LOW); 
}

void i2c_start(void)
{
    SDA_1; SCK_1; delayMicroseconds(5);  
    SDA_0; delayMicroseconds(5);
    SCK_0; delayMicroseconds(5);  
}

void i2c_stop(void)
{
    SDA_0; SCK_0; delayMicroseconds(5);  
    SCK_1; delayMicroseconds(5);  
    SDA_1; delayMicroseconds(5);  
}

void i2c_wr_8(byte data)
{
    byte i;

    for(i = 0; i < 8; i++)
    {
        if(data & 0x80) { SDA_1; } else { SDA_0; } delayMicroseconds(4);  
        SCK_1; delayMicroseconds(4);  
        SCK_0; delayMicroseconds(1);  
        data = data << 1;        
    }
    SDA_1; delayMicroseconds(1);  
}

byte i2c_rd_ack(void)
{
    byte ack;

    delayMicroseconds(4);  //before 0
    SCK_1; delayMicroseconds(4);  
    if(digitalRead(SDA_PIN) == HIGH) ack = 0; /*nack*/ else ack=1; /*ack*/
    SCK_0; delayMicroseconds(1);  
    return ack;
}

byte i2c_rd_8(void)
{
    byte i, data;

    data=0;
    for(i=0;i<8;i++)
    {
        delayMicroseconds(4);  //before 0
        SCK_1; delayMicroseconds(4);  
        if(digitalRead(SDA_PIN) == HIGH) data |= 1;  
        SCK_0; delayMicroseconds(1);  
        data = data << 1;        
    }
    return data;  
}

byte i2c_wr_ack(byte ack)   //1=ack, 0=nack
{
    if(ack) { SDA_0; } else { SDA_1; } delayMicroseconds(4);  
    SCK_1; delayMicroseconds(4);  
    SCK_0; delayMicroseconds(1);  
}



//==============================================================================================
//     PCA9685 (3x16=48)
//==============================================================================================

#define PCA9685_MODE1_REG    0x00
#define PCA9685_MODE2_REG    0x01
#define PCA9685_LED0_REG     0x06
#define PCA9685_PRESCALE_REG 0xfe

#define IO_PCA9685_NB 6
#define IO_PCA9685_OUT_NB (16 * IO_PCA9685_NB)

byte pca9685_tx_srv[ IO_PCA9685_OUT_NB];  // bit0=tx  bit1=srv
unsigned int pca9685_on[ IO_PCA9685_OUT_NB];
unsigned int pca9685_off[IO_PCA9685_OUT_NB];
unsigned int pca9685_off_target[IO_PCA9685_OUT_NB];
byte srv_speed[IO_PCA9685_OUT_NB];


byte set_pca9685_reg(byte i2c_adr, byte adr, byte dat)
{
    i2c_start();
    i2c_wr_8(0x80 + 2 * i2c_adr);  //A6=1,A5-A0=pin,R/W#
    if(i2c_rd_ack() == 0)
    {
        i2c_stop();  
        return 0;
    }
    i2c_wr_8(adr);    
    if(i2c_rd_ack() == 0)
    {
        i2c_stop();  
        return 0;
    }
    i2c_wr_8(dat);    
    if(i2c_rd_ack() == 0)
    {
        i2c_stop();  
        return 0;
    }
    i2c_stop();  
    return 1;
}


byte set_pca9685_on_off(byte i2c_adr, byte out, unsigned int on, unsigned int off)
{
    byte on_h, on_l, off_h, off_l;
    
    on_h  = byte(on  >> 8); on_l  = byte(on  & 255);  //existe aussi lowByte highByte
    off_h = byte(off >> 8); off_l = byte(off & 255); 

    i2c_start();  // A6=1,A5-A0=pin,R/W#
    i2c_wr_8(0x80+2*i2c_adr);          i2c_rd_ack();
    i2c_wr_8(PCA9685_LED0_REG+4*out);  i2c_rd_ack();
    i2c_wr_8(on_l);                    i2c_rd_ack();
    i2c_wr_8(on_h);                    i2c_rd_ack();
    i2c_wr_8(off_l);                   i2c_rd_ack();
    i2c_wr_8(off_h);                   i2c_rd_ack();
    i2c_stop();
}


byte set_pca9685_only_off_out(byte i2c_adr, byte out, unsigned int off)
{
    byte on_h, on_l, off_h, off_l;
    
    off_h = byte(off >> 8); off_l = byte(off & 255); 

    i2c_start();  // A6=1,A5-A0=pin,R/W#
    i2c_wr_8(0x80 + 2 * i2c_adr);              i2c_rd_ack();
    i2c_wr_8(PCA9685_LED0_REG + 4 * out + 2);  i2c_rd_ack();
    i2c_wr_8(off_l);                           i2c_rd_ack();
    i2c_wr_8(off_h);                           i2c_rd_ack();
    i2c_stop();
}


// to set directly pwm (just used at init)
void set_pca9685_pwm0_100(byte num, byte pwm)
{
    byte i2c_adr;
    byte out;
    unsigned int off;
    
    i2c_adr = num / 16; 
    out = num & 15;
    off = map(pwm, 0, 100, 0, 4096);

    if(off == 0)         set_pca9685_on_off(i2c_adr, out,    0, 4096);
    else if(off == 4096) set_pca9685_on_off(i2c_adr, out, 4096,    0);
    else                 set_pca9685_on_off(i2c_adr, out,    0, off );
}


// to set directly srv (no more used)
void set_paca9685_srv50_250(byte num, byte pulse)  //50 -> 150 -> 250
{
    byte i2c_adr;
    byte out;
    unsigned int off;
    
    i2c_adr = num / 16; 
    out =  num & 15;
    off = map(pulse, 50, 250, 102, 512);
    
    set_pca9685_on_off(i2c_adr, out, 0, off);
}


byte i2c_pca9685_online[IO_PCA9685_NB];

void pca9685_init(void)
{
    byte i;  
    byte i2c_adr;

    for(i2c_adr = 0; i2c_adr < IO_PCA9685_NB; i2c_adr++)
    {
        // reset et config MODE1
        i2c_pca9685_online[i2c_adr] = set_pca9685_reg(i2c_adr, PCA9685_MODE1_REG, 0xB0); //reset=1, extclk=0, autoincrement=1, sleep=1 (all leds are off after reset)
        if(i2c_pca9685_online[i2c_adr] == 0) return;
        delay(1);

        // Frequence de decoupage
        // 50Hz pour les servos: 25.000.000/(4096*50Hz) - 1 = 122-1 = 121.
        // 60Hz pour les servos: 25.000.000/(4096*60Hz) - 1 = 102-1 = 101.
        // mesure:
        // 121 -> 18.7ms     -1.3ms     err -6.5%
        // 130 -> 19.92ms    -0.08ms    err  0.4%
        // 131 -> 20.07ms    +0.07ms    err  0.4%
        set_pca9685_reg(i2c_adr, PCA9685_PRESCALE_REG, 130);

        // exit from sleep
        set_pca9685_reg(i2c_adr, PCA9685_MODE1_REG, 0x20);    //reset=0, extclk=0, autoincrement=1, sleep=0
        delay(1); //wait exit from low power mode

        // config MODE2
        set_pca9685_reg(i2c_adr, PCA9685_MODE2_REG, 0x04);     // invert=0 change=onSTOP out=totem off=led_0 UU: voir si pa smieux de clr sorties avant 
    }

    /* toutes les sorties a 0 */
    for(i = 0; i < IO_PCA9685_OUT_NB; i++) 
    {
        set_pca9685_pwm0_100(i, 0);
        pca9685_on[i] = 0;
        pca9685_off[i] = 4096;
        pca9685_tx_srv[i] = 0;        
        srv_speed[i] = 0;
    }
}


//variation lente des servos
byte pca9685_slow_srv()
{
    byte num;
    unsigned int off, off_target;

    for(num = 0; num < IO_PCA9685_OUT_NB; num++)
    {
        if(i2c_pca9685_online[num / 16] == 0) continue; // PCA9685 module is not present 

        if((pca9685_tx_srv[num] & 2) == 0) continue; // servos only
        
        if(pca9685_off_target[num] == pca9685_off[num]) continue;
        
        off_target = pca9685_off_target[num];
        off = pca9685_off[num];
        
        if(srv_speed[num]==0) 
        { 
            off = off_target;
        }
        else
        {
            if(off_target > off) { off += srv_speed[num]; if(off > off_target) off = off_target; }
            if(off_target < off) { off -= srv_speed[num]; if(off < off_target) off = off_target; }
        }
        if(pca9685_off[num] != off)
        {
            pca9685_off[num] = off;
            pca9685_tx_srv[num] |= 1;  //update
        }        
    }   
}


// return 1 when update is finished
byte pca9685_maj(byte index)
{
    byte i, num;
    byte i2c_adr;
    byte out;
    
    for(i = 0; i < 4; i++)
    {
        num = 4 * index + i;
        if(num >= IO_PCA9685_OUT_NB) return 1;       
        
        if(i2c_pca9685_online[num/16] == 0) continue;  // PCA9685 module is not present

        if(pca9685_tx_srv[num] & 1)
        {
            pca9685_tx_srv[num] &= 0xfe;

            i2c_adr = num / 16; 
            out = num & 15;

            set_pca9685_on_off(i2c_adr, out, pca9685_on[num], pca9685_off[num]);
        }
    }

    return 0;
}


void user_pwm_0_100(byte num, byte pwm)
{
    unsigned int off;
    
    //srv_on[num] = 0; //this pin is now a servo
    //if(pwm_inv[num]) pwm = 100 - pwm;

    if(num >= IO_PCA9685_OUT_NB) return;

    off = map(pwm, 0, 100, 0, 4096);

    if(off == 0)         { pca9685_on[num] = 0;    pca9685_off[num] = 4096; }
    else if(off == 4096) { pca9685_on[num] = 4096; pca9685_off[num] =    0; }
    else                 { pca9685_on[num] = 0;    pca9685_off[num] =  off; }

    pca9685_tx_srv[num] |= 1;     //update
    pca9685_tx_srv[num] &= 0xfd;  //not a servo
}


void user_servo_500_2500(byte num, unsigned int pulse)
{
    unsigned int off;

    if(num >= IO_PCA9685_OUT_NB) return;

    //4096=20ms > 102=0.5ms 512=2.5ms
    off = map(pulse, 500, 2500, 102, 512);

    pca9685_on[num] = 0;    pca9685_off_target[num] = off;

    pca9685_tx_srv[num] |= 2;  //is a servo (no update, done in pca9685_slow_srv)
}


void user_servo_speed(byte num, byte speed)
{
    if(num >= IO_PCA9685_OUT_NB) return;

    srv_speed[num] = speed; 
}


//==============================================================================================
//     DCC decoder engine part
//==============================================================================================

// remenber to use volatile when variable is shared between main and isr
volatile unsigned int actMicros, lastMicros, bitMicros;
volatile byte in_ext_irq = 0;
volatile byte last_bit_val = 0;
volatile byte preambule_state = 1;
volatile byte preambule_len = 0;
volatile byte bit_phase = 0;
volatile byte bit_nb = 0;
volatile byte byte_val = 0;
volatile byte dat[4]; //including xor
volatile byte dat_len = 0;
volatile byte skip_sep = 0;

volatile byte msg[4];
volatile byte msg_len = 0;
volatile byte msg_ok;

volatile byte err_preambule = 0;
volatile byte err_same      = 0;
volatile byte err_nested_it = 0;
volatile byte err_short     = 0;
volatile byte err_pkt_long  = 0;
volatile byte err_pkt_short = 0;
byte err_checksum  = 0;
byte dbg_pkt3 = 0;
byte dbg_pkt4 = 0;
byte dbg_pkt_idle = 0;

inline void process_bitval(byte bit_val)
{
    if(preambule_state)
    {
        if(bit_val) { preambule_len++; return; }
        
        if(preambule_len < 18) { preambule_len = 0; err_preambule++; return; }  //waiting at last 9 x "1" symbol
        
    preambule_len = 0;
        preambule_state = 0;
        bit_phase = 0;
        bit_nb = 0;
        dat_len = 0;
    skip_sep = 1;
        return;
    }
    
    bit_phase ^= 1;
  if((bit_phase & 1)==0) return;
  if(bit_val != last_bit_val) { preambule_state = 1; err_same++; return; }
    
  if(skip_sep) { skip_sep=0; return; } //skip the first 0 separator beetween preambule and first byte 

#if USER_USE_NEO == 1
    // To update, neopixels, we need to stop interrupts for sometime (ie:2.1ms for 60 neopixels)
    // During this time, decoder is not able to decode the DCC signal
    // Since this accessory decoder do not take care about loco packets, as soon as we detect the begining of a loco packet
    // we call neo_display() if an update was required by neo_to_be_updated.
    // loco packet will last about 4ms, so we will be ready to sync on the following preambule
    // Note this ughly method causes some errors in statistics
    if(neo_to_be_updated) if(dat_len == 0) if(bit_nb == 0) if(bit_val == 0)
    {
        neo_display();
        //Serial.print("£");
        neo_to_be_updated = 0;
        preambule_state = 1;
        return; 
    }
#endif  
    
    byte_val = (byte_val << 1) | bit_val;
    bit_nb++;
    
    if(bit_nb < 8) return;

    if(bit_nb == 8) { dat[dat_len] = byte_val; dat_len++; return; }
    
    // bit == 9 
    if(bit_val == 0)
    {
        bit_nb = 0;
        if(dat_len >= 4) { err_pkt_long++; preambule_state = 1; }
        return;
    }

    msg_ok = 1;
    msg_len = dat_len;
    msg[0] = dat[0];          
    msg[1] = dat[1];          
    msg[2] = dat[2];          
    msg[3] = dat[3];          
    preambule_state = 1;  
    return;
}

void ExternalInterruptHandler(void)
{
    byte bit_val = 0;

    if(in_ext_irq) { err_nested_it++; return; } //nested IT
    actMicros = micros();
    bitMicros = actMicros - lastMicros;
    if (bitMicros < 35) { err_short++; return; } //glitch
    lastMicros = actMicros;
    in_ext_irq = 1;
    interrupts(); //to not block micros
    if(bitMicros < 82) bit_val = 1; else bit_val=0;
    process_bitval(bit_val);
    last_bit_val = bit_val;
    in_ext_irq = 0;
    return;
}

void print_err(void);

// IDLE     : 1111111111 0 11111111 0 00000000            0 EEEEEEEE 1
//
// VIT  28  : 1111111111 0 0AAAAAAA 0 01DFSSSS            0 EEEEEEEE 1
// VIT 127  : 1111111111 0 0AAAAAAA 0 00111111            0 DSSSSSSS 0 EEEEEEEE 1
//
// FCT 0-4  : 1111111111 0 0AAAAAAA 0 100-FL-F4-F3-F2-F1  0 EEEEEEEE 1
// FCT 5-8  : 1111111111 0 0AAAAAAA 0 1011-F8-F7-F6-F5    0 EEEEEEEE 1 
// FCT 9-12 : 1111111111 0 0AAAAAAA 0 1010-F12-F11-F10-F9 0 EEEEEEEE 1 
// FCT 13-20: 1111111111 0 0AAAAAAA 0 11011110            0 F20-F19-F18-F17-F16-F15-F14-F13 0 EEEEEEEE 1 
// FCT 11-28: 1111111111 0 0AAAAAAA 0 11011111            0 F28-F27-F26-F25-F24-F23-F22-F21 0 EEEEEEEE 1 
//
// ACC 1-511: 1111111111 0 10AAAAAA 0 1AAA1DDD            0 EEEEEEEE 1 // decodeur d'accessoire attention, les 3xAAA sont inverses, ils forment aussi le MSB adr0 est reserve le 1 entre AetD peut etre mis a 0 pour mettre la sortie a 0, mais normalement allumer la 0 eteint la 1 et inversement. ou alors remise a 0 automatique apres une pulse
// ACC1-2044: 1111111111 0 10AAAAAA 0 0AAA0AA1 0 000DDDDD 0 EEEEEEEE 1 // decodeur de sortie 32 valeurs pour chaque sorties (ex: signal multi aspects, servos, pwm ...)
// une explication accessible sur le DCC est dispo sur: http://trainminiature.discutforum.com/t12784-dcc-comment-ca-marche-place-a-la-technique

void notify_dcc_acc(unsigned int adr_1_511, byte out_0_7, byte state_0_1)
{
    Serial.print(" BAS ACC pkt adr(1-511)="); Serial.print(adr_1_511);  
    Serial.print(" out(0-7)=");           Serial.print(out_0_7);  
    Serial.print(" val(0-1)=");           Serial.println(state_0_1);  

    user_notify_bas_acc_dec(adr_1_511, out_0_7, state_0_1); 
}

void notify_dcc_ext(unsigned int out_1_2044, byte val_0_31)
{
    Serial.print(" EXT ACC pkt out(1-2044)="); Serial.print(out_1_2044);  
    Serial.print(" val(0-31)=");           Serial.println(val_0_31);    

    user_notify_ext_acc_dec(out_1_2044, val_0_31);
}

void process_packet(void)  //to call at least each 5ms
{
    unsigned int adr_1_511;
    byte out_0_7;
    byte state_0_1;
    unsigned int out_1_2044;
    byte val_0_31;
  
    if(msg_ok == 0) return;
    msg_ok = 0;
  
    Serial.print("#"); Serial.print(msg_len);
    //print_err();
    
    if(msg_len < 3) { err_pkt_short++; return; }
    
    Serial.print(" "); Serial.print(msg[0]);
    Serial.print(" "); Serial.print(msg[1]);
    Serial.print(" "); Serial.print(msg[2]);

    if(msg_len == 3)
    {
        if(msg[0] ^ msg[1] ^ msg[2]) { err_checksum++; return; }
        dbg_pkt3++;

        // IDLE: 1111111111 0 11111111 0 00000000 0 EEEEEEEE 1
        if(msg[0] == 0xff && msg[1] == 0x00) 
        {
            dbg_pkt_idle++;
        }

        // ACC 1-511: 1111111111 0 10AAAAAA 0 1AAA1DDD 0 EEEEEEEE 1
        else if((msg[0] & 0xC0) == 0x80)
        {
            adr_1_511 = (((unsigned int)((msg[1] & 0x70) ^ 0x70)) << 2) | msg[0] & 0x3f;
            out_0_7   = msg[1] & 7;
            state_0_1 = (msg[1] >> 3) & 1;

            if(adr_1_511) notify_dcc_acc(adr_1_511, out_0_7, state_0_1);
        }

        Serial.println("");
        return;
    }

    Serial.print(" "); Serial.println(msg[3]);
    
    if(msg_len == 4)
    {
        // msg_len == 4
        if(msg[0] ^ msg[1] ^ msg[2] ^ msg[3]) { err_checksum++; return; }
        dbg_pkt4++;

        // ACC1-2044: 1111111111 0 10AAAAAA 0 0AAA0AA1 0 000DDDDD 0 EEEEEEEE 1 // decodeur de sortie 32 valeurs pour chaque sorties (ex: signal multi aspects, servos, pwm ...)
        if((msg[0] & 0xC0) == 0x80 && (msg[1] & 0b10001001) == 1 && (msg[2] & 0xE0) == 0)
        {
            adr_1_511 = (((unsigned int)((msg[1] & 0x70) ^ 0x70)) << 2) | msg[0] & 0x3f;
            // OutputAddress = (((BoardAddress - 1) << 2 ) | TurnoutPairIndex) + 1 ;
            // 1-511
            // 0-2040   +0..3  + 1    1-2044
            out_1_2044 = (((adr_1_511 - 1) << 2) | ((msg[1] >> 1) & 3)) + 1 ;
            val_0_31 = msg[2] & 0x1F;
            notify_dcc_ext(out_1_2044, val_0_31);   
        }
      
        Serial.println("");
        return;
    }
}

void print_err(void)
{
    Serial.print("err_nested_it = "); Serial.println(err_nested_it);
    Serial.print("err_short     = "); Serial.println(err_short);
    Serial.print("err_preambule = "); Serial.println(err_preambule);
    Serial.print("err_same      = "); Serial.println(err_same);
    Serial.print("err_pkt_long  = "); Serial.println(err_pkt_long);
    Serial.print("err_pkt_short = "); Serial.println(err_pkt_short);
    Serial.print("err_checksum  = "); Serial.println(err_checksum);  
    Serial.print("dbg_pkt3      = "); Serial.println(dbg_pkt3);
    Serial.print("dbg_pkt4      = "); Serial.println(dbg_pkt4);   
    Serial.print("dbg_pkt_idle  = "); Serial.println(dbg_pkt_idle);
}




//==============================================================================================
//     MAIN part
//==============================================================================================

byte wait_20ms = 0;  // 20ms flag to decrement pulse dcpt
byte wait_125ms = 0; // 125ms flag to update MAX & PCA
byte rx = 0; // rx indicates packet matching

void user_blink(void)
{
    rx = 16;  // * 125ms = 2sec
}

#define TEMPO_NB 80  //mettre un multiple de 8
unsigned int tempo_dcpt[TEMPO_NB];
byte tempo_flag[TEMPO_NB / 8];


void user_tempo_start(byte num_tempo, unsigned int duration_ms)
{
    if(num_tempo >= TEMPO_NB) return;

    tempo_dcpt[num_tempo] = duration_ms / 125; // 125ms increment

    if(duration_ms) tempo_flag[num_tempo / 8] |= (1 << (num_tempo & 7));
    else            tempo_flag[num_tempo / 8] &= (0xff ^ (1 << (num_tempo & 7)));
}


void tempo_maj(void)
{
    byte i, j, num;

    for(j = 0; j < (TEMPO_NB / 8); j++) if(tempo_flag[j])
    for(i = 0; i < 8; i++) if(tempo_dcpt[i])
    {
        num = 8*j + i;
        tempo_dcpt[num]--;
        if(tempo_dcpt[num] == 0) 
        { 
            tempo_flag[num / 8] &= (0xff ^ (1 << (num & 7)));
            user_notify_tempo_end(num);
        }
    }
    if(rx) rx--;
}


void user_out(byte num, byte val)
{
#if defined(__AVR_ATmega2560__)    //in arduino lib Dmax is before A0, so there is no output after Amax
    if(num > A15) return;
#else
    if(num > A5) return;
#endif
    
    if(num <= 2) return;
    if(num == 13) return;
    
#if USER_USE_MAX == 1
    if(num >= A0 && num <= A2) return;
#endif
#if USER_USE_NEO == 1
    if(num == A3) return;
#endif    
#if USER_USE_PCA == 1
    if(num == A4 || num == A5) return;
#endif    
   
    if(val) digitalWrite(num, HIGH);
    else    digitalWrite(num, LOW);
}


unsigned long time0;

void setup(void)
{
    // Serial output for debugging
    Serial.begin(115200);
    Serial.println("D18 DCC Accessory Decoder v20180208a");

    // init NmraDcc library (PIN, manufacturer, version...) 
    //Dcc.pin(digitalPinToInterrupt(PIN_DCC), PIN_DCC, 1);
    //Dcc.init(MAN_ID_DIY, 1, FLAGS_DCC_ACCESSORY_DECODER, 0);

    // configure pins
    // D13-2-1-0 not in the list because D13 is led, D2 DCC in, D1 TX, D0 RX
    pinMode(12, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(8, OUTPUT);
  
    pinMode(7, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(3, OUTPUT);

    //on Nano there is also A6 & A7 but they cannot be used as output 
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
    pinMode(A4, OUTPUT);
    pinMode(A5, OUTPUT);

    #if defined(__AVR_ATmega2560__)
        pinMode(14, OUTPUT);
        pinMode(15, OUTPUT);
        pinMode(16, OUTPUT);
        pinMode(17, OUTPUT);
        pinMode(18, OUTPUT);
        pinMode(19, OUTPUT);
        pinMode(20, OUTPUT);
        pinMode(21, OUTPUT);

        pinMode(22, OUTPUT);
        pinMode(23, OUTPUT);
        pinMode(24, OUTPUT);
        pinMode(25, OUTPUT);
        pinMode(26, OUTPUT);
        pinMode(27, OUTPUT);
        pinMode(28, OUTPUT);
        pinMode(29, OUTPUT);

        pinMode(30, OUTPUT);
        pinMode(31, OUTPUT);
        pinMode(32, OUTPUT);
        pinMode(33, OUTPUT);
        pinMode(34, OUTPUT);
        pinMode(35, OUTPUT);
        pinMode(36, OUTPUT);
        pinMode(37, OUTPUT);

        pinMode(38, OUTPUT);
        pinMode(39, OUTPUT);
        pinMode(40, OUTPUT);
        pinMode(41, OUTPUT);
        pinMode(42, OUTPUT);
        pinMode(43, OUTPUT);
        pinMode(44, OUTPUT);
        pinMode(45, OUTPUT);

        pinMode(46, OUTPUT);
        pinMode(47, OUTPUT);
        pinMode(48, OUTPUT);
        pinMode(49, OUTPUT);
        pinMode(50, OUTPUT);
        pinMode(51, OUTPUT);
        pinMode(52, OUTPUT);
        pinMode(53, OUTPUT);

        pinMode(A6, OUTPUT);
        pinMode(A7, OUTPUT);
  
        pinMode(A8, OUTPUT);
        pinMode(A9, OUTPUT);
        pinMode(A10, OUTPUT);
        pinMode(A11, OUTPUT);
        pinMode(A12, OUTPUT);
        pinMode(A13, OUTPUT);
        pinMode(A14, OUTPUT);
        pinMode(A15, OUTPUT);
    #endif
  
    pinMode(PIN_LED, OUTPUT);

    #if USER_USE_MAX == 1
        led_init();
    #endif
    #if USER_USE_PCA == 1  
        i2c_init();
        pca9685_init();
    #endif  
    #if USER_USE_NEO == 1
        neo_init();
    #endif  

    user_init();
    #if USER_USE_NEO == 1
        neo_display();
    #endif

    pinMode(PIN_DCC, INPUT);
    digitalWrite(PIN_DCC, HIGH); //enable pull-up
    attachInterrupt( digitalPinToInterrupt(PIN_DCC), ExternalInterruptHandler, CHANGE);  //on UNO: IT 0 = ExtInt Pin 2  or use digitalPinToInterrupt(2)

    time0 = millis();
}

byte index = 0;
byte cpt_125ms = 0;

void fast_loop(void)
{
    process_packet();   //A appeller au moins toutes les 5ms pour ne pas manquer de packets
}

void loop(void) 
{
    unsigned long m = millis();
    unsigned long time1;
    
   
    fast_loop();

    // attente 125ms    
    time1 = millis();
    if((time1 - time0) < 125) return;

    // on arrive ici toute les 125ms      
    //Serial.println(time1 - time0);    
    time0 = time1;
    cpt_125ms++;

    // gestion des tempos   
    tempo_maj();  

    // tache de fond de l'utilisateur
    user_125ms();

    // clignotement de la LED a 1Hz  (ou 4Hz pendant 2s lorsque l'on recoit un packet)
    if(rx == 0) { if(cpt_125ms & 4) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW); }
    else        { if(cpt_125ms & 1) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW); }

    // mise a jour des LEDs
    #if USER_USE_MAX == 1
        led_cpt++; if(led_cpt == 8) led_cpt=0;  // 0 - 7 used to blink leds (off 0-3, on 4-7)
        led_maj(0); fast_loop(); // total led_maj lasts 5ms, so we cut it to call process_packet() between;
        led_maj(1); fast_loop();            
    #endif
        
    // mise a jour des sorties PWM et servos
    #if USER_USE_PCA == 1
        pca9685_slow_srv(); // compute positions for servos using slow movements
        index = 0;
        while(1) // total pca9685_maj lasts between 0 and 60ms, so we cut it to call process_packet() between
        {
            if(pca9685_maj(index)) break;  //return 1 when nothing else to update
            fast_loop();                       
            index++;              
        }            
    #endif
        
    // demande de mise a jour des neopixels
    #if USER_USE_NEO == 1
        neo_maj(); 
    #endif
}
