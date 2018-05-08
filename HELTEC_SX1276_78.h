// HELTEC_SX1276.h
//
// Definitions pour module HELTEC LoRa WiFi
// Author: Robert PRE
// Copyright (C) 2018
// 

#ifndef HELTEC_SX1276_78
#define HELTEC_SX1276_78

#include <RHSPIDriver.h>

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define SX1276_NUM_INTERRUPTS 3

// Max number of octets the LORA Rx/Tx FIFO can hold
#define SX1276_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define SX1276_MAX_PAYLOAD_LEN SX1276_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define SX1276_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef SX1276_MAX_MESSAGE_LEN
 #define SX1276_MAX_MESSAGE_LEN (SX1276_MAX_PAYLOAD_LEN - SX1276_HEADER_LEN)
#endif

// The crystal oscillator frequency of the module
#define SX1276_FXOSC  32000000.0

// The Frequency Synthesizer step = SX1276_FXOSC / 2^^19
#define SX1276_FSTEP  (SX1276_FXOSC / 524288)


// Register names (LoRa Mode, from table 85)
#define SX1276_REG_00_FIFO                                0x00
#define SX1276_REG_01_OP_MODE                             0x01
#define SX1276_REG_02_RESERVED                            0x02
#define SX1276_REG_03_RESERVED                            0x03
#define SX1276_REG_04_RESERVED                            0x04
#define SX1276_REG_05_RESERVED                            0x05
#define SX1276_REG_06_FRF_MSB                             0x06
#define SX1276_REG_07_FRF_MID                             0x07
#define SX1276_REG_08_FRF_LSB                             0x08
#define SX1276_REG_09_PA_CONFIG                           0x09
#define SX1276_REG_0A_PA_RAMP                             0x0a
#define SX1276_REG_0B_OCP                                 0x0b
#define SX1276_REG_0C_LNA                                 0x0c
#define SX1276_REG_0D_FIFO_ADDR_PTR                       0x0d
#define SX1276_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define SX1276_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define SX1276_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define SX1276_REG_11_IRQ_FLAGS_MASK                      0x11
#define SX1276_REG_12_IRQ_FLAGS                           0x12
#define SX1276_REG_13_RX_NB_BYTES                         0x13
#define SX1276_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define SX1276_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define SX1276_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define SX1276_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define SX1276_REG_18_MODEM_STAT                          0x18
#define SX1276_REG_19_PKT_SNR_VALUE                       0x19
#define SX1276_REG_1A_PKT_RSSI_VALUE                      0x1a
#define SX1276_REG_1B_RSSI_VALUE                          0x1b
#define SX1276_REG_1C_HOP_CHANNEL                         0x1c
#define SX1276_REG_1D_MODEM_CONFIG1                       0x1d
#define SX1276_REG_1E_MODEM_CONFIG2                       0x1e
#define SX1276_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define SX1276_REG_20_PREAMBLE_MSB                        0x20
#define SX1276_REG_21_PREAMBLE_LSB                        0x21
#define SX1276_REG_22_PAYLOAD_LENGTH                      0x22
#define SX1276_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define SX1276_REG_24_HOP_PERIOD                          0x24
#define SX1276_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define SX1276_REG_26_MODEM_CONFIG3                       0x26

#define SX1276_REG_27_PPM_CORRECTION                      0x27
#define SX1276_REG_28_FEI_MSB                             0x28
#define SX1276_REG_29_FEI_MID                             0x29
#define SX1276_REG_2A_FEI_LSB                             0x2a
#define SX1276_REG_2C_RSSI_WIDEBAND                       0x2c
#define SX1276_REG_31_DETECT_OPTIMIZ                      0x31
#define SX1276_REG_33_INVERT_IQ                           0x33
#define SX1276_REG_37_DETECTION_THRESHOLD                 0x37
#define SX1276_REG_39_SYNC_WORD                           0x39

#define SX1276_REG_40_DIO_MAPPING1                        0x40
#define SX1276_REG_41_DIO_MAPPING2                        0x41
#define SX1276_REG_42_VERSION                             0x42

#define SX1276_REG_4B_TCXO                                0x4b
#define SX1276_REG_4D_PA_DAC                              0x4d
#define SX1276_REG_5B_FORMER_TEMP                         0x5b
#define SX1276_REG_61_AGC_REF                             0x61
#define SX1276_REG_62_AGC_THRESH1                         0x62
#define SX1276_REG_63_AGC_THRESH2                         0x63
#define SX1276_REG_64_AGC_THRESH3                         0x64

// SX1276_REG_01_OP_MODE                             0x01
#define SX1276_LONG_RANGE_MODE                       0x80
#define SX1276_ACCESS_SHARED_REG                     0x40
#define SX1276_LOW_FREQUENCY_MODE                    0x08
#define SX1276_MODE                                  0x07
#define SX1276_MODE_SLEEP                            0x00
#define SX1276_MODE_STDBY                            0x01
#define SX1276_MODE_FSTX                             0x02
#define SX1276_MODE_TX                               0x03
#define SX1276_MODE_FSRX                             0x04
#define SX1276_MODE_RXCONTINUOUS                     0x05
#define SX1276_MODE_RXSINGLE                         0x06
#define SX1276_MODE_CAD                              0x07

// SX1276_REG_09_PA_CONFIG                           0x09
#define SX1276_PA_SELECT                             0x80
#define SX1276_MAX_POWER                             0x70
#define SX1276_OUTPUT_POWER                          0x0f

// SX1276_REG_0A_PA_RAMP                             0x0a
#define SX1276_LOW_PN_TX_PLL_OFF                     0x10
#define SX1276_PA_RAMP                               0x0f
#define SX1276_PA_RAMP_3_4MS                         0x00
#define SX1276_PA_RAMP_2MS                           0x01
#define SX1276_PA_RAMP_1MS                           0x02
#define SX1276_PA_RAMP_500US                         0x03
#define SX1276_PA_RAMP_250US                         0x0
#define SX1276_PA_RAMP_125US                         0x05
#define SX1276_PA_RAMP_100US                         0x06
#define SX1276_PA_RAMP_62US                          0x07
#define SX1276_PA_RAMP_50US                          0x08
#define SX1276_PA_RAMP_40US                          0x09  // par defaut sur reset
#define SX1276_PA_RAMP_31US                          0x0a
#define SX1276_PA_RAMP_25US                          0x0b
#define SX1276_PA_RAMP_20US                          0x0c
#define SX1276_PA_RAMP_15US                          0x0d
#define SX1276_PA_RAMP_12US                          0x0e
#define SX1276_PA_RAMP_10US                          0x0f

// SX1276_REG_0B_OCP                                 0x0b  // Imax 
#define SX1276_OCP_ON                                0x20  // 0x00 = OFF
#define SX1276_OCP_TRIM                              0x1f  // sur 5 bits de 100 à 240 mA

// SX1276_REG_0C_LNA                                 0x0c
#define SX1276_LNA_GAIN                              0xe0
#define SX1276_LNA_GAIN_G1                           0x20 // gain Max
#define SX1276_LNA_GAIN_G2                           0x40
#define SX1276_LNA_GAIN_G3                           0x60                
#define SX1276_LNA_GAIN_G4                           0x80
#define SX1276_LNA_GAIN_G5                           0xa0
#define SX1276_LNA_GAIN_G6                           0xc0  // gain mini
#define SX1276_LNA_BOOST_LF                          0x18
#define SX1276_LNA_BOOST_LF_DEFAULT                  0x00
#define SX1276_LNA_BOOST_HF                          0x03
#define SX1276_LNA_BOOST_HF_DEFAULT                  0x00
#define SX1276_LNA_BOOST_HF_150PC                    0x11

// SX1276_REG_11_IRQ_FLAGS_MASK                      0x11
#define SX1276_RX_TIMEOUT_MASK                       0x80
#define SX1276_RX_DONE_MASK                          0x40
#define SX1276_PAYLOAD_CRC_ERROR_MASK                0x20
#define SX1276_VALID_HEADER_MASK                     0x10
#define SX1276_TX_DONE_MASK                          0x08
#define SX1276_CAD_DONE_MASK                         0x04
#define SX1276_FHSS_CHANGE_CHANNEL_MASK              0x02
#define SX1276_CAD_DETECTED_MASK                     0x01

// SX1276_REG_12_IRQ_FLAGS                           0x12
#define SX1276_RX_TIMEOUT                            0x80
#define SX1276_RX_DONE                               0x40
#define SX1276_PAYLOAD_CRC_ERROR                     0x20
#define SX1276_VALID_HEADER                          0x10
#define SX1276_TX_DONE                               0x08
#define SX1276_CAD_DONE                              0x04
#define SX1276_FHSS_CHANGE_CHANNEL                   0x02
#define SX1276_CAD_DETECTED                          0x01

// SX1276_REG_18_MODEM_STAT                          0x18
#define SX1276_RX_CODING_RATE                        0xe0
#define SX1276_MODEM_STATUS_CLEAR                    0x10
#define SX1276_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define SX1276_MODEM_STATUS_RX_ONGOING               0x04
#define SX1276_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define SX1276_MODEM_STATUS_SIGNAL_DETECTED          0x01

// SX1276_REG_1C_HOP_CHANNEL                         0x1c
#define SX1276_PLL_TIMEOUT                           0x80
#define SX1276_RX_PAYLOAD_CRC_IS_ON                  0x40
#define SX1276_FHSS_PRESENT_CHANNEL                  0x3f

// SX1276_REG_1D_MODEM_CONFIG1                       0x1d
#define SX1276_BW                                    0xf0

#define SX1276_BW_7_8KHZ                             0x00
#define SX1276_BW_10_4KHZ                            0x10
#define SX1276_BW_15_6KHZ                            0x20  // a essayer en long range avec XTAL
#define SX1276_BW_20_8KHZ                            0x30
#define SX1276_BW_31_25KHZ                           0x40
#define SX1276_BW_41_7KHZ                            0x50
#define SX1276_BW_62_5KHZ                            0x60
#define SX1276_BW_125KHZ                             0x70
#define SX1276_BW_250KHZ                             0x80
#define SX1276_BW_500KHZ                             0x90
#define SX1276_CODING_RATE                           0x0e
#define SX1276_CODING_RATE_4_5                       0x02
#define SX1276_CODING_RATE_4_6                       0x04
#define SX1276_CODING_RATE_4_7                       0x06
#define SX1276_CODING_RATE_4_8                       0x08  //*
#define SX1276_IMPLICIT_HEADER_MODE_ON               0x01  // à mettre OFF

// SX1276_REG_1E_MODEM_CONFIG2                       0x1e
#define SX1276_SPREADING_FACTOR                      0xf0
#define SX1276_SPREADING_FACTOR_64CPS                0x60
#define SX1276_SPREADING_FACTOR_128CPS               0x70
#define SX1276_SPREADING_FACTOR_256CPS               0x80
#define SX1276_SPREADING_FACTOR_512CPS               0x90  //*
#define SX1276_SPREADING_FACTOR_1024CPS              0xa0
#define SX1276_SPREADING_FACTOR_2048CPS              0xb0
#define SX1276_SPREADING_FACTOR_4096CPS              0xc0
#define SX1276_TX_CONTINUOUS_MOE                     0x08

#define SX1276_PAYLOAD_CRC_ON                        0x04
#define SX1276_SYM_TIMEOUT_MSB                       0x03

// SX1276_REG_4B_TCXO                                0x4b
#define SX1276_TCXO_TCXO_INPUT_ON                    0x10

// SX1276_REG_4D_PA_DAC                              0x4d
#define SX1276_PA_DAC_DISABLE                        0x04
#define SX1276_PA_DAC_ENABLE                         0x07

/////////////////////////////////////////////////////////////////////
class SX1276 : public RHSPIDriver
{
public:
     /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate registers
    /// to set the desired spreading factor, coding rate and bandwidth
/*	
    typedef struct
    {
	uint8_t    reg_1d;   ///< Value for register SX1276_REG_1D_MODEM_CONFIG1
	uint8_t    reg_1e;   ///< Value for register SX1276_REG_1E_MODEM_CONFIG2
	uint8_t    reg_26;   ///< Value for register SX1276_REG_26_MODEM_CONFIG3
    } ModemConfig;
*/  
    /// Choices for setModemConfig() for a selected subset of common
    /// data rates. If you need another configuration,
    /// determine the necessary settings and call setModemRegisters() with your
    /// desired settings. It might be helpful to use the LoRa calculator mentioned in 
    /// http://www.semtech.com/images/datasheet/LoraDesignGuide_STD.pdf
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// Caution: if you are using slow packet rates and long packets with RHReliableDatagram or subclasses
    /// you may need to change the RHReliableDatagram timeout for reliable operations.
    /// Caution: for some slow rates nad with ReliableDatagrams youi may need to increase the reply timeout 
    /// with manager.setTimeout() to
    /// deal with the long transmission times.
/*	
    typedef enum
    {
	// Bw31_5Cr48Sf512 = 0,  ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range 
	// Bw500Cr45Sf128,	      ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
	// Bw31_25Cr48Sf512,	  ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	// Bw125Cr48Sf4096,      ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
	// Bw125Cr45Sf128 ,      ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
    Bw31_25Cr45Sf128 = 0,
    Bw31_25Cr46Sf128,
    Bw31_25Cr47Sf128,
    Bw31_25Cr48Sf128,
    Bw31_25Cr45Sf256,
    Bw31_25Cr46Sf256,
    Bw31_25Cr47Sf256,
    Bw31_25Cr48Sf256,
    Bw31_25Cr45Sf512,
    Bw31_25Cr46Sf512,
    Bw31_25Cr47Sf512,
    Bw31_25Cr48Sf512,
    Bw31_25Cr48Sf1024,
    Bw125Cr45Sf128  ,
    Bw500Cr45Sf128  ,
    Bw125Cr48Sf4096 ,
	
 } ModemConfigChoice;
*/
    /// Constructor. You can have multiple instances, but each instance must have its own
    /// interrupt and slave select pin. After constructing, you must call init() to initialise the interface
    /// and the radio module. A maximum of 3 instances can co-exist on one processor, provided there are sufficient
    /// distinct interrupt lines, one for each instance.
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RH_RF22 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega, D10 for Maple)
    /// \param[in] interruptPin The interrupt Pin number that is connected to the RFM DIO0 interrupt line. 
    /// Defaults to pin 2, as required by Anarduino MinWirelessLoRa module.
    /// Caution: You must specify an interrupt capable pin.
    /// On many Arduino boards, there are limitations as to which pins may be used as interrupts.
    /// On Leonardo pins 0, 1, 2 or 3. On Mega2560 pins 2, 3, 18, 19, 20, 21. On Due and Teensy, any digital pin.
    /// On Arduino Zero from arduino.cc, any digital pin other than 4.
    /// On Arduino M0 Pro from arduino.org, any digital pin other than 2.
    /// On other Arduinos pins 2 or 3. 
    /// See http://arduino.cc/en/Reference/attachInterrupt for more details.
    /// On Chipkit Uno32, pins 38, 2, 7, 8, 35.
    /// On other boards, any digital pin may be used.
    /// \param[in] spi Pointer to the SPI interface object to use. 
    ///                Defaults to the standard Arduino hardware SPI interface
    SX1276(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, RHGenericSPI& spi = hardware_spi);
  
    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Prints the value of all chip registers
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging purposes only.
    /// \return true on success
    bool printRegisters();

    /// Sets all the registered required to configure the data modem in the SX1276/78, including the bandwidth, 
    /// spreading factor etc. You can use this to configure the modem with custom configurations if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
//    void           setModemRegisters(const ModemConfig* config);

    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig.
    /// \param[in] index The configuration choice.
    /// \return true if index is a valid choice.
	
 //   bool        setModemConfig(ModemConfigChoice index);

    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into ModeRx mode until
    /// a message is actually received by the transport, when it wil be returned to ModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then optionally waits for Channel Activity Detection (CAD) 
    /// to show the channnel is clear (if the radio supports CAD) by calling waitCAD().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// specify the maximum time in ms to wait. If 0 (the default) do not wait for CAD before transmitting.
    /// \return true if the message length was valid and it was correctly queued for transmit. Return false
    /// if CAD was requested and the CAD timeout timed out before clear channel was detected.
    virtual bool    send(const uint8_t* data, uint8_t len);

    /// Sets the length of the preamble
    /// in bytes. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 8.
    /// Sets the message preamble length in SX1276_REG_??_PREAMBLE_?SB
    /// \param[in] bytes Preamble length in bytes.  
    void           setPreambleLength(uint16_t bytes);

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();

    /// Sets the transmitter and receiver 
    /// centre frequency.
    /// \param[in] centre Frequency in MHz. 137.0 to 1020.0. Caution: SX1276/78/79 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    /// \return true if the selected frquency centre is within range
    bool        setFrequency(float centre);

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF95/96/97/98.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF95/96/97/98.
    void           setModeTx();

    /// Sets the transmitter power output level, and configures the transmitter pin.
    /// Be a good neighbour and set the lowest power level you need.
    /// Some SX1276/77/78/79 and compatible modules (such as RFM95/96/97/98) 
    /// use the PA_BOOST transmitter pin for high power output (and optionally the PA_DAC)
    /// while some (such as the Modtronix inAir4 and inAir9) 
    /// use the RFO transmitter pin for lower power but higher efficiency.
    /// You must set the appropriate power level and useRFO argument for your module.
    /// Check with your module manufacturer which transmtter pin is used on your module
    /// to ensure you are setting useRFO correctly. 
    /// Failure to do so will result in very low 
    /// transmitter power output.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm, with useRFO false (ie PA_BOOST enabled).
    /// \param[in] power Transmitter power level in dBm. For RFM95/96/97/98 LORA with useRFO false, 
    /// valid values are from +5 to +23.
    /// For Modtronix inAir4 and inAir9 with useRFO true (ie RFO pins in use), 
    /// valid values are from -1 to 14.
    /// \param[in] useRFO If true, enables the use of the RFO transmitter pins instead of
    /// the PA_BOOST pin (false). Choose the correct setting for your module.
    void           setTxPower(int8_t power, bool useRFO = false);
	
    ///  je rajoute la possibilité de mettre la protection courant du PA
    void           setPAprotect(int8_t power);	

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    /// \return true if sleep mode was successfully entered.
    virtual bool    sleep();


    /// Use the radio's Channel Activity Detect (CAD) function to detect channel activity.
    /// Sets the RF95 radio into CAD mode and waits until CAD detection is complete.
    /// To be used in a listen-before-talk mechanism (Collision Avoidance)
    /// with a reasonable time backoff algorithm.
    /// This is called automatically by waitCAD().
    /// \return true if channel is in use.  
    virtual bool    isChannelActive();

    /// Enable TCXO mode
    /// Call this immediately after init(), to force your radio to use an external 
    /// frequency source, such as a Temperature Compensated Crystal Oscillator (TCXO).
    /// See the comments in the main documentation about the sensitivity of this radio to
    /// clock frequency especially when using narrow bandwidths.
    /// Leaves the module in sleep mode.
    /// Caution, this function has not been tested by us.
    void enableTCXO();

    /// Returns the last measured frequency error.
    /// The LoRa receiver estimates the frequency offset between the receiver centre frequency
    /// and that of the received LoRa signal. This function returns the estimates offset (in Hz) 
    /// of the last received message. Caution: this measurement is not absolute, but is measured 
    /// relative to the local receiver's oscillator. 
    /// Apparent errors may be due to the transmitter, the receiver or both.
    /// \return The estimated centre frequency offset in Hz of the last received message. 
    /// If the modem bandwidth selector in 
    /// register SX1276_REG_1D_MODEM_CONFIG1 is invalid, returns 0.
    int frequencyError();

    /// Returns the Signal-to-noise ratio (SNR) of the last received message, as measured
    /// by the receiver.
    /// \return SNR of the last received message in dB
    int lastSNR();

protected:
    /// This is a low level function to handle the interrupts for one instance of SX1276.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void           handleInterrupt();

    /// Examine the revceive buffer to determine whether the message is for this node
    void validateRxBuf();

    /// Clear our local receive buffer
    void clearRxBuf();

private:
    /// Low level interrupt service routine for device connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static SX1276*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// Number of octets in the buffer
    volatile uint8_t    _bufLen;
    
    /// The receiver/transmitter buffer
    uint8_t             _buf[SX1276_MAX_PAYLOAD_LEN];

    /// True when there is a valid message in the buffer
    volatile bool       _rxBufValid;

    // True if we are using the HF port (779.0 MHz and above)
    bool                _usingHFport;

    // Last measured SNR, dB
    int8_t              _lastSNR;
};

#endif

