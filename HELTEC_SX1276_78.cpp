// SX1276.cpp
//
// Copyright (C) 2017 Robert Pré

#include <HELTEC_SX1276_78.h>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of SX1276, allowing you to have
// 2 or more LORAs per Arduino
SX1276* SX1276::_deviceForInterrupt[SX1276_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t SX1276::_interruptCount = 0; // Index into _deviceForInterrupt for next device

/*
// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
PROGMEM static const SX1276::ModemConfig MODEM_CONFIG_TABLE[] =
{
// 4 premiers bits de 1d donnent Bw : 0 = 7,8 KHz, 1 = 10.4, 2 = 15.6, 3 = 20.8, 4 = 31.25, 5 = 41.7, 6 =62.5, 7 = 125... 9 = 500KHz
// 3   bits  suivant de 1d donnent 001 = 4/5, 010= 4/6, 011=4/7, 100 = 4/8 
// le dernier bit de 1d donne 0 = explicit mode, 1 implicit mode
 
// 4 premiers bits de 1e donnent SF : 6 = 64, 7 = 128, ... 8 = 256... 9 = 512
   //  1d,     1e,      26   
    { 0x42,   0x74,    0x00}, // = 0  // Bw31_25Cr45Sf128  
    { 0x44,   0x74,    0x00}, // = 1  // Bw31_25Cr46Sf128 
    { 0x46,   0x74,    0x00}, // = 2  // Bw31_25Cr47Sf128 
    { 0x48,   0x74,    0x00}, // = 3  // Bw31_25Cr48Sf128 
	
    { 0x42,   0x84,    0x00}, // = 4  // Bw31_25Cr45Sf256
    { 0x44,   0x84,    0x00}, // = 5  // Bw31_25Cr46Sf256 
    { 0x46,   0x84,    0x00}, // = 6  // Bw31_25Cr47Sf256
    { 0x48,   0x84,    0x00}, // = 7  // Bw31_25Cr48Sf256
	
    { 0x42,   0x94,    0x00}, // = 8  // Bw31_25Cr45Sf512
    { 0x44,   0x94,    0x00}, // = 9  // Bw31_25Cr46Sf512
    { 0x46,   0x94,    0x00}, // = 10 // Bw31_25Cr47Sf512
    { 0x48,   0x94,    0x00}, // = 11 // Bw31_25Cr48Sf512

    { 0x48,   0xA4,    0x00}, // = 12 // Bw31_25Cr48Sf1024
    { 0x72,   0x74,    0x00}, // = 13 // Bw125Cr45Sf128 
    { 0x92,   0x74,    0x00}, // = 14 // Bw500Cr45Sf128
    { 0x78,   0xC4,    0x00}, // = 15 // Bw125Cr48Sf4096
     
};
*/

SX1276::SX1276(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHSPIDriver(slaveSelectPin, spi),
    _rxBufValid(0)
{
    _interruptPin = interruptPin;
    _myInterruptIndex = 0xff; // Not allocated yet
}

bool SX1276::init()
{
    if (!RHSPIDriver::init())
	return false;

    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_interruptPin);
    if (interruptNumber == NOT_AN_INTERRUPT)
	return false;
#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
#endif

    // pas de possibilité de determiner le type du Circuit :-(
    
    // Set sleep mode, so we can also set LORA mode:
    spiWrite(SX1276_REG_01_OP_MODE, SX1276_MODE_SLEEP | SX1276_LONG_RANGE_MODE);
    delay(10); // Wait for sleep mode to take over from say, CAD
    // Check we are in sleep mode, with LORA set
    if (spiRead(SX1276_REG_01_OP_MODE) != (SX1276_MODE_SLEEP | SX1276_LONG_RANGE_MODE))
    {
	Serial.println(spiRead(SX1276_REG_01_OP_MODE), HEX);
	return false; // No device present?
    }

    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 

    // Set up interrupt handler
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= SX1276_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return false; // Too many devices, not enough interrupt vectors
    }
    _deviceForInterrupt[_myInterruptIndex] = this;
    if (_myInterruptIndex == 0)
	attachInterrupt(interruptNumber, isr0, RISING);
    else if (_myInterruptIndex == 1)
	attachInterrupt(interruptNumber, isr1, RISING);
    else if (_myInterruptIndex == 2)
	attachInterrupt(interruptNumber, isr2, RISING);
    else
	return false; // Too many devices, not enough interrupt vectors

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    spiWrite(SX1276_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(SX1276_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets

    setModeIdle();

    // Set up  configuration par défaut sinon on prends ce qui vient du programme
    // No Sync Words in LORA mode.
	
 //   setModemConfig(7); // par default  = Bw31_25Cr48Sf256 
	
    setPreambleLength(6); // Default is 8

    setFrequency(439.995); // fréquence par defaut

    setTxPower(10); // par defaut 10mW

    return true;
}

// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly 
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void SX1276::handleInterrupt()
{
    // Read the interrupt register
    uint8_t irq_flags = spiRead(SX1276_REG_12_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (SX1276_RX_TIMEOUT | SX1276_PAYLOAD_CRC_ERROR))
    {
	_rxBad++;
    }
    else if (_mode == RHModeRx && irq_flags & SX1276_RX_DONE)
    {
	// Have received a packet
	uint8_t len = spiRead(SX1276_REG_13_RX_NB_BYTES);

	// Reset the fifo read ptr to the beginning of the packet
	spiWrite(SX1276_REG_0D_FIFO_ADDR_PTR, spiRead(SX1276_REG_10_FIFO_RX_CURRENT_ADDR));
	spiBurstRead(SX1276_REG_00_FIFO, _buf, len);
	_bufLen = len;
	spiWrite(SX1276_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

	// Remember the last signal to noise ratio, LORA mode
	// Per page 111, SX1276/77/78/79 datasheet
	_lastSNR = (int8_t)spiRead(SX1276_REG_19_PKT_SNR_VALUE) / 4;

	// Remember the RSSI of this packet, LORA mode
	// this is according to the doc, but is it really correct?
	// weakest receiveable signals are reported RSSI at about -66

	 _lastRssi = spiRead(SX1276_REG_1A_PKT_RSSI_VALUE);
	// Adjust the RSSI, datasheet page 87
	// if (_lastSNR < 0)
	    // _lastRssi = _lastRssi + _lastSNR;
	// else
	    // _lastRssi = (int)_lastRssi * 16 / 15;
	// if (_usingHFport)
	    // _lastRssi -= 157;
	// else
	    // _lastRssi -= 164;
	    
	// We have received a message.
	validateRxBuf(); 
	if (_rxBufValid)
	    setModeIdle(); // Got one 
    }
    else if (_mode == RHModeTx && irq_flags & SX1276_TX_DONE)
    {
	_txGood++;
	setModeIdle();
    }
    else if (_mode == RHModeCad && irq_flags & SX1276_CAD_DONE)
    {
        _cad = irq_flags & SX1276_CAD_DETECTED;
        setModeIdle();
    }
    
    spiWrite(SX1276_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

// These are low level functions that call the interrupt handler for the correct
// instance of SX1276.
// 3 interrupts allows us to have 3 different devices
void SX1276::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void SX1276::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void SX1276::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

// Check whether the latest received message is complete and uncorrupted
void SX1276::validateRxBuf()
{
    if (_bufLen < 4)
	return; // Too short to be a real message
    // Extract the 4 headers
    _rxHeaderTo    = _buf[0];
    _rxHeaderFrom  = _buf[1];
    _rxHeaderId    = _buf[2];
    _rxHeaderFlags = _buf[3];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

bool SX1276::available()
{
    if (_mode == RHModeTx)
	return false;
    setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void SX1276::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}

bool SX1276::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-SX1276_HEADER_LEN)
	    *len = _bufLen-SX1276_HEADER_LEN;
	memcpy(buf, _buf+SX1276_HEADER_LEN, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

bool SX1276::send(const uint8_t* data, uint8_t len)
{
    if (len > SX1276_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Position at the beginning of the FIFO
    spiWrite(SX1276_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    spiWrite(SX1276_REG_00_FIFO, _txHeaderTo);
    spiWrite(SX1276_REG_00_FIFO, _txHeaderFrom);
    spiWrite(SX1276_REG_00_FIFO, _txHeaderId);
    spiWrite(SX1276_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWrite(SX1276_REG_00_FIFO, data, len);
    spiWrite(SX1276_REG_22_PAYLOAD_LENGTH, len + SX1276_HEADER_LEN);

    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

bool SX1276::printRegisters()
{
#ifdef RH_HAVE_SERIAL
//    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a};
    uint8_t i;
	uint8_t val_reg;
	
 //   for (i = 0; i < sizeof(registers); i++)
    for (i = 0; i < 113; i++)
    {
	if (i< 16) Serial.print("0");
	Serial.print(i, HEX);
	Serial.print(": ");
	val_reg = spiRead(i);
	if (val_reg < 16) Serial.print("0");
	Serial.print(val_reg, HEX); Serial.print(" "); Serial.println(val_reg, BIN);
    }
#endif
    return true;
}

uint8_t SX1276::maxMessageLength()
{
    return SX1276_MAX_MESSAGE_LEN;
}

bool SX1276::setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / SX1276_FSTEP;
    spiWrite(SX1276_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(SX1276_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(SX1276_REG_08_FRF_LSB, frf & 0xff);
    _usingHFport = (centre >= 779.0);

    return true;
}

void SX1276::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	spiWrite(SX1276_REG_01_OP_MODE, SX1276_MODE_STDBY);
	_mode = RHModeIdle;
    }
}

bool SX1276::sleep()
{
    if (_mode != RHModeSleep)
    {
	spiWrite(SX1276_REG_01_OP_MODE, SX1276_MODE_SLEEP);
	_mode = RHModeSleep;
    }
    return true;
}

void SX1276::setModeRx()
{
    if (_mode != RHModeRx)
    {
	spiWrite(SX1276_REG_01_OP_MODE, SX1276_MODE_RXCONTINUOUS);
	spiWrite(SX1276_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
	_mode = RHModeRx;
    }
}

void SX1276::setModeTx()
{
    if (_mode != RHModeTx)
    {
	spiWrite(SX1276_REG_01_OP_MODE, SX1276_MODE_TX);
	spiWrite(SX1276_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
	_mode = RHModeTx;
    }
}

void SX1276::setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
	if (power > 14)
	    power = 14;
	if (power < -1)
	    power = -1;
	spiWrite(SX1276_REG_09_PA_CONFIG, SX1276_MAX_POWER | (power + 1));
    }
    else
    {
	if (power > 23)
	    power = 23;
	if (power < 5)
	    power = 5;

	// For SX1276_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
	// SX1276_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
	// for 21, 22 and 23dBm
	if (power > 20)
	{
	    spiWrite(SX1276_REG_4D_PA_DAC, SX1276_PA_DAC_ENABLE);
	    power -= 3;
	}
	else
	{
	    spiWrite(SX1276_REG_4D_PA_DAC, SX1276_PA_DAC_DISABLE);
	}

	// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
	// pin is connected, so must use PA_BOOST
	// Pout = 2 + OutputPower.
	// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
	// but OutputPower claims it would be 17dBm.
	// My measurements show 20dBm is correct
	spiWrite(SX1276_REG_09_PA_CONFIG, SX1276_PA_SELECT | (power-5));
    }
}

void SX1276::setPAprotect(int8_t OCP)
{
	    spiWrite(SX1276_REG_0B_OCP,OCP);
}

/*
// Sets registers from a canned modem configuration structure
void SX1276::setModemRegisters(const ModemConfig* config)
{
    spiWrite(SX1276_REG_1D_MODEM_CONFIG1,       config->reg_1d);
    spiWrite(SX1276_REG_1E_MODEM_CONFIG2,       config->reg_1e);
    spiWrite(SX1276_REG_26_MODEM_CONFIG3,       config->reg_26);
}


// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool SX1276::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(SX1276::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}
*/

void SX1276::setPreambleLength(uint16_t bytes)
{
    spiWrite(SX1276_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(SX1276_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

bool SX1276::isChannelActive()
{
    // Set mode RHModeCad
    if (_mode != RHModeCad)
    {
        spiWrite(SX1276_REG_01_OP_MODE, SX1276_MODE_CAD);
        spiWrite(SX1276_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
        _mode = RHModeCad;
    }

    while (_mode == RHModeCad)
        YIELD;

    return _cad;
}

void SX1276::enableTCXO()
{
    while ((spiRead(SX1276_REG_4B_TCXO) & SX1276_TCXO_TCXO_INPUT_ON) != SX1276_TCXO_TCXO_INPUT_ON)
    {
	sleep();
	spiWrite(SX1276_REG_4B_TCXO, (spiRead(SX1276_REG_4B_TCXO) | SX1276_TCXO_TCXO_INPUT_ON));
    } 
}

// From section 4.1.5 of SX1276/77/78/79
// Ferror = FreqError * 2**24 * BW / Fxtal / 500
int SX1276::frequencyError()
{
    int32_t freqerror = 0;

    // Convert 2.5 bytes (5 nibbles, 20 bits) to 32 bit signed int
    freqerror = spiRead(SX1276_REG_28_FEI_MSB) << 16;
    freqerror |= spiRead(SX1276_REG_29_FEI_MID) << 8;
    freqerror |= spiRead(SX1276_REG_2A_FEI_LSB);
    // Sign extension into top 3 nibbles
    if (freqerror & 0x80000)
	freqerror |= 0xfff00000;

    int error = 0; // In hertz
    float bw_tab[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
    uint8_t bwindex = spiRead(SX1276_REG_1D_MODEM_CONFIG1) >> 4;
    if (bwindex < (sizeof(bw_tab) / sizeof(float)))
	error = (float)freqerror * bw_tab[bwindex] * ((float)(1L << 24) / (float)SX1276_FXOSC / 500.0);
    // else not defined

    return error;
}

int SX1276::lastSNR()
{
    return _lastSNR;
}
