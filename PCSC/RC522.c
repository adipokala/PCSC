#include "RC522.h"

unsigned char SPI_txrx (unsigned char byte)
{
 SPI_I2S_ClearFlag(SPI2,SPI_I2S_FLAG_RXNE);
//while (SPI_I2S_GetFlagStatus (SPI2, SPI_I2S_FLAG_TXE) == RESET);
 SPI_SendData8 (SPI1, byte);
 while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);  // wait here till byte is actually shifted out
    return (SPI_ReceiveData8(SPI1));
 return 0;
}

// while calling this function, use only register defined names
unsigned char PCD_ReadReg (unsigned char regname)
{
 unsigned char ch1;
// ch1 = regname << 1;
 ch1 = regname | 0x80;  // MSB = 1 for reading
 GPIO_ResetBits (GPIOA, GPIO_Pin_1);		// make CS low
 SPI_txrx (ch1);
 ch1 = SPI_txrx (0);						// dummy write
 GPIO_SetBits (GPIOA, GPIO_Pin_1);
 return ch1;
}

// while calling this function, use only register defined names
void PCD_ReadRegs (unsigned char regname, unsigned char count, unsigned char buf[],unsigned char RxAlign)
{
 unsigned char index;
 unsigned char mask;
 unsigned char i;
 unsigned char ch;
 if (count == 0) return;
 GPIO_ResetBits (GPIOA, GPIO_Pin_1);		// make CS low
 ch = regname | 0x80;
 SPI_txrx (ch);
 count--;			// one read will be done outside the loop
 while (index < count)
 {
  if (index == 0 && RxAlign)
  {
	  mask = 0;
	  for (i=RxAlign; i<= 7; i++)
		mask |=  (1 << i);
	  i = SPI_txrx (ch);
	  buf[0] = (buf[index] & ~mask) | (i & mask);
    }
   else
	buf[index] =  SPI_txrx (ch);
   index++;
  }
 buf[index] = SPI_txrx (0);
 GPIO_SetBits (GPIOA, GPIO_Pin_1);
}

// while calling this function, use only register defined names
void PCD_WriteReg (unsigned char regname, unsigned char data)
{
 //unsigned char ch1;
 //ch1 = regname << 1;
 // MSB = 0 for writing
 GPIO_ResetBits (GPIOA, GPIO_Pin_1);		// make CS low
 SPI_txrx (regname);
 SPI_txrx (data);						// dummy write
 GPIO_SetBits (GPIOA, GPIO_Pin_1);
// return ch1;
}

// multiple write registers. while calling this function, use only register defined names
void PCD_WriteRegs (unsigned char regname, unsigned char count, unsigned char data[])
{
 unsigned char index;
// unsigned char ch1;
 //ch1 = regname << 1;
 GPIO_ResetBits (GPIOA, GPIO_Pin_1);		// make CS low
 SPI_txrx (regname);
 for (index=0; index < count; index++)
	 SPI_txrx (data[(int)index]);
 GPIO_SetBits (GPIOA, GPIO_Pin_1);
}

void init_pcd (void)
{
  GPIO_ResetBits (GPIOA, GPIO_Pin_3);   // 522 in reset
  Delay (30);
  GPIO_SetBits (GPIOA, GPIO_Pin_3);   // 522 out of reset
  Delay (20);

 // return;

  PCD_Reset ();
	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.

//    ClearBitMask(Status2Reg,0x08);			// disable cryto as per other code
//	PCD_WriteReg (TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteReg (TModeReg, 0x8D);			// as per other code

//	PCD_WriteReg (TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25mSec.
    PCD_WriteReg (TPrescalerReg, 0x3e);		// as per other code

	PCD_WriteReg (TReloadRegH, 0x00);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteReg (TReloadRegL, 0x30);  		//NPS - tripled the original timeout.
// added following
//    PCD_WriteReg (RxSelReg,0x86);
//	PCD_WriteReg(TxModeReg, 0x30);  		//Not needed. Default value suffices.
//	PCD_WriteReg(RFCfgReg, 0x7F);
	PCD_WriteReg (TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteReg (ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
//    PCD_WriteReg (ModeReg, 0x8D);		// as per other code
//    PCD_WriteReg (ModeReg, 0xA9);
	Delay (100);				// some delay before antenna is switched ON
    PCD_AntennaOn ();
}

void PCD_Reset (void)
{
  PCD_WriteReg (CommandReg,PCD_SoftReset); // Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74
  // Let us be generous: 50ms.
  Delay (150);   // After setting systick timer adjust this value
  while (PCD_ReadReg (CommandReg) & (1<<4));
}

void PCD_AntennaOn (void)
{
 unsigned char ch1;
 ch1 = PCD_ReadReg (TxControlReg);
 if ((ch1 & 3) != 3)
  PCD_WriteReg (TxControlReg, ch1|3);
}

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff(void)
{
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}


/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
unsigned char PCD_GetAntennaGain(void)
{
	return PCD_ReadReg (RFCfgReg) & (0x07<<4);
}

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void PCD_SetAntennaGain (unsigned char mask)
{
	if (PCD_GetAntennaGain() != mask)
	 {						// only bother if there is a change
		PCD_ClearRegisterBitMask (RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		PCD_SetRegisterBitMask (RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
	 }
}



void Delay(long nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}


/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PICC_REQA_or_WUPA (unsigned char command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
								 unsigned char *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								 unsigned char *bufferSize)	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
 {
	unsigned char validBits;
	unsigned char status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}

	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,0);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()


/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PCD_TransceiveData (unsigned char *sendData,		///< Pointer to the data to transfer to the FIFO.
								  unsigned char sendLen,		///< Number of bytes to transfer to the FIFO.
								  unsigned char *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
								  unsigned char *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
								  unsigned char *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
								  unsigned char rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
								  unsigned char checkCRC)		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
{
	unsigned char waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
 } // End PCD_TransceiveData()


/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PCD_CommunicateWithPICC(unsigned char command,		///< The command to execute. One of the PCD_Command enums.
									  unsigned char waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
									  unsigned char *sendData,		///< Pointer to the data to transfer to the FIFO.
									  unsigned char sendLen,		///< Number of bytes to transfer to the FIFO.
									  unsigned char *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
									  unsigned char *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									  unsigned char *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
									  unsigned char rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									  unsigned char checkCRC)		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
  {
	unsigned char n, _validBits;
	unsigned int i;
	unsigned char txLastBits;
	unsigned char bitFraming;
	unsigned char errorRegValue;
	unsigned char controlBuffer[2];
	unsigned char status;

	// Prepare values for BitFramingReg
	txLastBits = validBits ? *validBits : 0;
	bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteReg (CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteReg (ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegs (FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteReg (BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteReg (CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86?s.
	i = 10000;
	while (1) {
		n = PCD_ReadReg (ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			Mes2Usart1("PCD timeout\r\n");
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
		Delay(1000);
	}

	// Stop now if any errors except collisions were detected.
	errorRegValue = PCD_ReadReg (ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen)
	{
		n = PCD_ReadReg (FIFOLevelReg);			// Number of bytes in the FIFO
		if (n > *backLen)
			return STATUS_NO_ROOM;

		*backLen = n;											// Number of bytes returned
		PCD_ReadRegs (FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadReg (ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits)
		  *validBits = _validBits;
	 }

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
	//	byte controlBuffer[2];
		status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PICC_RequestA (unsigned char *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							 unsigned char *bufferSize)	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PICC_HaltA (void)
 {
	unsigned char result;
	unsigned char buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK)
		return result;

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData (buffer, sizeof(buffer), NULL, 0,0,0,0);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PICC_WakeupA(unsigned char *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
						   unsigned char *bufferSize)	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
{
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
unsigned char PCD_CalculateCRC(unsigned char *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
							   unsigned char length,	///< In: The number of bytes to transfer.
							   unsigned char *result)	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
 {
    int i = 5000;
	unsigned char n;
	PCD_WriteReg (CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteReg (DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegs (FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteReg (CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73?s.
	//word i = 5000;
	//byte n;
	while (1)
	{
	  n = PCD_ReadReg (DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
	  if (n & 0x04) 						// CRCIRq bit set - calculation done
			break;
	  if (--i == 0) 						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
		return STATUS_TIMEOUT;
      }

	PCD_WriteReg (CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.

	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadReg (CRCResultRegL);
	result[1] = PCD_ReadReg (CRCResultRegH);
	return STATUS_OK;
} // End PCD_CalculateCRC()
