/*!
 * \author no1wudi
 * \file LIS3DSH.c
 */

#include "LIS3DSH.h"

/*!
 * \brief Youc can edit your own init process here.
 */

void LIS3DSH_Init(void) {
	LIS3DSH_Init_SPI_Bus();
}

/*!
 * \brief Read the content of the register
 * \param LIS3DSH_Reg The address of the object register.
 */

unsigned char LIS3DSH_Read(unsigned char LIS3DSH_Reg) {
	unsigned char Temp;
	CS_L;
	LIS3DSH_SPI_Read_Write(LIS3DSH_Reg | LIS3DSH_Read_Mask);
	Temp = LIS3DSH_SPI_Read_Write(0xAA);
	CS_H;
	return Temp;
}

/*!
 *\brief Write data to the register.
 *\param LIS3DSH_Reg The object register.
 *\param Data The data will be written.
 */

void LIS3DSH_Write(unsigned char LIS3DSH_Reg, unsigned char Data) {
	CS_L;
	LIS3DSH_SPI_Read_Write(LIS3DSH_Reg | LIS3DSH_Write_Mask);
	LIS3DSH_SPI_Read_Write(Data);
	CS_H;
}

/*!
 * \brief Set the output status.
 * \param LIS3DSH_Output The setting of the output.
 */

void LIS3DSH_Set_Output(unsigned char LIS3DSH_Output) {
	LIS3DSH_Write(LIS3DSH_Reg_Ctrl_4, LIS3DSH_Output);
}

/*!
 * \brief Set The output feature.
 * \param LIS3DSH_Output_Feature The feature of output.
 */
void LIS3DSH_Set_Output_Feature(unsigned char LIS3DSH_Output_Feature) {
	LIS3DSH_Write(LIS3DSH_Reg_Ctrl_5, LIS3DSH_Output_Feature);
}

/*!
 * \brief Get the output value of X-axis.
 * \param LIS3DSH_Sense Sensitivity of the sensor.
 * \return The value of x-axis
 */

float LIS3DSH_Get_X_Out(float LIS3DSH_Sense) {
	short Temp;

	Temp = LIS3DSH_Read(LIS3DSH_Reg_X_Out_H);
	Temp = Temp << 8;
	Temp = Temp + LIS3DSH_Read(LIS3DSH_Reg_X_Out_L);
	return (float) Temp * LIS3DSH_Sense / 1000;
}

/*!
 * \brief Get the output value of y-axis.
 * \param LIS3DSH_Sense Sensitivity of the sensor.
 * \return The value of y-axis
 */
float LIS3DSH_Get_Y_Out(float LIS3DSH_Sense) {
	short Temp;

	Temp = LIS3DSH_Read(LIS3DSH_Reg_Y_Out_H);
	Temp = Temp << 8;
	Temp = Temp + LIS3DSH_Read(LIS3DSH_Reg_Y_Out_L);
	return (float) Temp * LIS3DSH_Sense / 1000;
}

/*!
 * \brief Get the output value of Z-axis.
 * \param LIS3DSH_Sense Sensitivity of the sensor.
 * \return The value of Z-axis
 */

float LIS3DSH_Get_Z_Out(float LIS3DSH_Sense) {
	short Temp;

	Temp = LIS3DSH_Read(LIS3DSH_Reg_Z_Out_H);
	Temp = Temp << 8;
	Temp = Temp + LIS3DSH_Read(LIS3DSH_Reg_Z_Out_L);
	return (float) Temp * LIS3DSH_Sense / 1000;
}
