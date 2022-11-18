uint16 LeftMotor_CtrlWord
int32 LeftMotor_Speed
int32 LeftMotor_Encoder
uint16 LeftMotor_Status
uint8 LeftMotor_Error
uint32 LeftMotor_ErrorHist1
uint32 LeftMotor_ErrorHist2
int32 LeftMotor_Current
uint16 LeftMotor_I2t_Motor
uint16 LeftMotor_I2t_Controller
int16 LeftMotor_Temperature
uint16 RightMotor_CtrlWord
int32 RightMotor_Speed
int32 RightMotor_Encoder
uint16 RightMotor_Status
uint8 RightMotor_Error
uint32 RightMotor_ErrorHist1
uint32 RightMotor_ErrorHist2
int32 RightMotor_Current
uint16 RightMotor_I2t_Motor
uint16 RightMotor_I2t_Controller
int16 RightMotor_Temperature
#Status bits for brake feedback.
uint8 BRAKE_STATUS_BRAKED_BIT=1		# is "1" if brake is supposed to be braked
uint8 BRAKE_STATUS_FB_BIT=4			# is "1" if brake feedback sensor is activated (brake is released)
uint8 BRAKE_STATUS_TRANSITION_BIT=128
# So error combinations are - (Any combination with the TRANSITION bit set are valid)
#  BRAKED	FB	TRANSITION	STATUSWORD
#	0		0	0			0x00			Brake is suppused to be released, but FB indicate braked. We are not in transition.
#	1		1	0			0x05			Brake is suppused to be braked, but FB indicate released. We are not in transition.
uint8 Brake_LeftStatus
uint8 Brake_RightStatus
